"""
Project Helmet — Rider-check VLM agent
======================================

The ESP32-CAM POSTs a rider photo here; this agent asks a vision model
whether the rider is wearing a helmet and looks drowsy, cleans up the answer,
and returns it in the OpenAI chat-completions shape the camera already parses:

    POST /api/v1/chat/completions      (also /v1/chat/completions)
      Authorization: Bearer <AGENT_API_KEY>
      body: OpenAI-style messages containing one image_url part with a
            data:image/jpeg;base64,... URI  (exactly what HelmetCam.ino sends)

    -> choices[0].message.content is ALWAYS clean JSON:
       {"face_visible":true,"helmet":true,"drowsy":false,"eyes":"open",
        "confidence":0.87,"notes":"full-face helmet on, eyes open"}

    GET /health    no auth, for DigitalOcean health checks
    GET /last      last result (auth), handy while debugging

The prompt lives here on the server, so you can tune it without reflashing
the camera. Whatever system/user text the camera sends is ignored; only its
image is used.

Vision backends (VLM_BACKEND):
    anthropic  Claude via the Anthropic API              (default)
    openai     any OpenAI-compatible server: OpenAI, vLLM, LiteLLM, or
               Ollama on the same droplet (OPENAI_BASE_URL=http://localhost:11434/v1,
               OPENAI_MODEL=qwen2.5vl:7b)
    mock       fake answers, no keys needed; use it to test the whole
               helmet -> camera -> agent chain first

Run locally:
    pip install -r requirements.txt
    VLM_BACKEND=mock python agent.py
    python agent.py --test rider.jpg          # one image, no server

Every setting below can be overridden with an environment variable of the
same name. The hardcoded values are DUMMIES.
"""

from __future__ import annotations

import asyncio
import base64
import binascii
import hmac
import json
import logging
import os
import random
import sys
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import httpx
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

# ============================  CONFIG  ======================================
# Must match VLM_KEY in HelmetCam.ino.
AGENT_API_KEY = os.getenv("AGENT_API_KEY", "do-agent-key-DUMMY-3f9a1c7e5b2d8f4a6c0e9b1d7f3a5c8e")

VLM_BACKEND = os.getenv("VLM_BACKEND", "anthropic").lower()   # anthropic | openai | mock

ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY", "sk-ant-api03-DUMMY-replace-me")
ANTHROPIC_MODEL   = os.getenv("ANTHROPIC_MODEL", "claude-haiku-4-5-20251001")

OPENAI_BASE_URL = os.getenv("OPENAI_BASE_URL", "https://api.openai.com/v1")
OPENAI_API_KEY  = os.getenv("OPENAI_API_KEY", "sk-DUMMY-replace-me")
OPENAI_MODEL    = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

UPSTREAM_TIMEOUT_S = float(os.getenv("UPSTREAM_TIMEOUT_S", "30"))  # camera waits 40 s
MAX_IMAGE_BYTES    = int(os.getenv("MAX_IMAGE_BYTES", str(4 * 1024 * 1024)))

SAVE_CAPTURES = os.getenv("SAVE_CAPTURES", "false").lower() in ("1", "true", "yes")
CAPTURE_DIR   = Path(os.getenv("CAPTURE_DIR", "captures"))
KEEP_CAPTURES = int(os.getenv("KEEP_CAPTURES", "200"))

PORT = int(os.getenv("PORT", "8080"))          # App Platform sets PORT
# ============================================================================

SYSTEM_PROMPT = (
    "You are a road-safety vision checker for a motorcycle rider-monitoring "
    "system. You receive one photo from a camera facing the rider. Reply with "
    "ONLY one JSON object: no markdown, no code fences, no extra text. Use "
    "exactly these keys: "
    "face_visible (boolean: a person's face or head is visible), "
    "helmet (boolean: the person is wearing a motorcycle or bicycle helmet ON "
    "their head; a helmet held in the hand or lying nearby is false; a cap, "
    "hood or headphones is false), "
    "drowsy (boolean: signs of drowsiness such as closed or half-closed eyes, "
    "a drooping or nodding head, or yawning), "
    "eyes (string: one of open, half, closed, not_visible; use not_visible if "
    "a dark visor or glasses hide them), "
    "confidence (number from 0 to 1 for your overall judgement), "
    "notes (string, under 20 words, what you based the decision on). "
    "If no person is visible set face_visible false, helmet false, drowsy "
    "false, eyes not_visible, confidence 0."
)
USER_PROMPT = "Check this rider: is a helmet being worn, and does the rider look drowsy?"
RETRY_SUFFIX = (
    " Your previous reply could not be parsed. Respond with the JSON object "
    "only, starting with { and ending with }."
)

EYES_VALUES = {"open", "half", "closed", "not_visible"}
EYES_ALIASES = {
    "half-closed": "half", "half_closed": "half", "half-open": "half",
    "half_open": "half", "partially closed": "half", "squinting": "half",
    "shut": "closed", "hidden": "not_visible", "not visible": "not_visible",
    "unknown": "not_visible", "obscured": "not_visible",
}

log = logging.getLogger("helmet-agent")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

STARTED = time.time()
last_result: dict[str, Any] = {}


# ----------------------------------------------------------------------------
#  Errors -> OpenAI-style error bodies
# ----------------------------------------------------------------------------
class AgentError(Exception):
    def __init__(self, status: int, message: str, kind: str = "invalid_request_error"):
        super().__init__(message)
        self.status, self.message, self.kind = status, message, kind


def upstream_error(name: str, r: httpx.Response) -> AgentError:
    snippet = r.text[:200].replace("\n", " ")
    if r.status_code in (401, 403):
        return AgentError(502, f"{name} rejected the upstream API key (HTTP {r.status_code}); "
                               f"check the key in the agent's environment", "upstream_error")
    if r.status_code == 429:
        return AgentError(503, f"{name} rate limit hit; try again shortly", "upstream_error")
    return AgentError(502, f"{name} HTTP {r.status_code}: {snippet}", "upstream_error")


# ----------------------------------------------------------------------------
#  Request parsing
# ----------------------------------------------------------------------------
def check_auth(request: Request) -> None:
    auth = request.headers.get("authorization", "")
    token = auth[7:].strip() if auth.lower().startswith("bearer ") else request.headers.get("x-api-key", "")
    if not token or not hmac.compare_digest(token.encode(), AGENT_API_KEY.encode()):
        raise AgentError(401, "invalid or missing API key", "authentication_error")


def decode_data_uri(url: str) -> tuple[bytes, str]:
    # Only inline images: fetching remote URLs would let callers make this
    # server request arbitrary addresses.
    if not url.startswith("data:"):
        raise AgentError(400, "image_url must be a data:image/...;base64 URI")
    header, _, data = url.partition(",")
    media = header[5:].split(";")[0].lower() or "image/jpeg"
    if media not in ("image/jpeg", "image/jpg", "image/png"):
        raise AgentError(400, f"unsupported image type {media}; send JPEG or PNG")
    if ";base64" not in header:
        raise AgentError(400, "image data must be base64-encoded")
    try:
        img = base64.b64decode(data, validate=True)
    except (binascii.Error, ValueError):
        raise AgentError(400, "image base64 is corrupt")
    if len(img) > MAX_IMAGE_BYTES:
        raise AgentError(413, f"image is {len(img)} bytes; limit is {MAX_IMAGE_BYTES}")
    if img[:3] == b"\xff\xd8\xff":
        return img, "image/jpeg"
    if img[:4] == b"\x89PNG":
        return img, "image/png"
    raise AgentError(400, "image data is not a JPEG or PNG (check the camera capture)")


def extract_image(body: dict[str, Any]) -> tuple[bytes, str]:
    for msg in body.get("messages") or []:
        content = msg.get("content") if isinstance(msg, dict) else None
        if not isinstance(content, list):
            continue
        for part in content:
            if isinstance(part, dict) and part.get("type") == "image_url":
                iu = part.get("image_url")
                url = iu.get("url") if isinstance(iu, dict) else iu
                if isinstance(url, str):
                    return decode_data_uri(url)
    raise AgentError(400, "no image found; send a user message with an image_url part")


# ----------------------------------------------------------------------------
#  Model output -> clean, predictable JSON for the ESP32
# ----------------------------------------------------------------------------
def as_bool(v: Any, default: bool = False) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return v != 0
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "yes", "y", "1", "worn", "on"):
            return True
        if s in ("false", "no", "n", "0", "none", "off"):
            return False
    return default


def normalize(raw: str) -> dict[str, Any]:
    text = (raw or "").strip()
    a, b = text.find("{"), text.rfind("}")
    if a < 0 or b <= a:
        raise ValueError("no JSON object in reply")
    d = json.loads(text[a:b + 1])
    if not isinstance(d, dict):
        raise ValueError("reply JSON is not an object")

    face = as_bool(d.get("face_visible", d.get("face")), True)

    eyes = str(d.get("eyes", "not_visible")).strip().lower()
    eyes = EYES_ALIASES.get(eyes, eyes)
    if eyes not in EYES_VALUES:
        eyes = "not_visible"

    try:
        conf = float(d.get("confidence", 0))
    except (TypeError, ValueError):
        conf = 0.0
    if 1 < conf <= 100:            # model answered as a percentage
        conf /= 100
    conf = max(0.0, min(1.0, conf))

    helmet = as_bool(d.get("helmet"), False)
    drowsy = as_bool(d.get("drowsy"), False)
    notes = " ".join(str(d.get("notes", "")).split())[:140]

    if not face:
        helmet, drowsy, eyes, conf = False, False, "not_visible", 0.0
    elif eyes == "closed":
        drowsy = True              # closed eyes on a rider is never "alert"

    return {"face_visible": face, "helmet": helmet, "drowsy": drowsy,
            "eyes": eyes, "confidence": round(conf, 2), "notes": notes}


# ----------------------------------------------------------------------------
#  Vision backends — each returns the model's raw text
# ----------------------------------------------------------------------------
async def ask_anthropic(client: httpx.AsyncClient, img: bytes, media: str, retry: bool) -> str:
    payload = {
        "model": ANTHROPIC_MODEL,
        "max_tokens": 300,
        "temperature": 0,
        "system": SYSTEM_PROMPT,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "image", "source": {"type": "base64", "media_type": media,
                                             "data": base64.b64encode(img).decode()}},
                {"type": "text", "text": USER_PROMPT + (RETRY_SUFFIX if retry else "")},
            ],
        }],
    }
    r = await client.post(
        "https://api.anthropic.com/v1/messages",
        headers={"x-api-key": ANTHROPIC_API_KEY, "anthropic-version": "2023-06-01",
                 "content-type": "application/json"},
        json=payload,
    )
    if r.status_code != 200:
        raise upstream_error("Anthropic", r)
    return "".join(b.get("text", "") for b in r.json().get("content", []) if b.get("type") == "text")


async def ask_openai(client: httpx.AsyncClient, img: bytes, media: str, retry: bool) -> str:
    data_uri = f"data:{media};base64,{base64.b64encode(img).decode()}"
    payload = {
        "model": OPENAI_MODEL,
        "temperature": 0,
        "max_tokens": 300,
        "messages": [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": [
                {"type": "text", "text": USER_PROMPT + (RETRY_SUFFIX if retry else "")},
                {"type": "image_url", "image_url": {"url": data_uri}},
            ]},
        ],
    }
    r = await client.post(
        f"{OPENAI_BASE_URL.rstrip('/')}/chat/completions",
        headers={"Authorization": f"Bearer {OPENAI_API_KEY}"},
        json=payload,
    )
    if r.status_code != 200:
        raise upstream_error("OpenAI-compatible backend", r)
    content = r.json()["choices"][0]["message"].get("content")
    if isinstance(content, list):   # some servers return content parts
        content = "".join(p.get("text", "") for p in content if isinstance(p, dict))
    return content or ""


async def ask_mock(client: httpx.AsyncClient, img: bytes, media: str, retry: bool) -> str:
    await asyncio.sleep(0.8)
    drowsy = random.random() < 0.25
    return json.dumps({
        "face_visible": True, "helmet": random.random() < 0.8, "drowsy": drowsy,
        "eyes": "half" if drowsy else "open", "confidence": round(random.uniform(0.6, 0.95), 2),
        "notes": f"mock backend, {len(img)} byte image",
    })


BACKENDS = {"anthropic": ask_anthropic, "openai": ask_openai, "mock": ask_mock}


def model_name() -> str:
    return {"anthropic": ANTHROPIC_MODEL, "openai": OPENAI_MODEL}.get(VLM_BACKEND, "mock")


async def analyze(client: httpx.AsyncClient, img: bytes, media: str) -> tuple[dict[str, Any], str]:
    backend = BACKENDS.get(VLM_BACKEND)
    if not backend:
        raise AgentError(500, f"unknown VLM_BACKEND '{VLM_BACKEND}'", "server_error")

    last_err: Exception | None = None
    for attempt in (0, 1):          # one retry if the model rambles instead of JSON
        try:
            raw = await backend(client, img, media, attempt == 1)
        except httpx.TimeoutException:
            raise AgentError(504, f"vision model timed out after {UPSTREAM_TIMEOUT_S:.0f} s", "upstream_error")
        except httpx.HTTPError as e:
            raise AgentError(502, f"cannot reach vision model: {e.__class__.__name__}", "upstream_error")
        try:
            return normalize(raw), raw
        except (ValueError, json.JSONDecodeError) as e:
            last_err = e
            log.warning("unparseable model reply (attempt %d): %r", attempt + 1, raw[:200])
    raise AgentError(502, f"vision model did not return usable JSON ({last_err})", "upstream_error")


# ----------------------------------------------------------------------------
#  Optional capture log (App Platform disks are wiped on redeploy)
# ----------------------------------------------------------------------------
def save_capture(req_id: str, img: bytes, media: str, result: dict[str, Any]) -> None:
    try:
        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
        stem = CAPTURE_DIR / f"{time.strftime('%Y%m%d-%H%M%S')}-{req_id[:8]}"
        stem.with_suffix(".png" if media == "image/png" else ".jpg").write_bytes(img)
        stem.with_suffix(".json").write_text(json.dumps(result, indent=2))
        files = sorted(CAPTURE_DIR.glob("*.json"))
        for old in files[:-KEEP_CAPTURES]:
            for f in CAPTURE_DIR.glob(old.stem + ".*"):
                f.unlink(missing_ok=True)
    except OSError as e:
        log.warning("could not save capture: %s", e)


# ----------------------------------------------------------------------------
#  App
# ----------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    if "DUMMY" in AGENT_API_KEY:
        log.warning("AGENT_API_KEY is still the dummy value; set a real secret (and VLM_KEY on the camera)")
    if VLM_BACKEND == "anthropic" and "DUMMY" in ANTHROPIC_API_KEY:
        log.warning("ANTHROPIC_API_KEY is a dummy; requests will fail. Set it, or use VLM_BACKEND=mock")
    if VLM_BACKEND == "openai" and "DUMMY" in OPENAI_API_KEY and "api.openai.com" in OPENAI_BASE_URL:
        log.warning("OPENAI_API_KEY is a dummy; requests will fail")
    log.info("helmet agent up: backend=%s model=%s", VLM_BACKEND, model_name())
    app.state.http = httpx.AsyncClient(timeout=httpx.Timeout(UPSTREAM_TIMEOUT_S, connect=10))
    yield
    await app.state.http.aclose()


app = FastAPI(title="Project Helmet VLM agent", lifespan=lifespan)


@app.exception_handler(AgentError)
async def agent_error_handler(request: Request, exc: AgentError):
    log.warning("%s %s -> %d %s", request.method, request.url.path, exc.status, exc.message)
    return JSONResponse(status_code=exc.status,
                        content={"error": {"message": exc.message, "type": exc.kind}})


@app.get("/health")
async def health():
    return {"ok": True, "backend": VLM_BACKEND, "model": model_name(),
            "uptime_s": int(time.time() - STARTED)}


@app.get("/last")
async def last(request: Request):
    check_auth(request)
    return last_result or {"message": "no checks yet"}


@app.post("/api/v1/chat/completions")
@app.post("/v1/chat/completions")
async def chat_completions(request: Request):
    check_auth(request)
    length = int(request.headers.get("content-length") or 0)
    if length > MAX_IMAGE_BYTES * 1.4 + 20_000:
        raise AgentError(413, "request too large")
    try:
        body = await request.json()
    except (json.JSONDecodeError, UnicodeDecodeError):
        raise AgentError(400, "request body must be JSON")
    if not isinstance(body, dict):
        raise AgentError(400, "request body must be a JSON object")

    img, media = extract_image(body)
    req_id = uuid.uuid4().hex
    t0 = time.monotonic()
    result, _raw = await analyze(request.app.state.http, img, media)
    ms = int((time.monotonic() - t0) * 1000)

    client_ip = request.client.host if request.client else "?"
    log.info("check %s from %s: %d B -> %s (%d ms)", req_id[:8], client_ip, len(img),
             json.dumps(result), ms)

    last_result.clear()
    last_result.update({"id": req_id, "at": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "from": client_ip, "image_bytes": len(img), "ms": ms, "result": result})
    if SAVE_CAPTURES:
        save_capture(req_id, img, media, result)

    return {
        "id": f"chatcmpl-{req_id}",
        "object": "chat.completion",
        "created": int(time.time()),
        "model": model_name(),
        "choices": [{
            "index": 0,
            "message": {"role": "assistant",
                        "content": json.dumps(result, separators=(",", ":"))},
            "finish_reason": "stop",
        }],
        "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
    }


# ----------------------------------------------------------------------------
#  CLI: `python agent.py` serves; `python agent.py --test photo.jpg` checks one image
# ----------------------------------------------------------------------------
async def _test(path: str) -> None:
    data = Path(path).read_bytes()
    media = "image/png" if data[:4] == b"\x89PNG" else "image/jpeg"
    async with httpx.AsyncClient(timeout=httpx.Timeout(UPSTREAM_TIMEOUT_S, connect=10)) as c:
        t0 = time.monotonic()
        result, raw = await analyze(c, data, media)
    print(f"backend={VLM_BACKEND} model={model_name()} ({time.monotonic() - t0:.1f} s)")
    print("raw model reply:", raw)
    print("normalised:     ", json.dumps(result, indent=2))


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--test":
        try:
            asyncio.run(_test(sys.argv[2]))
        except AgentError as e:
            sys.exit(f"error: {e.message}")
    else:
        import uvicorn
        uvicorn.run("agent:app", host="0.0.0.0", port=PORT, log_level="info")
