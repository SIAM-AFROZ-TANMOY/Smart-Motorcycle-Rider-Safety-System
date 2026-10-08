"""
Rider-check VLM agent.

The ESP32 POSTs a JPEG snapshot to /analyze (raw body, header X-API-Key) and gets
back a small flat JSON verdict:

    {"ok":true,"helmet":true,"riders":1,"drowsy":false,"quality":"good","note":"..."}

Claude looks at the photo and reports three things:
  1. is the rider wearing a helmet (a GREEN object in front of the face counts too)
  2. how many people are in the photo = number of riders (more than MAX_RIDERS -> not ok)
  3. does the rider look drowsy

The setting / background / camera angle does not matter: any person in the
frame is treated as a rider, whether or not a vehicle is visible.

The Anthropic key lives ONLY in this server's environment (ANTHROPIC_API_KEY);
the ESP32 only knows DEVICE_KEY, which is useless against Anthropic directly.
"""
import asyncio
import hmac
import logging
import os
import re
from typing import Literal

import anthropic
from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel, Field

MODEL = os.environ.get("VLM_MODEL", "claude-opus-5")
DEVICE_KEY = os.environ["DEVICE_KEY"]
MAX_RIDERS = int(os.environ.get("MAX_RIDERS", "2"))
# If the eyes can't be seen (tinted visor, camera angle) drowsiness can't be
# judged. Default: treat as "not drowsy". Set to 1 to fail closed instead.
REQUIRE_EYES = os.environ.get("REQUIRE_EYES_VISIBLE", "0") == "1"
MAX_IMAGE_BYTES = 3 * 1024 * 1024

log = logging.getLogger("vlm-agent")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

client = anthropic.AsyncAnthropic()          # reads ANTHROPIC_API_KEY
gate = asyncio.Semaphore(2)                  # at most 2 model calls at once
app = FastAPI(title="rider-check", docs_url=None, redoc_url=None)


class Observation(BaseModel):
    people_count: int = Field(
        description="Number of real, distinct people visible in the photo. Every person counts "
                    "as a rider - do NOT require a motorcycle, seat or any particular setting. "
                    "Count people even if partly cropped, as long as a face or head is clearly "
                    "recognisable. Do not count pictures, posters, screens, mannequins or "
                    "reflections. 0 if nobody is visible.")
    helmet_worn: bool = Field(
        description="True if the main rider (the most prominent / nearest person) has a helmet "
                    "ON their head, OR has a GREEN object in front of their face (a green "
                    "helmet, visor, mask, board or any green item covering or held/placed in "
                    "front of the face) - treat that green object as a helmet. False otherwise: "
                    "a bare head, or a helmet held far from the head, on a shelf, or hung on "
                    "the handlebar. A plain cap, hat or hood is not a helmet.")
    eyes_visible: bool = Field(
        description="True if the main rider's eyes/face are visible well enough to judge alertness.")
    drowsy: bool = Field(
        description="True if the main rider shows clear signs of drowsiness: eyes closed or "
                    "nearly closed, heavy drooping eyelids, head nodding/slumped, or a wide "
                    "yawn. False if awake and alert, or if the eyes cannot be seen. Judge only "
                    "from the rider, never from the surroundings.")
    image_quality: Literal["good", "poor"] = Field(
        description="'poor' only if the photo is so dark, blurred or blocked that the rider "
                    "cannot be judged. An unusual or cluttered background, a different room, "
                    "outdoor light or an odd camera angle is NOT poor quality.")
    note: str = Field(description="One short plain sentence (max 15 words) explaining the verdict.")


SYSTEM = (
    "You are the vision check for a motorcycle ignition interlock. You receive one photo from a "
    "camera. Only the people in the photo matter: judge them, and ignore the environment, "
    "background, lighting style and camera angle - it can be anywhere, on or off a vehicle. "
    "Each person in the photo counts as one rider. A green object in front of a rider's face "
    "is accepted as that rider's helmet. Report only what is visible and do not identify anyone."
)
PROMPT = "Inspect this photo and fill in the observation."


def clean(text: str) -> str:
    """ASCII, no quotes/backslashes/newlines: the ESP32 parses this with indexOf()."""
    text = re.sub(r"[^A-Za-z0-9 .,;:()%/+-]", "", text or "")
    return re.sub(r"\s+", " ", text).strip()[:80]


def media_type(data: bytes) -> str | None:
    if data[:3] == b"\xff\xd8\xff":
        return "image/jpeg"
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return "image/png"
    return None


@app.get("/health")
async def health():
    return {"status": "ok", "model": MODEL}


@app.post("/analyze")
async def analyze(request: Request):
    key = request.headers.get("x-api-key", "")
    if not hmac.compare_digest(key.encode(), DEVICE_KEY.encode()):
        raise HTTPException(status_code=401, detail="bad key")

    data = await request.body()
    if not data or len(data) > MAX_IMAGE_BYTES:
        raise HTTPException(status_code=413, detail="image missing or too large")
    mt = media_type(data)
    if not mt:
        raise HTTPException(status_code=415, detail="send a JPEG or PNG")

    import base64
    b64 = base64.standard_b64encode(data).decode()

    async with gate:
        try:
            resp = await client.messages.parse(
                model=MODEL,
                max_tokens=1024,
                system=SYSTEM,
                messages=[{"role": "user", "content": [
                    {"type": "image", "source": {"type": "base64", "media_type": mt, "data": b64}},
                    {"type": "text", "text": PROMPT},
                ]}],
                output_format=Observation,
            )
        except anthropic.APIError as e:
            log.error("anthropic error: %s", e)
            raise HTTPException(status_code=502, detail="vision model error")

    obs = resp.parsed_output
    if obs is None:                                   # refusal / max_tokens / unparsable
        log.warning("no parsed output, stop_reason=%s", resp.stop_reason)
        raise HTTPException(status_code=502, detail="no verdict")

    riders = max(0, obs.people_count)
    ok = (
        obs.image_quality == "good"
        and 1 <= riders <= MAX_RIDERS
        and obs.helmet_worn
        and not obs.drowsy
        and (obs.eyes_visible or not REQUIRE_EYES)
    )
    out = {
        "ok": ok,
        "helmet": obs.helmet_worn,
        "riders": riders,
        "drowsy": obs.drowsy,
        "quality": obs.image_quality,
        "note": clean(obs.note),
    }
    log.info("verdict %s (%d bytes)", out, len(data))
    return out
