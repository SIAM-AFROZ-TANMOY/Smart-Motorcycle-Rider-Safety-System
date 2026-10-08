#!/usr/bin/env python3
"""
Deploy the VLM agent to the droplet over SSH (password auth).

Secrets come from the environment so they never land in a file in this repo:
    DO_HOST, DO_USER (default root), DO_PASSWORD, ANTHROPIC_API_KEY, DEVICE_KEY
    python3 deploy.py probe     # read-only look at the server
    python3 deploy.py deploy    # install + start the service
    python3 deploy.py update    # push app.py only + restart (secrets untouched)
"""
import os, shlex, sys, warnings
warnings.filterwarnings("ignore")
import paramiko

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.environ["DO_HOST"]
USER = os.environ.get("DO_USER", "root")


def connect():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, username=USER, password=os.environ["DO_PASSWORD"], timeout=20,
              allow_agent=False, look_for_keys=False)
    return c


def run(c, cmd, stdin=None, show=True):
    _in, out, err = c.exec_command(cmd, get_pty=False)
    if stdin is not None:
        _in.write(stdin); _in.channel.shutdown_write()
    o = out.read().decode(errors="replace"); e = err.read().decode(errors="replace")
    rc = out.channel.recv_exit_status()
    if show:
        if o.strip(): print(o.rstrip())
        if e.strip(): print("[stderr]", e.rstrip())
    return rc, o


def probe(c):
    run(c, "hostname; . /etc/os-release; echo $PRETTY_NAME; python3 --version; "
           "id -u vlm 2>&1 | head -1; ls /opt 2>&1; "
           "(ufw status 2>&1 | head -5); (ss -ltnp 2>/dev/null | awk 'NR==1||/LISTEN/' | head -15); "
           "df -h / | tail -1; free -m | sed -n 2p")


def deploy(c):
    sftp = c.open_sftp()
    steps = [
        "export DEBIAN_FRONTEND=noninteractive; apt-get update -qq && "
        "apt-get install -y -qq python3-venv python3-pip >/dev/null",
        "id vlm >/dev/null 2>&1 || useradd --system --home /opt/vlm-agent --shell /usr/sbin/nologin vlm",
        "mkdir -p /opt/vlm-agent",
    ]
    for s in steps:
        rc, _ = run(c, s)
        if rc: sys.exit(f"step failed: {s}")
    for f in ("app.py", "requirements.txt"):
        sftp.put(os.path.join(HERE, f), f"/opt/vlm-agent/{f}")
    sftp.put(os.path.join(HERE, "vlm-agent.service"), "/etc/systemd/system/vlm-agent.service")

    # secrets go over stdin -> file, never on a command line
    env = (f"ANTHROPIC_API_KEY={os.environ['ANTHROPIC_API_KEY']}\n"
           f"DEVICE_KEY={os.environ['DEVICE_KEY']}\n"
           f"VLM_MODEL={os.environ.get('VLM_MODEL', 'claude-opus-5')}\n"
           "MAX_RIDERS=2\nREQUIRE_EYES_VISIBLE=0\n")
    run(c, "umask 077; cat > /etc/vlm-agent.env", stdin=env)
    run(c, "chown root:root /etc/vlm-agent.env; chmod 600 /etc/vlm-agent.env")

    for s in [
        "cd /opt/vlm-agent && python3 -m venv venv && ./venv/bin/pip install -q --upgrade pip "
        "&& ./venv/bin/pip install -q -r requirements.txt",
        "chown -R vlm:vlm /opt/vlm-agent",
        "if ufw status 2>/dev/null | grep -q 'Status: active'; then ufw allow 8000/tcp; fi",
        "systemctl daemon-reload && systemctl enable vlm-agent >/dev/null 2>&1 && systemctl restart vlm-agent",
        "sleep 3; systemctl is-active vlm-agent; curl -s localhost:8000/health",
    ]:
        rc, _ = run(c, s)
        if rc: sys.exit(f"step failed: {s}")
    print("deployed.")


def update(c):
    sftp = c.open_sftp()
    sftp.put(os.path.join(HERE, "app.py"), "/opt/vlm-agent/app.py")
    run(c, "chown vlm:vlm /opt/vlm-agent/app.py && systemctl restart vlm-agent")
    run(c, "sleep 3; systemctl is-active vlm-agent; curl -s localhost:8000/health")
    print("updated.")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "probe"
    conn = connect()
    {"probe": probe, "deploy": deploy, "update": update}[cmd](conn)
    conn.close()
