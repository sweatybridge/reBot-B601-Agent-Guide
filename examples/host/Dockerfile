# Host image: the capture server and every one-shot CV tool.
#
# One image, three entry points: `app/capture_server.py` (the long-running
# service), the CLIs in `app/` (calibrate_intrinsics, locate_target, make_tag,
# make_cert) and `tests/selftest.py`. compose.yaml runs it as `capture` and
# `tools`.
#
# opencv-contrib is required for cv2.aruco; the headless build keeps X11 out.
FROM python:3.11-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1

# The headless OpenCV wheel still links glib at import time.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Mirrors the host layout: /app is the project root, the code is in /app/app,
# the pages in /app/web, the bind mounts in /app/state. The scripts compute
# ROOT = parent(__file__), so both layouts behave the same.
WORKDIR /app
COPY requirements.txt ./
RUN pip install -r requirements.txt

COPY app ./app
COPY web ./web
COPY tests ./tests

# Runs as root on purpose: captures, calibrations and certs are bind mounts
# owned by the host user, and a fixed non-root uid would not be able to write
# them on Linux. Nothing here is exposed beyond the LAN.
EXPOSE 8443
CMD ["python", "app/capture_server.py"]
