# Capture server + CV tools for the phone pipeline.
#
# One image, three entry points: `capture_server.py` (the long-running service),
# the one-shot CLIs (calibrate_intrinsics.py, locate_target.py, make_tag.py,
# make_cert.py) and selftest.py. compose.yaml runs it as `capture` and `tools`.
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

WORKDIR /app
COPY requirements.txt ./
RUN pip install -r requirements.txt

COPY *.py *.html ./

# Runs as root on purpose: captures, calibrations and certs are bind mounts
# owned by the host user, and a fixed non-root uid would not be able to write
# them on Linux. Nothing here is exposed beyond the LAN.
EXPOSE 8443
CMD ["python", "capture_server.py"]
