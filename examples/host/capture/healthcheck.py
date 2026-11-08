#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Docker healthcheck for capture_server.py.

Tries https first (the normal case) and then plain http (the fallback when no
certificate exists yet), so `docker compose ps` is honest in both states.
"""

import os
import ssl
import sys
import urllib.request

PORT = os.environ.get("REBOT_PORT", "8443")

for scheme in ("https", "http"):
    context = None
    if scheme == "https":
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    try:
        with urllib.request.urlopen("%s://127.0.0.1:%s/health" % (scheme, PORT),
                                    context=context, timeout=3) as answer:
            sys.exit(0 if answer.status == 200 else 1)
    except Exception:  # noqa: BLE001 - try the other scheme
        continue
sys.exit(1)
