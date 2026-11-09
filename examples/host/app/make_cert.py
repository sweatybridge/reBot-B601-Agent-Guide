#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Create a local CA and a TLS certificate for the phone capture page.

`getUserMedia` only exists on a secure origin, and `https://<lan-ip>` on a phone
is only secure if the phone trusts the certificate. A self-signed leaf cannot be
installed as a CA on Android, so this writes a small CA plus a leaf signed by it:
install `ca.pem` once, then `--reuse` mints a new leaf whenever the host's IP
changes.

    python make_cert.py --host rebot-host --ip 192.168.1.23
    python make_cert.py --reuse --host rebot-host --ip 192.168.1.44

The private keys stay in `tls/`; that directory is git-ignored.
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import socket
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID


def lan_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))
            return probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def addresses(explicit, from_env) -> list:
    """The SANs for the leaf: --ip flags, then REBOT_HOST_IP, then detection.

    Inside a container the detected address is the container's own, which no
    phone can reach, so an explicit address or REBOT_HOST_IP always wins.
    """
    values = [value for value in list(explicit) + [from_env] if value]
    if values:
        return list(dict.fromkeys(values))
    return [lan_ip()]


def private_key():
    return rsa.generate_private_key(public_exponent=65537, key_size=2048)


def write_key(path: Path, key):
    path.write_bytes(key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.TraditionalOpenSSL,
        encryption_algorithm=serialization.NoEncryption(),
    ))


def write_cert(path: Path, cert):
    path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))


def make_ca(common_name="reBot local CA", days=825):
    key = private_key()
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, common_name)])
    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(hours=1))
        .not_valid_after(now + timedelta(days=days))
        .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
        .add_extension(x509.KeyUsage(
            digital_signature=True, content_commitment=False, key_encipherment=False,
            data_encipherment=False, key_agreement=False, key_cert_sign=True,
            crl_sign=True, encipher_only=False, decipher_only=False), critical=True)
        .sign(key, hashes.SHA256())
    )
    return key, cert


def load_ca(directory: Path):
    key = serialization.load_pem_private_key((directory / "ca-key.pem").read_bytes(), password=None)
    cert = x509.load_pem_x509_certificate((directory / "ca.pem").read_bytes())
    return key, cert


def make_leaf(ca_key, ca_cert, hostname, ips, days):
    key = private_key()
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, hostname)])
    alt = [x509.DNSName(hostname), x509.DNSName("localhost")]
    for address in ips:
        try:
            alt.append(x509.IPAddress(ipaddress.ip_address(address)))
        except ValueError:
            raise SystemExit("not an IP address: %s" % address)
    alt.append(x509.IPAddress(ipaddress.ip_address("127.0.0.1")))

    now = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(ca_cert.subject)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(hours=1))
        .not_valid_after(now + timedelta(days=days))
        .add_extension(x509.SubjectAlternativeName(alt), critical=False)
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
        .sign(ca_key, hashes.SHA256())
    )
    return key, cert


def main(argv=None):
    parser = argparse.ArgumentParser(description="Local CA + TLS certificate for the capture page.")
    parser.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "state" / "tls"),
                        help="directory for the PEM files")
    parser.add_argument("--host", default="rebot-host.local", help="DNS name the phone will use")
    parser.add_argument("--ip", action="append", default=[], help="extra IP SAN, repeatable")
    parser.add_argument("--days", type=int, default=825, help="leaf lifetime (Apple caps at 825)")
    parser.add_argument("--reuse", action="store_true", help="reuse tls/ca.pem instead of a new CA")
    parser.add_argument("--force", action="store_true", help="overwrite an existing CA")
    args = parser.parse_args(argv)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ca_cert_path, ca_key_path = out / "ca.pem", out / "ca-key.pem"

    if ca_cert_path.is_file() and not args.force:
        ca_key, ca_cert = load_ca(out)
        print("reusing CA  : %s" % ca_cert_path)
    else:
        ca_key, ca_cert = make_ca()
        write_key(ca_key_path, ca_key)
        write_cert(ca_cert_path, ca_cert)
        print("created CA  : %s" % ca_cert_path)

    ips = addresses(args.ip, os.environ.get("REBOT_HOST_IP", "").strip())
    if args.reuse and not ca_cert_path.is_file():
        raise SystemExit("--reuse given but %s does not exist" % ca_cert_path)

    leaf_key, leaf_cert = make_leaf(ca_key, ca_cert, args.host, ips, args.days)
    write_key(out / "key.pem", leaf_key)
    cert_chain = leaf_cert.public_bytes(serialization.Encoding.PEM) + \
        ca_cert.public_bytes(serialization.Encoding.PEM)
    (out / "cert.pem").write_bytes(cert_chain)

    print("leaf        : %s (%d days)" % (out / "cert.pem", args.days))
    print("names       : %s, %s" % (args.host, ", ".join(ips)))
    print("\nnext:")
    print("  1. install %s on the phone (iOS: profile + 'Certificate Trust Settings';" % ca_cert_path)
    print("     Android: Settings > Security > Encryption & credentials > Install a CA certificate)")
    print("  2. python capture_server.py --cert %s --key %s"
          % (out / "cert.pem", out / "key.pem"))
    print("  3. open https://%s:8443/ on the phone" % ips[0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
