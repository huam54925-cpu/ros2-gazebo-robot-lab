#!/usr/bin/env python3
"""Install a checksum-pinned uv locally, without pip or host changes."""
import hashlib
import io
from pathlib import Path
import platform
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[1]
URL = "https://files.pythonhosted.org/packages/e5/83/85a6c63c24905af4924fddb11a499b934913f59a134248367a1ef1a4716f/uv-0.12.23-py3-none-manylinux_2_17_x86_64.manylinux2014_x86_64.whl"
SHA256 = "565c6e2874dbeae86c02f3dea97255e878fec672659a73d4930c6b93fcab2fff"


def main():
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise SystemExit("This bootstrap supports Linux x86_64 only.")
    target = ROOT / ".tools" / "uv"
    if target.is_file():
        return
    with urllib.request.urlopen(URL, timeout=60) as response:
        wheel = response.read()
    if hashlib.sha256(wheel).hexdigest() != SHA256:
        raise SystemExit("uv wheel checksum mismatch")
    with zipfile.ZipFile(io.BytesIO(wheel)) as archive:
        names = [n for n in archive.namelist() if n.endswith("/scripts/uv")]
        if len(names) != 1:
            raise SystemExit("Unexpected uv wheel layout")
        binary = archive.read(names[0])
    target.parent.mkdir(exist_ok=True)
    temporary = target.with_suffix(".tmp")
    temporary.write_bytes(binary)
    temporary.chmod(0o755)
    temporary.replace(target)
    print("Installed project-local uv 0.12.23")


if __name__ == "__main__":
    main()
