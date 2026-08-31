#!/usr/bin/env python3
"""Patch Portduino's TCP EOF handling before a native build."""

from __future__ import annotations

import os
import pathlib
import sys

OLD = "if (portduino_socket_would_block(errorCode) || numread == 0)"
NEW = "if (portduino_socket_would_block(errorCode))"


def patch_text(source: str) -> str:
    if OLD in source:
        return source.replace(OLD, NEW, 1)
    if NEW in source:
        return source
    raise RuntimeError("expected Portduino WiFiClient EOF handling was not found")


def default_path() -> pathlib.Path:
    packages = pathlib.Path(
        os.environ.get("PLATFORMIO_PACKAGES_DIR", pathlib.Path.home() / ".platformio" / "packages")
    )
    return packages / "framework-portduino" / "libraries" / "WiFi" / "src" / "WiFiClient.cpp"


def main() -> int:
    path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else default_path()
    source = path.read_text()
    patched = patch_text(source)
    if patched != source:
        path.write_text(patched)
        print(f"patched Portduino TCP EOF handling: {path}")
    else:
        print(f"Portduino TCP EOF handling already patched: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
