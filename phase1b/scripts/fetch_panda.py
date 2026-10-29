"""Vendor pinned upstream Panda assets, preserving upstream license and hashes."""

import concurrent.futures
import hashlib
import json
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

REVISION = "f3b6013629c7b94f8ee93fd977b161fa8350f9bc"
ROOT = Path(__file__).resolve().parents[1] / "vendor/franka_emika_panda"
BASE = f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{REVISION}/franka_emika_panda/"


def fetch(name):
    target = ROOT / name
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(BASE + name, timeout=60) as response:
            target.write_bytes(response.read())
    return name, hashlib.sha256(target.read_bytes()).hexdigest()


def main():
    fetch("panda.xml")
    names = ["panda.xml", "README.md", "LICENSE", "CHANGELOG.md"] + [
        "assets/" + node.attrib["file"]
        for node in ET.parse(ROOT / "panda.xml").findall("./asset/mesh")
    ]
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        hashes = dict(pool.map(fetch, names))
    (ROOT / "provenance.json").write_text(
        json.dumps({"upstream": BASE, "revision": REVISION, "sha256": hashes}, indent=2)
        + "\n"
    )
    print(f"Vendored {len(hashes)} files at {REVISION}", flush=True)


if __name__ == "__main__":
    main()
