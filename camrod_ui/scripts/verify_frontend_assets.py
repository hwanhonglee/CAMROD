#!/usr/bin/env python3
"""HH_261006 - Reject UI publication with missing or stale portable model assets."""

import argparse
import hashlib
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit


REQUIRED_ASSETS = (
    "models/ranger-navigation.glb",
    "models/woraksan-side-wrap.png",
    "models/woraksan-front-wrap.png",
    "models/woraksan-rear-wrap.png",
    "models/ranger-driving-rear.png",
)


class _BundleReferences(HTMLParser):
    def __init__(self):
        super().__init__()
        self.paths = []

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        value = attributes.get("src") if tag == "script" else None
        if tag == "link" and attributes.get("rel") == "stylesheet":
            value = attributes.get("href")
        if value:
            parsed = urlsplit(value)
            if parsed.scheme or parsed.netloc:
                # Existing optional web fonts are not local package artifacts.
                # The compiled application itself still requires a local JS.
                return
            self.paths.append(unquote(parsed.path).lstrip("/"))


def _digest(path):
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"missing or empty UI asset: {path}")
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.digest()


def verify_assets(frontend, build, installed=None):
    """Validate a CRA tree without importing ROS, browsers or developer paths."""
    frontend, build = Path(frontend), Path(build)
    public = frontend / "public"
    for relative in REQUIRED_ASSETS:
        _digest(public / relative)

    parser = _BundleReferences()
    _digest(build / "index.html")
    parser.feed((build / "index.html").read_text(encoding="utf-8"))
    if not any(path.endswith(".js") for path in parser.paths):
        raise ValueError(f"compiled JavaScript reference missing: {build / 'index.html'}")
    for relative in ("index.html", *parser.paths):
        if ".." in Path(relative).parts:
            raise ValueError(f"invalid UI bundle path: {relative}")
        expected = _digest(build / relative)
        if installed is not None and _digest(Path(installed) / relative) != expected:
            raise ValueError(f"installed UI bundle differs: {relative}")

    count = 0
    for source in sorted(public.rglob("*")):
        if not source.is_file():
            continue
        relative = source.relative_to(public)
        # CRA substitutes environment values only in public/index.html.
        if relative.as_posix() == "index.html":
            continue
        expected = _digest(source)
        if _digest(build / relative) != expected:
            raise ValueError(f"built UI asset differs from public source: {relative}")
        if installed is not None and _digest(Path(installed) / relative) != expected:
            raise ValueError(f"installed UI asset differs from public source: {relative}")
        count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frontend", required=True, type=Path)
    parser.add_argument("--build", required=True, type=Path)
    parser.add_argument("--installed", type=Path)
    args = parser.parse_args()
    try:
        count = verify_assets(args.frontend, args.build, args.installed)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"[frontend_assets] ERROR: {exc}; rebuild using ./colcon_build.sh --packages-select camrod_ui\n")
    print(f"[frontend_assets] verified {count} public assets, including Ranger GLB and all livery textures")


if __name__ == "__main__":
    main()
