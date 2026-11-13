#!/usr/bin/env python3
"""Install and serve the optional Lichtblick web app in the devcontainer"""

import argparse
import functools
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import tarfile
import tempfile
from urllib.parse import quote
from urllib.request import Request, urlopen


def install(directory: Path, version: str) -> None:
    directory = directory.resolve()
    if (directory / "index.html").is_file():
        return
    if directory.exists():
        raise ValueError(f"Web directory exists without index.html: {directory}")

    tag = "v" + version.removeprefix("v")
    request = Request(
        "https://api.github.com/repos/lichtblick-suite/lichtblick/releases/tags/" + quote(tag, safe=""),
        headers={"Accept": "application/vnd.github+json", "User-Agent": "ros-visualizer-launcher"},
    )
    with urlopen(request, timeout=30) as response:
        release = json.load(response)
    asset = next((item for item in release["assets"] if item["name"] == "lichtblick-web.tar.gz"), None)
    if asset is None:
        raise ValueError(f"Release {tag} does not provide lichtblick-web.tar.gz")

    directory.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading Lichtblick {tag} web assets ({asset['size'] / 1024 / 1024:.1f} MiB)", flush=True)
    with tempfile.TemporaryDirectory(prefix=".lichtblick-download-", dir=directory.parent) as temporary:
        temporary = Path(temporary)
        archive = temporary / "lichtblick-web.tar.gz"
        digest = hashlib.sha256()
        with urlopen(asset["browser_download_url"], timeout=60) as response, archive.open("wb") as output:
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
                digest.update(chunk)
        expected_digest = asset.get("digest")
        if expected_digest and expected_digest != "sha256:" + digest.hexdigest():
            raise ValueError("Lichtblick web archive checksum does not match the release")

        unpacked = temporary / "web"
        unpacked.mkdir()
        with tarfile.open(archive, "r:gz") as bundle:
            bundle.extractall(unpacked, filter="data")
        if not (unpacked / "index.html").is_file():
            children = list(unpacked.iterdir())
            if len(children) == 1 and children[0].is_dir():
                unpacked = children[0]
        if not (unpacked / "index.html").is_file():
            raise ValueError("Downloaded archive does not contain the Lichtblick web application")
        if (directory / "index.html").is_file():
            return  # Another launcher completed the same download.
        unpacked.rename(directory)
    print(f"Lichtblick web assets cached in {directory}", flush=True)


class WebHandler(SimpleHTTPRequestHandler):
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".wasm": "application/wasm",
        ".foxe": "application/octet-stream",
    }

    def __init__(self, *args, directory: Path, overlay: Path, **kwargs):
        self.web_root = directory.resolve()
        self.overlay_root = overlay.resolve()
        super().__init__(*args, directory=str(self.web_root), **kwargs)

    def translate_path(self, path):
        original = Path(super().translate_path(path))
        relative = original.relative_to(self.web_root)
        overlay = self.overlay_root / relative
        if overlay.is_file():
            return str(overlay)
        if not original.exists() and not relative.suffix:
            return str(self.web_root / "index.html")
        return str(original)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    installer = commands.add_parser("install", help="Download and cache the official web bundle")
    installer.add_argument("--version", required=True)
    installer.add_argument("--directory", type=Path, required=True)
    server = commands.add_parser("serve", help="Serve cached assets with the visualizer layout and extension")
    server.add_argument("--directory", type=Path, required=True)
    server.add_argument("--overlay", type=Path, required=True)
    server.add_argument("--bind", default="0.0.0.0")
    server.add_argument("--port", type=int, default=8080)
    arguments = parser.parse_args()
    try:
        if arguments.command == "install":
            install(arguments.directory, arguments.version)
            return
        if not (arguments.directory / "index.html").is_file():
            raise ValueError(f"Lichtblick index.html not found in {arguments.directory}")
        handler = functools.partial(WebHandler, directory=arguments.directory, overlay=arguments.overlay)
        with ThreadingHTTPServer((arguments.bind, arguments.port), handler) as http:
            print(f"Lichtblick HTTP server listening on {arguments.bind}:{arguments.port}", flush=True)
            http.serve_forever()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, tarfile.TarError) as error:
        parser.exit(1, f"Lichtblick: {error}\n")


if __name__ == "__main__":
    main()
