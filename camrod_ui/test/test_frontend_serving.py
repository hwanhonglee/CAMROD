"""HH_261006 - Deployment HTTP checks must distinguish models from SPA fallback."""

import asyncio
from pathlib import Path
import sys
from types import SimpleNamespace

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
from camrod_ui.frontend_serving import (  # noqa: E402
    configured_frontend_path, find_frontend_directory, register_frontend_routes,
)


@pytest.fixture
def frontend(tmp_path):
    root = tmp_path / "build"
    (root / "models").mkdir(parents=True)
    (root / "static/js").mkdir(parents=True)
    (root / "index.html").write_text("<!doctype html><title>CAMROD</title>")
    (root / "models/ranger-navigation.glb").write_bytes(b"glTF-fixture")
    (root / "models/woraksan-rear-wrap.png").write_bytes(b"PNG-fixture")
    (root / "static/js/main.hash.js").write_text("/* bundled code */")
    return root


def client_for(root):
    app = FastAPI()

    @app.get("/api/state")
    def state():
        return {"connected": True}

    register_frontend_routes(app, root)
    return TestClient(app)


def head_request(client, path):
    # HH_261006 - Test actual ASGI HEAD frames: old Starlette's requests adapter
    # misreads bodyless HEAD responses with newer urllib3 content-length checks.
    async def run():
        messages = []

        async def receive():
            return {"type": "http.request", "body": b""}

        async def send(message):
            messages.append(message)

        await client.app({
            "type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1",
            "method": "HEAD", "scheme": "http", "path": path, "raw_path": path.encode(),
            "query_string": b"", "root_path": "", "headers": [],
            "client": ("127.0.0.1", 12345), "server": ("testserver", 80),
        }, receive, send)
        start = next(message for message in messages if message["type"] == "http.response.start")
        return SimpleNamespace(
            status_code=start["status"],
            headers={key.decode(): value.decode() for key, value in start["headers"]},
            content=b"".join(message.get("body", b"") for message in messages
                             if message["type"] == "http.response.body"),
        )
    return asyncio.run(run())


def test_empty_configuration_never_resolves_to_working_directory(tmp_path, monkeypatch):
    (tmp_path / "index.html").write_text("unrelated working directory")
    monkeypatch.chdir(tmp_path)
    assert configured_frontend_path("") is None
    assert configured_frontend_path("  ") is None
    assert find_frontend_directory([configured_frontend_path("")]) is None


def test_resolution_skips_incomplete_builds_and_preserves_priority(frontend, tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    assert find_frontend_directory([None, empty, frontend]) == frontend.resolve()
    configured = tmp_path / "override"
    configured.mkdir()
    (configured / "index.html").write_text("operator override")
    assert find_frontend_directory([configured, frontend]) == configured.resolve()


@pytest.mark.parametrize("url", [
    "/", "/navigation", "/operator/diagnostics", "/index.html",
])
def test_spa_routes_still_serve_non_cached_html(frontend, url):
    response = client_for(frontend).get(url)
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]
    assert "no-store" in response.headers["cache-control"]


@pytest.mark.parametrize("url", [
    "/models/missing.glb", "/models/missing.png", "/models/extensionless",
    "/static/js/stale.js", "/assets/missing", "/favicon.ico", "/api/missing",
    "/ui/not-an-endpoint",
])
def test_missing_assets_and_endpoints_are_404_not_successful_html(frontend, url):
    response = client_for(frontend).get(url)
    assert response.status_code == 404
    assert "text/html" not in response.headers["content-type"]


def test_assets_and_explicit_api_routes_remain_available(frontend):
    client = client_for(frontend)
    model = client.get("/models/ranger-navigation.glb")
    assert model.status_code == 200
    assert model.content == b"glTF-fixture"
    assert model.headers["cache-control"] == "no-cache"
    assert client.get("/models/woraksan-rear-wrap.png").content == b"PNG-fixture"
    assert "immutable" in client.get("/static/js/main.hash.js").headers["cache-control"]
    assert client.get("/api/state").json() == {"connected": True}


@pytest.mark.parametrize("url, status", [
    ("/models/ranger-navigation.glb", 200), ("/navigation", 200),
    ("/models/missing.glb", 404),
])
def test_head_has_get_status_and_metadata_without_body(frontend, url, status):
    client = client_for(frontend)
    head = head_request(client, url)
    get = client.get(url)
    assert head.status_code == get.status_code == status
    assert head.content == b""
    assert head.headers["content-length"] == get.headers["content-length"]
    assert head.headers["content-type"] == get.headers["content-type"]


def test_symlink_install_assets_may_point_to_the_source_tree(frontend, tmp_path):
    source_asset = tmp_path / "source-only.glb"
    source_asset.write_bytes(b"linked-source-asset")
    (frontend / "models/linked.glb").symlink_to(source_asset)
    response = client_for(frontend).get("/models/linked.glb")
    assert response.status_code == 200
    assert response.content == b"linked-source-asset"


@pytest.mark.parametrize("url", [
    "/%2e%2e/private.txt", "/models/%2e%2e/%2e%2e/private.txt",
    "/models/%5c..%5cprivate.txt", "/.env", "/models/%00.glb",
])
def test_traversal_and_hidden_files_are_not_served(frontend, tmp_path, url):
    (tmp_path / "private.txt").write_text("must not be served")
    (frontend / ".env").write_text("must not be served")
    response = client_for(frontend).get(url)
    assert response.status_code == 404
    assert "must not be served" not in response.text


def test_missing_build_is_503_including_head_and_model_probes(tmp_path):
    client = client_for(tmp_path)
    for url in ("/", "/models/ranger-navigation.glb"):
        assert client.get(url).status_code == 503
        head = head_request(client, url)
        assert head.status_code == 503
        assert head.content == b""
