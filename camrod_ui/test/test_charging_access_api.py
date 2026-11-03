"""HH_261102 - Test the real cargo-access HTTP boundary without sending motion."""

from pathlib import Path
import sys

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
from camrod_ui.charging_access_api import register_charging_access_routes


class Node:
    def __init__(self):
        self.actions = []
        self.result = {"success": True, "phase": "departing"}

    def charging_access_snapshot(self):
        return {"phase": "inactive", "is_charging": True}

    def request_charging_access(self, action, confirm):
        self.actions.append((action, confirm))
        return self.result


@pytest.fixture
def api():
    node = Node()
    app = FastAPI()
    register_charging_access_routes(app, node)
    with TestClient(app) as client:
        yield client, node


def test_read_and_refresh_do_not_start_departure(api):
    client, node = api
    for _ in range(3):
        assert client.get("/api/charging-access").json()["phase"] == "inactive"
    assert client.get("/ui/charging_access?action=leave&confirm=true").status_code == 405
    assert node.actions == []


@pytest.mark.parametrize("action", ["leave"])
def test_confirmed_actions_reach_existing_controller_once(api, action):
    client, node = api
    response = client.post(
        f"/ui/charging_access?action={action}&confirm=true",
        headers={"Origin": "http://testserver"})
    assert response.status_code == 200
    assert node.actions == [(action, True)]


@pytest.mark.parametrize("query", [
    "", "action=leave", "action=leave&confirm=1", "action=leave&confirm=True",
    "action=leave&confirm=false", "action=unknown&confirm=true",
    "action=redock&confirm=true",
])
def test_missing_or_truthy_confirmation_cannot_move(api, query):
    client, node = api
    assert client.post(f"/ui/charging_access?{query}").status_code == 400
    assert node.actions == []


@pytest.mark.parametrize("origin", ["http://other-host", "https://testserver", "http://["])
def test_foreign_browser_origin_is_rejected(api, origin):
    client, node = api
    response = client.post("/ui/charging_access?action=leave&confirm=true",
                           headers={"Origin": origin})
    assert response.status_code == 403
    assert node.actions == []


def test_controller_rejection_is_not_reported_as_success(api):
    client, node = api
    node.result = {"success": False, "error": "battery_not_ready"}
    response = client.post("/ui/charging_access?action=leave&confirm=true")
    assert response.status_code == 409
    assert response.json()["error"] == "battery_not_ready"
