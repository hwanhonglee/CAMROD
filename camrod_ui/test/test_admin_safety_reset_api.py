"""Exercise the authenticated admin reset API without ROS or motion output."""

from pathlib import Path
import sys

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.safety_reset_api import register_safety_reset_routes  # noqa: E402


RESET_PATH = "/api/admin/safety/reset"


class _Node:
    def __init__(self) -> None:
        self.actions = []
        self.status = {"available": True, "busy": False}
        self.result = {"success": True, "message": "reset requested"}

    def get_admin_safety_reset_status(self) -> dict:
        return dict(self.status)

    async def request_admin_safety_reset(self, confirm: bool) -> dict:
        self.actions.append(confirm)
        return dict(self.result)


@pytest.fixture
def api(monkeypatch):
    # HH_261024 - Exercise actual HTTP authentication and validation, not a
    # copied route or a direct invocation that could bypass its guards.
    monkeypatch.setenv("CAMROD_ADMIN_USER", "test-admin")
    monkeypatch.setenv("CAMROD_ADMIN_PASSWORD", "test-password")
    node = _Node()
    app = FastAPI()
    register_safety_reset_routes(app, node)
    with TestClient(app) as client:
        yield client, node


def _auth():
    return "test-admin", "test-password"


@pytest.mark.parametrize("method", ["get", "post"])
def test_safety_reset_requires_admin_authentication(api, method):
    client, node = api
    request = getattr(client, method)
    kwargs = {"json": {"confirm": True}} if method == "post" else {}
    for auth in (None, ("test-admin", "wrong"), ("wrong", "test-password")):
        response = request(RESET_PATH, auth=auth, **kwargs)
        assert response.status_code == 401
        assert response.headers["www-authenticate"].lower().startswith("basic")
    assert node.actions == []


def test_safety_reset_status_reads_never_release_or_resume_motion(api):
    client, node = api
    for _ in range(3):
        response = client.get(RESET_PATH, auth=_auth())
        assert response.status_code == 200
        assert response.json()["available"] is True
    assert node.actions == []


@pytest.mark.parametrize(
    "body", [{}, {"confirm": False}, {"confirm": "true"}, {"confirm": 1},
             {"confirm": []}, {"confirm": None}, {"confirm": {"yes": True}}],
)
def test_safety_reset_requires_literal_true_not_truthy_values(api, body):
    # HH_261024 - An admin must explicitly confirm this one-shot operation;
    # strings, numbers and stale empty form submissions grant no authority.
    client, node = api
    response = client.post(RESET_PATH, auth=_auth(), json=body)
    assert response.status_code in (400, 422)
    assert node.actions == []


@pytest.mark.parametrize("body", [[], True, "reset", 1, None])
def test_safety_reset_rejects_non_object_json_without_side_effects(api, body):
    client, node = api
    response = client.post(RESET_PATH, auth=_auth(), json=body)
    assert response.status_code in (400, 422)
    assert node.actions == []


def test_safety_reset_rejects_malformed_json_without_side_effects(api):
    client, node = api
    response = client.post(
        RESET_PATH, auth=_auth(), data='{"confirm":',
        headers={"Content-Type": "application/json"},
    )
    assert response.status_code in (400, 422)
    assert node.actions == []


def test_safety_reset_explicit_admin_confirmation_calls_once(api):
    client, node = api
    response = client.post(RESET_PATH, auth=_auth(), json={"confirm": True})
    assert response.status_code == 200
    assert response.json()["success"] is True
    assert node.actions == [True]


@pytest.mark.parametrize(
    ("result", "http_status"),
    [
        ({"success": False, "available": False, "error": "unavailable"}, 503),
        ({"success": False, "busy": True, "error": "reset_in_progress"}, 409),
        ({"success": False, "error": "physical_estop_active"}, 409),
    ],
)
def test_safety_reset_propagates_unavailable_busy_and_blocked_results(api, result, http_status):
    # HH_261024 - Reporting an accepted reset is different from proving every
    # safety condition cleared; show blocked results rather than fake success.
    client, node = api
    node.result = result
    response = client.post(RESET_PATH, auth=_auth(), json={"confirm": True})
    assert response.status_code == http_status
    assert response.json() == result
    assert node.actions == [True]


@pytest.mark.parametrize("method", ["get", "post"])
@pytest.mark.parametrize(
    "origin", ["http://evil.example", "http://testserver.evil.example",
               "https://testserver", "null", "http://["],
)
def test_safety_reset_rejects_cross_origin_and_malformed_origin_before_any_action(api, method, origin):
    client, node = api
    kwargs = {"json": {"confirm": True}} if method == "post" else {}
    response = getattr(client, method)(
        RESET_PATH, auth=_auth(), headers={"Origin": origin}, **kwargs
    )
    assert response.status_code == 403
    assert node.actions == []


def test_safety_reset_accepts_matching_origin_but_get_is_still_read_only(api):
    client, node = api
    headers = {"Origin": "http://testserver"}
    assert client.get(RESET_PATH, auth=_auth(), headers=headers).status_code == 200
    assert node.actions == []
    response = client.post(
        RESET_PATH, auth=_auth(), json={"confirm": True}, headers=headers
    )
    assert response.status_code == 200
    assert node.actions == [True]


@pytest.mark.parametrize("authorization", ["", "Bearer token", "Basic !!!", "Basic Og=="])
def test_safety_reset_invalid_authorization_never_reaches_node(api, authorization):
    client, node = api
    response = client.post(
        RESET_PATH, json={"confirm": True}, headers={"Authorization": authorization}
    )
    assert response.status_code == 401
    assert node.actions == []


def test_safety_reset_uses_existing_admin_defaults_when_not_overridden(monkeypatch):
    # HH_261024 - Keep the robot's existing local admin credentials compatible
    # while allowing deployments to replace them through the environment.
    monkeypatch.delenv("CAMROD_ADMIN_USER", raising=False)
    monkeypatch.delenv("CAMROD_ADMIN_PASSWORD", raising=False)
    node = _Node()
    app = FastAPI()
    register_safety_reset_routes(app, node)
    with TestClient(app) as client:
        response = client.get(RESET_PATH, auth=("admin", "1234"))
        assert response.status_code == 200
        assert client.get(RESET_PATH, auth=_auth()).status_code == 401
    assert node.actions == []
