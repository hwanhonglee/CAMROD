"""Fail-closed contracts for the visible Guest UI CARLA acceptance path."""

import os
from pathlib import Path
import subprocess


SRC_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_ROOT = SRC_ROOT / "scripts" / "virtual_carla"
RUNNER = SCRIPT_ROOT / "run.sh"
ENVIRONMENT = SCRIPT_ROOT / "env.sh"


def test_guest_environment_and_commands_are_explicit() -> None:
    env_source = ENVIRONMENT.read_text(encoding="utf-8")
    run_source = RUNNER.read_text(encoding="utf-8")

    assert 'CAMROD_GUEST_UI_URL:-http://127.0.0.1:8012' in env_source
    assert 'CAMROD_GUEST_CDP_URL:-http://127.0.0.1:9223' in env_source
    assert "guest-ui)" in run_source
    assert "camping-sites)" in run_source
    assert "camping-sites-recall)" in run_source
    assert "camping-sites-guest)" in run_source
    assert 'matrix_return_authority="operator_rest"\n' in run_source
    assert 'matrix_mission_intent="delivery"' in run_source
    assert 'matrix_mission_intent="recall"' in run_source
    assert '"--app=${CAMROD_GUEST_UI_URL}"' in run_source
    assert "--remote-debugging-address=127.0.0.1" in run_source
    assert "--remote-debugging-port=9223" in run_source
    assert '"--remote-allow-origins=*"' in run_source
    assert 'mktemp -d "${guest_browser_temp_root}/camrod-guest-chrome.XXXXXX"' in run_source
    assert "guest_browser_target_ready" in run_source
    assert "--return-authority \"${matrix_return_authority}\"" in run_source
    assert "--mission-intent \"${matrix_mission_intent}\"" in run_source
    assert "--guest-ui-url \"${CAMROD_GUEST_UI_URL}\"" in run_source
    assert "--guest-cdp-url \"${CAMROD_GUEST_CDP_URL}\"" in run_source
    assert "camrod_camping_site_matrix_guest_usage_complete" in run_source


def test_visible_operator_browser_commands_are_isolated_and_explicit() -> None:
    env_source = ENVIRONMENT.read_text(encoding="utf-8")
    run_source = RUNNER.read_text(encoding="utf-8")

    assert 'CAMROD_OPERATOR_CDP_URL:-http://127.0.0.1:9224' in env_source
    assert "operator-ui)" in run_source
    assert "camping-sites-browser)" in run_source
    assert "camping-sites-browser-recall)" in run_source
    assert 'matrix_return_authority="operator_browser"' in run_source
    assert '"--app=${CAMROD_UI_URL}"' in run_source
    assert "--remote-debugging-port=9224" in run_source
    assert 'mktemp -d "${operator_browser_temp_root}/camrod-operator-chrome.XXXXXX"' in run_source
    assert "operator_browser_target_ready" in run_source
    assert '--operator-cdp-url "${CAMROD_OPERATOR_CDP_URL}"' in run_source
    assert "camrod_camping_site_matrix_operator_browser_delivery" in run_source
    assert "camrod_camping_site_matrix_operator_browser_recall" in run_source


def test_guest_commands_are_listed_in_help() -> None:
    result = subprocess.run(
        [str(RUNNER), "--help"],
        cwd="/tmp",
        check=True,
        capture_output=True,
        text=True,
    )
    assert "guest-ui" in result.stdout
    assert "camping-sites-guest" in result.stdout
    assert "camping-sites-recall" in result.stdout
    assert "navigate -> usage_complete" in result.stdout
    assert "operator-ui" in result.stdout
    assert "camping-sites-browser" in result.stdout


def test_guest_ui_refuses_headless_or_noncanonical_endpoints() -> None:
    environment = os.environ.copy()
    for name in (
        "DISPLAY",
        "RANGER_CARLA_ROOT",
        "RANGER_ENV_FILE",
        "CAMROD_GUEST_UI_URL",
        "CAMROD_GUEST_CDP_URL",
    ):
        environment.pop(name, None)

    headless = subprocess.run(
        [str(RUNNER), "guest-ui"],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
    )
    assert headless.returncode != 0
    assert "guest-ui requires DISPLAY" in headless.stderr

    environment.update(
        {
            "DISPLAY": ":99",
            "CAMROD_GUEST_UI_URL": "http://0.0.0.0:8012",
            "CAMROD_GUEST_CDP_URL": "http://127.0.0.1:9223",
        }
    )
    noncanonical = subprocess.run(
        [str(RUNNER), "guest-ui"],
        cwd="/tmp",
        env=environment,
        capture_output=True,
        text=True,
    )
    assert noncanonical.returncode != 0
    assert "must be exactly the local Guest UI endpoint" in noncanonical.stderr
    assert "refusing Guest UI launch with a non-canonical endpoint" in noncanonical.stderr
