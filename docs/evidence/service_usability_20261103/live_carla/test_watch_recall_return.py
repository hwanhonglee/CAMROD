"""HH_261103 - Check recall identity and both canonical confirmation stages without live actions."""
import importlib.util
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location("recall_watch_guard_test", ROOT / "watch_recall_return.py")
watch = importlib.util.module_from_spec(spec); spec.loader.exec_module(watch)


def state():
    return {"mission_dispatch_active": True, "mission_dispatch_generation": 123, "mission_dispatch_site": "B7",
            "mission_dispatch_owner": "robot", "mission_dispatch_intent": "recall", "service_state": 8}


def test_identity_rejects_other_site_generation_owner_and_intent():
    value = state()
    assert watch.validate_recall_identity(value, 123, "B7")["owner"] == "robot"
    for key, wrong in [("mission_dispatch_generation", 124), ("mission_dispatch_owner", "operator"),
            ("mission_dispatch_owner", "guest"), ("mission_dispatch_intent", "delivery"),
            ("mission_dispatch_site", "B6"), ("mission_dispatch_active", False)]:
        with pytest.raises((RuntimeError, watch.matrix.MatrixError)):
            watch.validate_recall_identity({**value, key: wrong}, 123, "B7")


def test_two_stage_context_requires_actual_loading_wait_and_final_readiness():
    value = state(); identity = watch.validate_recall_identity(value, 123, "B7")
    view = {"service_state": {"state": 8}, "site": {"operating_state": "WAIT_RETURN"}}
    assert watch.matrix.return_context(value, view, identity)["final_return"] is False
    view["site"]["operating_state"] = "RECALL_RETURN_WAIT"
    with pytest.raises(watch.matrix.MatrixError):
        watch.matrix.return_context(value, view, identity, final_return=True)
    assert watch.matrix.return_context({**value, "recall_final_return_ready": True}, view, identity,
                                       final_return=True)["final_return"] is True
