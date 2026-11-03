"""HH_261103 - Unexpected artifact drift must not be laundered into a fresh receipt."""
import hashlib
from pathlib import Path
import pytest
from create_receipts import prior_paths, require_pass


def test_prior_artifact_must_match_or_be_explicitly_reviewed(tmp_path):
    path = tmp_path / 'binary'
    path.write_bytes(b'old')
    document = {'files': {'binary': {'path': str(path), 'sha256': hashlib.sha256(b'old').hexdigest()}}}
    assert prior_paths(document, set()) == [path]
    path.write_bytes(b'new')
    with pytest.raises(ValueError, match='Unreviewed'):
        prior_paths(document, set())
    assert prior_paths(document, {path}) == [path]


@pytest.mark.parametrize('tests,failures,errors', [(0, 0, 0), (1, 1, 0), (1, 0, 1)])
def test_empty_or_failed_suite_is_rejected(tmp_path, tests, failures, errors):
    path = tmp_path / 'result.xml'
    path.write_text(f'<testsuite tests="{tests}" failures="{failures}" errors="{errors}"/>')
    with pytest.raises(ValueError):
        require_pass(path)


def test_actual_passing_suite_is_accepted(tmp_path):
    path = tmp_path / 'result.xml'
    path.write_text('<testsuites><testsuite tests="2" failures="0" errors="0"/></testsuites>')
    require_pass(path)
