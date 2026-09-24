from aag_yam_sim.upstream import REQUIRED_UPSTREAM_FILES, validate_upstream


def test_validate_upstream_reports_all_missing_files(tmp_path):
    assert validate_upstream(tmp_path) == list(REQUIRED_UPSTREAM_FILES)


def test_validate_upstream_accepts_expected_layout(tmp_path):
    for name in REQUIRED_UPSTREAM_FILES:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    assert validate_upstream(tmp_path) == []
