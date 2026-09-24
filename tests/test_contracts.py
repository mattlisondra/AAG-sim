import pytest

from aag_yam_sim.contracts import YAM_CONTRACT, normalize_server_url


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ("127.0.0.1:8202", "http://127.0.0.1:8202/act"),
        ("http://gpu:8202", "http://gpu:8202/act"),
        ("https://example.test/act/", "https://example.test/act"),
    ],
)
def test_normalize_server_url(raw, expected):
    assert normalize_server_url(raw) == expected


def test_yam_contract_is_fixed():
    assert YAM_CONTRACT.camera_keys == ("top_cam", "left_cam", "right_cam")
    assert YAM_CONTRACT.state_dim == YAM_CONTRACT.action_dim == 14
    assert YAM_CONTRACT.norm_tag == "yam_dual_molmoact2"
