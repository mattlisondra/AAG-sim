from aag_yam_sim.benchmark import scenario_by_id, scenarios


def test_benchmark_has_five_increasing_scenarios():
    items = scenarios()
    assert len(items) == 5
    assert [item["difficulty"] for item in items] == [1, 2, 3, 4, 5]
    assert len({item["id"] for item in items}) == 5


def test_each_scenario_is_explicitly_not_yet_an_environment():
    for item in scenarios():
        assert item["status"] == "specification"
        assert item["broad_instruction"]
        assert item["resolved_instruction"]
        assert len(item["subtasks"]) >= 3


def test_scenario_lookup():
    assert scenario_by_id("bedside-assistance")["difficulty"] == 3
