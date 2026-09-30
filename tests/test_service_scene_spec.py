from aag_yam_sim.benchmark import scenarios
from aag_yam_sim.service_scene_spec import (
    service_scene_by_env_id,
    service_scene_layout,
    service_scene_layouts,
)


def test_preview_layouts_match_benchmark_scenarios():
    benchmark = scenarios()
    layouts = service_scene_layouts()

    assert len(layouts) == 5
    assert [layout["id"] for layout in layouts] == [item["id"] for item in benchmark]
    assert [layout["env_id"] for layout in layouts] == [item["env_id"] for item in benchmark]
    assert [layout["preview_env_id"] for layout in layouts] == [
        item["preview_env_id"] for item in benchmark
    ]


def test_layout_objects_have_unique_names_and_supported_kinds():
    supported = {"ycb", "box", "disc", "bottle", "open_container", "rack"}
    for layout in service_scene_layouts():
        objects = layout["objects"]
        names = [item["name"] for item in objects]

        assert 6 <= len(objects) <= 10
        assert len(names) == len(set(names))
        for item in objects:
            assert item["kind"] in supported
            assert len(item["position_xy"]) == 2
            assert item["label"]
            if item["kind"] == "ycb":
                assert item["asset_id"]
                assert item.get("scale", 1.0) > 0


def test_layout_lookup_by_scene_and_environment():
    layout = service_scene_layout("bedside-assistance")
    assert layout["env_id"] == "AAGBedsideAssistance-v1"
    assert service_scene_by_env_id(layout["env_id"]) == layout
    assert service_scene_by_env_id(layout["preview_env_id"]) == layout


def test_task_predicates_reference_known_objects_and_have_expected_counts():
    expected_counts = [3, 3, 3, 3, 6]
    for layout, scenario, expected_count in zip(
        service_scene_layouts(), scenarios(), expected_counts, strict=True
    ):
        objects = {item["name"]: item for item in layout["objects"]}
        predicates = [predicate for stage in layout["stages"] for predicate in stage["predicates"]]
        assert len(predicates) == expected_count
        assert scenario["atomic_subtask_count"] == expected_count
        assert len({item["id"] for item in predicates}) == expected_count
        for predicate in predicates:
            assert predicate["relation"] in {"in", "on", "beside"}
            assert predicate["object"] in objects
            assert predicate["target"] in objects
            assert not objects[predicate["object"]].get("static", False)
            assert len(predicate["goal_offset_xy"]) == 2
            if "support" in predicate:
                assert predicate["support"] in objects
