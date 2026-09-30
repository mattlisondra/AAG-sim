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
    assert [layout["env_id"] for layout in layouts] == [
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
    assert layout["env_id"] == "AAGBedsideAssistancePreview-v0"
    assert service_scene_by_env_id(layout["env_id"]) == layout
