from benchmarks.datasets import synthetic_instance
from benchmarks.run import run_instance
from droneplan.planner import Algorithm


def test_synthetic_instances_are_deterministic() -> None:
    a, b = synthetic_instance(4, with_obstacles=True), synthetic_instance(4, with_obstacles=True)
    assert a.area.equals(b.area) and (a.stations == b.stations).all()
    assert len(a.obstacles) == len(b.obstacles)


def test_obstacle_variant_differs_only_in_obstacles() -> None:
    for seed in range(5):
        a, b = synthetic_instance(seed), synthetic_instance(seed, with_obstacles=True)
        assert a.area.equals(b.area) and (a.stations == b.stations).all()
        assert a.config == b.config and b.obstacles


def test_run_instance_smoke() -> None:
    for with_obstacles in (False, True):
        row = run_instance(synthetic_instance(0, with_obstacles=with_obstacles), Algorithm.RSS)
        assert row["valid"] is True and row["makespan_s"] > 0
