from shapely.geometry import box

from battery_route_planner import plan_sorties


def main() -> None:
    # The ordered route is assumed to come from an earlier coverage-planning stage.
    route = [
        [20.0, 0.0],
        [40.0, 0.0],
        [60.0, 0.0],
        [80.0, 0.0],
    ]
    stations = [
        [0.0, 30.0],
        [100.0, 30.0],
    ]
    obstacles = [box(45.0, -10.0, 55.0, 10.0)]

    result = plan_sorties(
        route=route,
        stations=stations,
        obstacles=obstacles,
        battery_range_m=140.0,
    )

    print(f"Sorties: {result.sortie_count}")
    print(f"Total flight distance: {result.total_distance_m:.2f} m")
    print()

    for index, sortie in enumerate(result.sorties, start=1):
        print(
            f"Sortie {index}: route points {sortie.start_index}..{sortie.end_index}, "
            f"station {sortie.start_station} -> station {sortie.end_station}"
        )
        print(f"  distance: {sortie.total_distance_m:.2f} m")
        print(f"  path: {sortie.path}")


if __name__ == "__main__":
    main()
