from __future__ import annotations

import time
from collections.abc import Callable
from typing import NamedTuple

from ortools.sat.python import cp_model


class Variables[T](NamedTuple):
    building_x: dict[str, T]
    building_y: dict[str, T]
    building_dx: dict[str, T]
    building_dy: dict[str, T]
    building_orientation: dict[str, T]
    conveyor_start_x: list[T]
    conveyor_start_y: list[T]
    conveyor_segment_end_x: list[list[T]]
    conveyor_segment_end_y: list[list[T]]
    conveyor_orientation: list[list[T]]
    conveyor_mask: list[list[T]]
    pickup_perm: list[T]
    objective: T

    def map[U](self, f: Callable[[T], U]) -> Variables[U]:
        def walk(a):
            if isinstance(a, dict):
                return {k: walk(v) for k, v in a.items()}
            elif isinstance(a, list):
                return [walk(v) for v in a]
            else:
                return f(a)

        return Variables(*[walk(a) for a in self])


class Building(NamedTuple):
    name: str
    width: int
    height: int


class Pickup(NamedTuple):
    x: int
    y: int
    orientation: int  # 0=North 1=South 2=East 3=West


class Data(NamedTuple):
    gridrange_x: int
    gridrange_y: int
    buildings: list[Building]
    conveyor_joins: list[tuple[str | None, str]]
    pickups: list[Pickup]
    # um yeah this is kinda stupid
    hints: Callable[
        [Variables[None]],
        Variables[int | None],
    ] | None = None


def create_model(data: Data):
    (
        gridrange_x,
        gridrange_y,
        buildings,
        conveyor_joins,
        pickups,
        _,
    ) = data
    building_names = [building.name for building in buildings]

    model = cp_model.CpModel()

    building_x = {b: model.new_int_var(0, gridrange_x, "") for b in building_names}
    building_y = {b: model.new_int_var(0, gridrange_y, "") for b in building_names}
    building_dx = {b: model.new_int_var(0, gridrange_x, "") for b in building_names}
    building_dy = {b: model.new_int_var(0, gridrange_y, "") for b in building_names}
    # Ordering is [North, South, East, West]
    building_orientation = {b: model.new_int_var(0, 3, "") for b in building_names}
    building_is_ns = {b: model.new_bool_var("") for b in building_names}

    for b in buildings:
        is_ns = building_is_ns[b.name]
        model.add(building_orientation[b.name] < 2).only_enforce_if(is_ns)
        model.add(building_orientation[b.name] >= 2).only_enforce_if(~is_ns)
        model.add(building_dx[b.name] == b.width).only_enforce_if(is_ns)
        model.add(building_dy[b.name] == b.height).only_enforce_if(is_ns)
        model.add(building_dx[b.name] == b.height).only_enforce_if(~is_ns)
        model.add(building_dy[b.name] == b.width).only_enforce_if(~is_ns)

    building_interval_x_end = {
        b: model.new_int_var(0, gridrange_x + 1, "") for b in building_names
    }
    building_interval_x = [
        model.new_interval_var(
            building_x[b], building_dx[b], building_interval_x_end[b], ""
        )
        for b in building_names
    ]
    building_interval_y = [
        model.new_interval_var(
            building_y[b], building_dy[b], model.new_int_var(0, gridrange_y + 1, ""), ""
        )
        for b in building_names
    ]

    model.add_no_overlap_2d(building_interval_x, building_interval_y)

    turns = 3
    conveyor_start_x = [model.new_int_var(0, gridrange_x, "") for _ in conveyor_joins]
    conveyor_start_y = [model.new_int_var(0, gridrange_y, "") for _ in conveyor_joins]
    conveyor_end_x = [model.new_int_var(0, gridrange_x, "") for _ in conveyor_joins]
    conveyor_end_y = [model.new_int_var(0, gridrange_y, "") for _ in conveyor_joins]
    conveyor_length = [
        [model.new_int_var(1, max(gridrange_x, gridrange_y), "") for _ in range(turns)]
        for _ in conveyor_joins
    ]
    conveyor_orientation = [
        [model.new_int_var(0, 3, "") for _ in range(turns)] for _ in conveyor_joins
    ]
    conveyor_turns = [model.new_int_var(1, turns, "") for _ in conveyor_joins]

    model.add(conveyor_turns[0] == 1)  # TODO: remove

    # First bunch of conveyors are all our pickups
    assert all(b is None for b, _ in conveyor_joins[: len(pickups)]), "rip"
    assert sum(b is None for b, _ in conveyor_joins) == len(pickups), "bozo"
    pickup_perm = [model.new_int_var(0, len(pickups) - 1, "") for _ in pickups]
    model.add_all_different(pickup_perm)

    for c, perm in enumerate(pickup_perm):
        for p, pickup in enumerate(pickups):
            in_use = model.new_bool_var("")
            model.add(perm == p).only_enforce_if(in_use)
            model.add(perm != p).only_enforce_if(~in_use)
            model.add(conveyor_start_x[c] == pickup.x).only_enforce_if(in_use)
            model.add(conveyor_start_y[c] == pickup.y).only_enforce_if(in_use)
            model.add(conveyor_orientation[c][0] == pickup.orientation).only_enforce_if(
                in_use
            )

    # Make sure conveyors for each building don't go to the same port
    model.add_all_different().only_enforce_if  # type shi
    for b in building_names:
        end_x_to_b = [
            x for x, (_, dst) in zip(conveyor_end_x, conveyor_joins) if dst == b
        ]
        model.add_all_different(end_x_to_b).only_enforce_if(building_is_ns[b])
        end_y_to_b = [
            y for y, (_, dst) in zip(conveyor_end_y, conveyor_joins) if dst == b
        ]
        model.add_all_different(end_y_to_b).only_enforce_if(~building_is_ns[b])

    building_nsew = {
        b: [model.new_bool_var("") for _ in range(4)] for b in building_names
    }
    for b, nsew in building_nsew.items():
        for i, v in enumerate(nsew):
            model.add(building_orientation[b] == i).only_enforce_if(v)
            model.add(building_orientation[b] != i).only_enforce_if(~v)

    # Conveyor start and end location and orientation
    for c, (b1, b2) in enumerate(conveyor_joins):
        if isinstance(b1, str):
            model.add(conveyor_orientation[c][0] == building_orientation[b1])

            b1_n, b1_s, b1_e, b1_w, b1_ns = *building_nsew[b1], building_is_ns[b1]
            b1_x, b1_y = building_x[b1], building_y[b1]
            b1_dx, b1_dy = building_dx[b1], building_dy[b1]
            # NS -> x in [building_x, building_x + building_dx)
            model.add(conveyor_start_x[c] >= b1_x).only_enforce_if(b1_ns)
            model.add(conveyor_start_x[c] < b1_x + b1_dx).only_enforce_if(b1_ns)
            # EW -> y in [building_y, building_y + building_dy)
            model.add(conveyor_start_y[c] >= b1_y).only_enforce_if(~b1_ns)
            model.add(conveyor_start_y[c] < b1_y + b1_dy).only_enforce_if(~b1_ns)

            model.add(conveyor_start_y[c] == b1_y + b1_dy).only_enforce_if(b1_n)
            model.add(conveyor_start_y[c] == b1_y - 1).only_enforce_if(b1_s)
            model.add(conveyor_start_x[c] == b1_x + b1_dx).only_enforce_if(b1_e)
            model.add(conveyor_start_x[c] == b1_x - 1).only_enforce_if(b1_w)

        # No orientation constraint for end, only positional
        b2_n, b2_s, b2_e, b2_w, b2_ns = *building_nsew[b2], building_is_ns[b2]
        b2_x, b2_y = building_x[b2], building_y[b2]
        b2_dx, b2_dy = building_dx[b2], building_dy[b2]
        model.add(conveyor_end_x[c] >= b2_x).only_enforce_if(b2_ns)
        model.add(conveyor_end_x[c] < b2_x + b2_dx).only_enforce_if(b2_ns)
        model.add(conveyor_end_y[c] >= b2_y).only_enforce_if(~b2_ns)
        model.add(conveyor_end_y[c] < b2_y + b2_dy).only_enforce_if(~b2_ns)

        model.add(conveyor_end_y[c] == b2_y - 1).only_enforce_if(b2_n)
        model.add(conveyor_end_y[c] == b2_y + b2_dy).only_enforce_if(b2_s)
        model.add(conveyor_end_x[c] == b2_x - 1).only_enforce_if(b2_e)
        model.add(conveyor_end_x[c] == b2_x + b2_dx).only_enforce_if(b2_w)

    # TODO: redundant mask where conveyor_mask[*][0] is always true!
    conveyor_mask = [
        [model.new_bool_var("") for _ in range(turns)] for _ in conveyor_joins
    ]
    for c, mask in enumerate(conveyor_mask):
        for t, m in enumerate(mask):
            model.add(t < conveyor_turns[c]).only_enforce_if(m)
            model.add(t >= conveyor_turns[c]).only_enforce_if(~m)

    conveyor_is_ns = [
        [model.new_bool_var("") for _ in range(turns)] for _ in conveyor_joins
    ]
    for c, is_ns_arr in enumerate(conveyor_is_ns):
        for t, is_ns in enumerate(is_ns_arr):
            model.add(conveyor_orientation[c][t] < 2).only_enforce_if(is_ns)
            model.add(conveyor_orientation[c][t] >= 2).only_enforce_if(~is_ns)

    # 90 degree turns
    for c, (oris, is_ns_arr) in enumerate(zip(conveyor_orientation, conveyor_is_ns)):
        for ori2, ns1, ns2, m2 in zip(
            oris[1:], is_ns_arr, is_ns_arr[1:], conveyor_mask[c][1:]
        ):
            model.add(ns1 != ns2).only_enforce_if(m2)
            model.add(ori2 == 0).only_enforce_if(~m2)  # Symmetry break

    for lengths, mask in zip(conveyor_length, conveyor_mask):
        for t, (length, m) in enumerate(zip(lengths, mask)):
            if t == 0:
                model.add(length >= 1)
            else:
                model.add(length >= 2).only_enforce_if(m)
                model.add(length == 1).only_enforce_if(~m)

    conveyor_nsew = [
        [[model.new_bool_var("") for _ in range(4)] for _ in range(turns)]
        for _ in conveyor_joins
    ]
    for c, nsew_arr in enumerate(conveyor_nsew):
        for t, nsew in enumerate(nsew_arr):
            for i, v in enumerate(nsew):
                model.add(conveyor_orientation[c][t] == i).only_enforce_if(v)
                model.add(conveyor_orientation[c][t] != i).only_enforce_if(~v)

    conveyor_segment_end_x = [
        [model.new_int_var(0, gridrange_x, "") for _ in range(turns)]
        for _ in conveyor_joins
    ]
    conveyor_segment_end_y = [
        [model.new_int_var(0, gridrange_y, "") for _ in range(turns)]
        for _ in conveyor_joins
    ]

    for c, _ in enumerate(conveyor_joins):
        for t in range(turns):
            if t == 0:
                sx, sy = conveyor_start_x[c], conveyor_start_y[c]
            else:
                sx, sy = (
                    conveyor_segment_end_x[c][t - 1],
                    conveyor_segment_end_y[c][t - 1],
                )
            ex, ey = conveyor_segment_end_x[c][t], conveyor_segment_end_y[c][t]
            n, s, e, w = conveyor_nsew[c][t]
            is_ns = conveyor_is_ns[c][t]
            length = conveyor_length[c][t]
            model.add(sx == ex).only_enforce_if(is_ns)
            model.add(sy == ey).only_enforce_if(~is_ns)
            model.add(sy + length - 1 == ey).only_enforce_if(n)
            model.add(sy - length + 1 == ey).only_enforce_if(s)
            model.add(sx + length - 1 == ex).only_enforce_if(e)
            model.add(sx - length + 1 == ex).only_enforce_if(w)

    for c, _ in enumerate(conveyor_joins):
        model.add(conveyor_segment_end_x[c][-1] == conveyor_end_x[c])
        model.add(conveyor_segment_end_y[c][-1] == conveyor_end_y[c])

    # Vertical collisions
    conveyor_vert_interval_x = []
    conveyor_vert_interval_y = []
    for c, _ in enumerate(conveyor_joins):
        for t in range(turns):
            cond = model.new_bool_var("")
            model.add_min_equality(cond, conveyor_is_ns[c][t], conveyor_mask[c][t])
            conveyor_vert_interval_x.append(
                model.new_optional_fixed_size_interval_var(
                    conveyor_segment_end_x[c][t], 1, cond, ""
                )
            )
            start = model.new_int_var(0, gridrange_y, "")
            if t == 0:
                prev = conveyor_start_y[c]
            else:
                prev = conveyor_segment_end_y[c][t - 1]
            model.add_min_equality(start, prev, conveyor_segment_end_y[c][t])
            conveyor_vert_interval_y.append(
                model.new_optional_interval_var(
                    start,
                    conveyor_length[c][t],
                    model.new_int_var(0, gridrange_y + 1, ""),
                    cond,
                    "",
                )
            )

    model.add_no_overlap_2d(
        building_interval_x + conveyor_vert_interval_x,
        building_interval_y + conveyor_vert_interval_y,
    )

    # Horizontal collisions
    conveyor_horiz_interval_x = []
    conveyor_horiz_interval_y = []
    for c, _ in enumerate(conveyor_joins):
        for t in range(turns):
            cond = model.new_bool_var("")
            model.add_min_equality(cond, ~conveyor_is_ns[c][t], conveyor_mask[c][t])
            conveyor_horiz_interval_y.append(
                model.new_optional_fixed_size_interval_var(
                    conveyor_segment_end_y[c][t], 1, cond, ""
                )
            )
            start = model.new_int_var(0, gridrange_x, "")
            if t == 0:
                prev = conveyor_start_x[c]
            else:
                prev = conveyor_segment_end_x[c][t - 1]
            model.add_min_equality(start, prev, conveyor_segment_end_x[c][t])
            conveyor_horiz_interval_x.append(
                model.new_optional_interval_var(
                    start,
                    conveyor_length[c][t],
                    model.new_int_var(0, gridrange_x + 1, ""),
                    cond,
                    "",
                )
            )

    model.add_no_overlap_2d(
        building_interval_x + conveyor_horiz_interval_x,
        building_interval_y + conveyor_horiz_interval_y,
    )

    # Score constraints
    # objective = model.new_int_var(0, gridrange_x, "objective")
    # for x_end in building_interval_x_end.values():
    #     model.add(x_end - 1 <= objective)
    # for x_ends in conveyor_segment_end_x:
    #     for x_end in x_ends:
    #         model.add(x_end <= objective)

    objective = model.new_int_var(len(conveyor_joins), 300, "objective")
    model.add(
        objective
        == sum(sum(c) for c in conveyor_length) - len(conveyor_joins) * (turns - 1)
    )

    def abs_dist(v1: cp_model.IntVar, v2: cp_model.IntVar, ub: int) -> cp_model.IntVar:
        d = model.new_int_var(0, ub, "")
        model.add_abs_equality(d, v1 - v2)
        return d

    model.add(
        objective
        >= len(conveyor_joins)
        + sum(
            abs_dist(x1, x2, gridrange_x)
            for x1, x2 in zip(conveyor_start_x, conveyor_end_x)
        )
        + sum(
            abs_dist(y1, y2, gridrange_y)
            for y1, y2 in zip(conveyor_start_y, conveyor_end_y)
        )
    )

    model.minimize(objective)

    model.add_decision_strategy(
        [
            *building_x.values(),
            *building_y.values(),
            # *building_orientation.values(),
            # *building_dx.values(),
            # *building_dy.values(),
        ],
        cp_model.CHOOSE_FIRST,
        cp_model.SELECT_MEDIAN_VALUE,
    )
    # model.add_decision_strategy(
    #     [max_x],
    #     cp_model.CHOOSE_FIRST,
    #     cp_model.SELECT_MAX_VALUE,
    # )

    variables = Variables(
        building_x=building_x,
        building_y=building_y,
        building_dx=building_dx,
        building_dy=building_dy,
        building_orientation=building_orientation,
        conveyor_start_x=conveyor_start_x,
        conveyor_start_y=conveyor_start_y,
        conveyor_segment_end_x=conveyor_segment_end_x,
        conveyor_segment_end_y=conveyor_segment_end_y,
        conveyor_orientation=conveyor_orientation,
        conveyor_mask=conveyor_mask,
        pickup_perm=pickup_perm,
        objective=objective,
    )

    # Apply hinting
    def rec_map(varib, hint):
        if hint is None:
            return
        elif isinstance(hint, (tuple, list)):
            for v, h in zip(varib, hint, strict=True):
                rec_map(v, h)
        elif isinstance(hint, dict):
            for h in hint:
                rec_map(varib[h], hint[h])
        else:
            # model.add_hint(varib, hint)
            model.add(varib == hint)

    if data.hints is not None:
        rec_map(variables, data.hints(variables.map(lambda _: None)))

    return model, variables


def solve(data: Data, *, workers: int = 1, seconds: int = 30):
    model, variables = create_model(data)

    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = workers
    solver.parameters.max_time_in_seconds = seconds

    start_time = time.time()

    def rec_map(f, x):
        if isinstance(x, dict):
            return {k: rec_map(f, v) for k, v in x.items()}
        elif isinstance(x, (tuple, list)):
            return type(x)(rec_map(f, v) for v in x)
        else:
            return f(x)

    def values(f):
        return Variables[int](*[rec_map(f, x) for x in variables])

    class Printer(cp_model.CpSolverSolutionCallback):
        def OnSolutionCallback(self):
            elapsed = int(time.time() - start_time)
            print(f"[{elapsed} sec] Objective: {self.value(variables.objective)}")
            print(values(self.value), flush=True)

    status = solver.solve(model, Printer())

    print(f"Status = {solver.status_name(status)}")

    return model, variables, values(solver.value)


if __name__ == "__main__":
    solve(
        Data(
            gridrange_x=5,
            gridrange_y=6,
            buildings=[Building("deez", 3, 3), Building("nuts", 3, 3)],
            conveyor_joins=[("deez", "nuts")],
            pickups=[],
        )
    )
