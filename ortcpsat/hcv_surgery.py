from model import Variables
from model_data import HC_VALLEY_BATTERY_REDUCED as HC_VALLEY_BATTERY_REDUCED_MOD, HC_VALLEY_BATTERY

ADDON = ["SHR1", "SHR2", "SHR3", "SHR4", "SHR5", "SHR6", "RU1", "RU2", "RU3", "RU4"]

HC_VALLEY_BATTERY_REDUCED_MOD = HC_VALLEY_BATTERY_REDUCED_MOD._replace(
    buildings=HC_VALLEY_BATTERY.buildings,
    conveyor_joins=HC_VALLEY_BATTERY.conveyor_joins[:10]
                    + [(a, b) for a, (_, b) in zip(ADDON, HC_VALLEY_BATTERY_REDUCED_MOD.conveyor_joins[:10])]
                    + HC_VALLEY_BATTERY_REDUCED_MOD.conveyor_joins[10:],
)


def surgery(varibs: Variables) -> Variables:
    # Shift everything over
    varibs = varibs._replace(
        building_x={k: v + 4 for k, v in varibs.building_x.items()},
        conveyor_start_x=[v + 4 for v in varibs.conveyor_start_x],
        conveyor_segment_end_x=[[v + 4 for v in a] for a in varibs.conveyor_segment_end_x],
    )

    # Add on the fixed parts
    varibs = varibs._replace(
        building_x={**{a: 1 for a in ADDON}, **varibs.building_x},
        building_y={**{a: varibs.pickup_perm[i] * 3 for i, a in enumerate(ADDON)}, **varibs.building_y},
        building_dx={**{a: 3 for a in ADDON}, **varibs.building_dx},
        building_dy={**{a: 3 for a in ADDON}, **varibs.building_dy},
        building_orientation={**{a: 2 for a in ADDON}, **varibs.building_orientation},
        conveyor_start_x=[0 for _ in ADDON] + varibs.conveyor_start_x,
        conveyor_start_y=[i * 3 + 1 for i, _ in enumerate(ADDON)] + varibs.conveyor_start_y,
        conveyor_segment_end_x=[[0] * 3 for _ in ADDON] + varibs.conveyor_segment_end_x,
        conveyor_segment_end_y=[[i * 3 + 1] * 3 for i, _ in enumerate(ADDON)] + varibs.conveyor_segment_end_y,
    )

    return varibs
