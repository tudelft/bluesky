#!/usr/bin/env python3
"""
DAA Validation Scenario Generator

Comprehensive test scenarios for validating SSD_Drone conflict detection
and resolution behavior. Focuses on edge cases and various conflict geometries.
"""

import math
from typing import Dict, Any, List, Tuple


def create_scenario_1_head_on_basic():
    """
    Scenario 1: Basic head-on conflict
    - Simple 180° head-on encounter
    - Equal speeds
    - Same altitude
    - Initial separation: ~150m
    - Tests: Basic velocity obstacle construction
    """
    return {
        "name": "1. Head-On Basic",
        "description": "Simple head-on conflict, equal speeds, 150m separation",
        "aircraft": [
            {
                "id": "OWN001",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,    # North
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT001",
                "lat": 52.00405,   # ~150m north
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 180.0,   # South (head-on)
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN001",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_2_head_on_speed_differential():
    """
    Scenario 2: Head-on with speed differential
    - Ownship faster than intruder
    - Initial separation: ~180m
    - Tests: Velocity obstacle with asymmetric velocities
    """
    return {
        "name": "2. Head-On Fast vs Slow",
        "description": "Head-on conflict with ownship 40% faster, 180m separation",
        "aircraft": [
            {
                "id": "OWN002",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,
                "gs": 14.0,    # Faster
                "vs": 0.0,
            },
            {
                "id": "INT002",
                "lat": 52.00486,   # ~180m north
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 180.0,
                "gs": 12.0,    # Slower
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN002",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_3_crossing_90deg():
    """
    Scenario 3: 90° crossing conflict
    - Initial separation: ~400m to crossing point
    - Tests: ARV construction for crossing geometry
    """
    return {
        "name": "3. 90° Crossing",
        "description": "Perpendicular crossing paths with proper separation",
        "aircraft": [
            {
                "id": "OWN003",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 90.0,   # East
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT003",
                "lat": 52.00225,    # 250m north
                "lon": 4.00225,     # 250m east
                "alt": 1000.0,
                "trk": 180.0,       # South
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN003",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_4_crossing_45deg():
    """
    Scenario 4: 45° crossing angle
    - Initial separation: ~400m
    - Tests: ARV for oblique crossing
    """
    return {
        "name": "4. 45° Crossing",
        "description": "Oblique crossing at 45 degrees with proper separation",
        "aircraft": [
            {
                "id": "OWN004",
                "lat": 52.0,
                "lon": 4.00225,
                "alt": 1000.0,
                "trk": 337.5,    # North west
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT004",
                "lat": 52.0,    # ~400m NE
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 22.5,       # Northeast (opposite)
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN004",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_5_overtaking():
    """
    Scenario 5: Ownship overtaking
    - Initial separation: ~100m (ownship catching up)
    - Tests: Detection when both moving same direction
    """
    return {
        "name": "5. Overtaking",
        "description": "Ownship overtaking slower traffic, 100m ahead",
        "aircraft": [
            {
                "id": "OWN005",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,     # North
                "gs": 16.0,     # Fast
                "vs": 0.0,
            },
            {
                "id": "INT005",
                "lat": 52.0009,     # ~100m ahead
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,         # Same direction
                "gs": 11.0,         # Slow
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN005",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_6_being_overtaken():
    """
    Scenario 6: Being overtaken
    - Initial separation: ~100m behind (fast approaching)
    - Tests: Detection of threat from rear
    """
    return {
        "name": "6. Being Overtaken",
        "description": "Faster traffic overtaking from behind, 100m back",
        "aircraft": [
            {
                "id": "OWN006",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,
                "gs": 11.0,     # Slower
                "vs": 0.0,
            },
            {
                "id": "INT006",
                "lat": 51.9991,     # ~100m behind
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,         # Same direction
                "gs": 16.0,         # Fast (catching up)
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN006",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_7_multiple_intruders():
    """All within 150m separation
    - Tests: Multi-conflict resolution
    """
    return {
        "name": "7. Multiple Intruders",
        "description": "Three simultaneous conflicts, all <150m away",
        "aircraft": [
            {
                "id": "OWN007",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT007A",
                "lat": 52.00405,    # ~150m north
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 180.0,       # Coming from north
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT007B",
                "lat": 52.0,
                "lon": 4.00163,     # ~130m east
                "alt": 1000.0,
                "trk": 270.0,       # Coming from east
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT007C",
                "lat": 51.99865,    # ~150m south
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,         # Coming from south
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN007",
            "vertices": [
                (51.95, 3.95),
                (51.95, 4.05),
                (52.05, 4.05),
                (52.05, 3.95),
            ]
        }
    }


def create_scenario_8_near_geofence_edge():
    """
    Scenario 8: Near geofence edge
    - Initial separation: ~140m
    - Tests: Boundary awareness in resolution
    """
    return {
        "name": "8. Near Geofence Edge",
        "description": "Conflict near geofence boundary, 140m separation",
        "aircraft": [
            {
                "id": "OWN009",
                "lat": 52.0045,     # Near north edge of geofence
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 90.0,        # East
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT009",
                "lat": 52.0045,
                "lon": 4.00139,     # ~140m east (head-on)
                "alt": 1000.0,
                "trk": 270.0,       # West (head-on)
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN009",
            "vertices": [
                (51.995, 3.995),
                (51.995, 4.005),
                (52.005, 4.005),    # North boundary - ownship near this
                (52.005, 3.995),
            ]
        }
    }


def create_scenario_9_vertical_separation():
    """
    Scenario 9: Vertical conflict
    - Close horizontal position with converging altitudes
    - Climbing/descending scenarios
    - Initial separation: ~150m horizontal, converging vertical
    - Tests: Vertical separation logic
    """
    return {
        "name": "9. Vertical Separation",
        "description": "Conflicting with vertical component, 150m horizontal",
        "aircraft": [
            {
                "id": "OWN009",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,
                "gs": 12.0,
                "vs": 5.0,          # Climbing at 5 m/s
            },
            {
                "id": "INT009",
                "lat": 52.00405,    # ~150m north
                "lon": 4.0,
                "alt": 1080.0,      # Only 80m higher
                "trk": 180.0,
                "gs": 12.0,
                "vs": -5.0,         # Descending at 5 m/s
            }
        ],
        "geofence": {
            "ac_id": "OWN009",
            "vertices": [
                (51.995, 3.995),
                (51.995, 4.005),
                (52.005, 4.005),
                (52.005, 3.995),
            ]
        }
    }


def create_scenario_10_converging_angles():
    """
    Scenario 10: Converging at shallow angle
    - Paths converging at ~30°
    - Initial separation: ~450m
    - Tests: Detection of gradual convergence
    """
    return {
        "name": "10. Shallow Convergence",
        "description": "Paths converging at 30 degree angle, 450m apart",
        "aircraft": [
            {
                "id": "OWN010",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 15.0,    # NNE
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT010",
                "lat": 52.0,
                "lon": 4.00405,     # ~450m east
                "alt": 1000.0,
                "trk": 345.0,       # NNW (converging at 30°)
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN010",
            "vertices": [
                (51.995, 3.995),
                (51.995, 4.005),
                (52.005, 4.005),
                (52.005, 3.995),
            ]
        }
    }


def create_scenario_11_parallel_offset():
    """
    Scenario 11: Parallel paths with small offset
    - Nearly parallel but slightly converging
    - Initial separation: ~250m lateral offset
    - Tests: Minimum safe separation
    """
    return {
        "name": "11. Parallel Near-Miss",
        "description": "Nearly parallel paths, 250m lateral offset, converging",
        "aircraft": [
            {
                "id": "OWN011",
                "lat": 52.0,
                "lon": 4.0,
                "alt": 1000.0,
                "trk": 0.0,
                "gs": 12.0,
                "vs": 0.0,
            },
            {
                "id": "INT011",
                "lat": 52.0,
                "lon": 4.00225,     # ~250m lateral offset
                "alt": 1000.0,
                "trk": 355.0,       # Slightly converging (5° angle)
                "gs": 12.0,
                "vs": 0.0,
            }
        ],
        "geofence": {
            "ac_id": "OWN012",
            "vertices": [
                (51.995, 3.995),
                (51.995, 4.005),
                (52.005, 4.005),
                (52.005, 3.995),
            ]
        }
    }


# Scenario catalog
ALL_SCENARIOS = [
    create_scenario_1_head_on_basic,
    create_scenario_2_head_on_speed_differential,
    create_scenario_3_crossing_90deg,
    create_scenario_4_crossing_45deg,
    create_scenario_5_overtaking,
    create_scenario_6_being_overtaken,
    create_scenario_7_multiple_intruders,
    create_scenario_8_near_geofence_edge,
    create_scenario_9_vertical_separation,
    create_scenario_10_converging_angles,
    create_scenario_11_parallel_offset,
]


def get_scenario_by_number(n: int) -> Dict[str, Any]:
    """Get scenario by number (1-11)"""
    if 1 <= n <= len(ALL_SCENARIOS):
        return ALL_SCENARIOS[n - 1]()
    raise ValueError(f"Scenario {n} not found. Valid range: 1-{len(ALL_SCENARIOS)}")


def get_all_scenarios() -> List[Dict[str, Any]]:
    """Get all scenarios"""
    return [func() for func in ALL_SCENARIOS]


def get_scenario_summary() -> str:
    """Get formatted summary of all scenarios"""
    lines = ["Available DAA Validation Scenarios", "=" * 80]
    for i, func in enumerate(ALL_SCENARIOS, 1):
        scenario = func()
        num_aircraft = len(scenario['aircraft'])
        has_geofence = "geofence" in scenario
        lines.append(f"{i:2d}. {scenario['name']:<25} "
                    f"({num_aircraft} aircraft, "
                    f"{'with' if has_geofence else 'no'} geofence)")
        lines.append(f"    {scenario['description']}")
    return "\n".join(lines)


if __name__ == "__main__":
    print(get_scenario_summary())
