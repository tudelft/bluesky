#!/usr/bin/env python3
"""
Test static SSD visualization with the live monitor.
No MQTT required - directly visualizes specific aircraft configurations.
"""

from ssd_live_monitor import SSDLiveMonitor

def test_head_on_scenario():
    """Test head-on conflict scenario."""
    monitor = SSDLiveMonitor()
    
    # Head-on scenario: two aircraft approaching each other
    # Ownship at origin, heading north at 15 m/s
    ownship = {
        'lat': 52.0,      # degrees
        'lon': 4.0,       # degrees
        'alt': 100.0,     # meters
        'vn': 15.0,       # m/s north
        've': 0.0,        # m/s east
        'vd': 0.0         # m/s down
    }
    
    # Intruder 200m north, heading south at 15 m/s (head-on)
    # 200m gives sufficient separation (4x the 50m requirement)
    intruder = {
        'lat': 52.0018,   # ~200m north (0.0018 degrees)
        'lon': 4.0,
        'alt': 100.0,
        'vn': -15.0,      # m/s south (toward ownship)
        've': 0.0,
        'vd': 0.0
    }
    
    print("Test Case: Head-on conflict")
    print(f"  Ownship: heading north at 15 m/s")
    print(f"  Intruder: 200m north, heading south at 15 m/s")
    monitor.visualize_static(ownship, intruder, "OWN1", "INT1")


def test_crossing_scenario():
    """Test crossing conflict scenario."""
    monitor = SSDLiveMonitor()
    
    # Ownship heading north at 15 m/s
    ownship = {
        'lat': 52.0,
        'lon': 4.0,
        'alt': 100.0,
        'vn': 15.0,       # m/s north
        've': 0.0,
        'vd': 0.0
    }
    
    # Intruder 100m northeast, heading west at 15 m/s (crossing from right)
    intruder = {
        'lat': 52.0009,   # ~100m north
        'lon': 4.00127,   # ~100m east
        'alt': 100.0,
        'vn': 0.0,
        've': -15.0,      # m/s west (crossing path)
        'vd': 0.0
    }
    
    print("Test Case: Crossing conflict (intruder from right)")
    print(f"  Ownship: heading north at 15 m/s")
    print(f"  Intruder: 100m NE, heading west at 15 m/s")
    monitor.visualize_static(ownship, intruder, "OWN2", "INT2")


def test_overtaking_scenario():
    """Test overtaking scenario."""
    monitor = SSDLiveMonitor()
    
    # Slow ownship heading north at 10 m/s
    ownship = {
        'lat': 52.0,
        'lon': 4.0,
        'alt': 100.0,
        'vn': 10.0,       # m/s north (slow)
        've': 0.0,
        'vd': 0.0
    }
    
    # Fast intruder 80m behind, heading north at 20 m/s (overtaking)
    intruder = {
        'lat': 51.9993,   # ~80m south
        'lon': 4.0,
        'alt': 100.0,
        'vn': 20.0,       # m/s north (faster, catching up)
        've': 0.0,
        'vd': 0.0
    }
    
    print("Test Case: Overtaking scenario")
    print(f"  Ownship: heading north at 10 m/s")
    print(f"  Intruder: 80m behind, heading north at 20 m/s")
    monitor.visualize_static(ownship, intruder, "OWN3", "INT3")


if __name__ == "__main__":
    import sys
    
    scenarios = {
        '1': ('Head-on', test_head_on_scenario),
        '2': ('Crossing', test_crossing_scenario),
        '3': ('Overtaking', test_overtaking_scenario)
    }
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        print("\nAvailable test scenarios:")
        for key, (name, _) in scenarios.items():
            print(f"  {key}: {name}")
        print("\nUsage: python test_ssd_static.py [1|2|3]")
        print("       or run without argument to see this menu\n")
        choice = input("Select scenario (1-3): ").strip()
    
    if choice in scenarios:
        name, test_func = scenarios[choice]
        print(f"\n{'='*60}")
        test_func()
    else:
        print(f"Invalid choice: {choice}")
        sys.exit(1)
