"""Aircraft performance profile database.

Backward-compatibility alias re-exporting adsb.synthesis.performance.
"""

from __future__ import annotations

from adsb.synthesis.performance import (
    AircraftPerformanceProfile,
    WakeVortexCategory,
    get_aircraft_profile,
    has_aircraft_profile,
    list_aircraft_profiles,
    register_aircraft_profile,
)

__all__ = [
    "AircraftPerformanceProfile",
    "WakeVortexCategory",
    "get_aircraft_profile",
    "has_aircraft_profile",
    "list_aircraft_profiles",
    "register_aircraft_profile",
]
