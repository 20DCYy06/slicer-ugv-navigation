"""WGS84 local ENU with one durable origin per evidence run.

An ENU trace describes relative antenna motion; it is not absolute accuracy.
No origin is inferred from an invalid/old position or reset after reconnection.
"""
import json
import math
import os


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def _position(latitude, longitude, altitude):
    values = (latitude, longitude, altitude)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError("WGS84 latitude, longitude and ellipsoid altitude must be finite numbers")
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("WGS84 latitude/longitude outside their bounds")
    return tuple(float(v) for v in values)


def geodetic_to_ecef(latitude, longitude, altitude):
    """Return ECEF metres; altitude is WGS84 ellipsoid height, not MSL."""
    latitude, longitude, altitude = _position(latitude, longitude, altitude)
    lat, lon = math.radians(latitude), math.radians(longitude)
    slat, clat, slon, clon = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * slat * slat)
    return ((normal + altitude) * clat * clon,
            (normal + altitude) * clat * slon,
            (normal * (1.0 - WGS84_E2) + altitude) * slat)


def to_enu(latitude, longitude, altitude, origin):
    """Convert WGS84 antenna coordinates into a fixed East, North, Up frame."""
    xyz = geodetic_to_ecef(latitude, longitude, altitude)
    origin_xyz = geodetic_to_ecef(origin["latitude"], origin["longitude"], origin["altitude_ellipsoid_m"])
    dx, dy, dz = (a - b for a, b in zip(xyz, origin_xyz))
    lat, lon = math.radians(origin["latitude"]), math.radians(origin["longitude"])
    slat, clat, slon, clon = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return (-slon * dx + clon * dy,
            -slat * clon * dx - slat * slon * dy + clat * dz,
            clat * clon * dx + clat * slon * dy + slat * dz)


class FixedOriginTrack:
    """Persist the first explicitly qualified sample; caller owns quality policy.

    Existing origin.json is loaded and validated, never replaced. A missing
    finite ellipsoid height is rejected rather than silently setting height=0.
    The evidence tool supplies fresh_rtk only after raw/ROS/diagnostic checks.
    """
    def __init__(self, output_dir):
        self.origin_path = os.path.join(os.fspath(output_dir), "origin.json")
        self.origin = None
        if os.path.exists(self.origin_path):
            with open(self.origin_path, encoding="utf-8") as stream:
                self.origin = self._validate(json.load(stream))

    @staticmethod
    def _validate(origin):
        if origin.get("version") != 1 or origin.get("frame") != "WGS84 local ENU":
            raise ValueError("unsupported origin format; preserve and inspect the original run")
        _position(origin["latitude"], origin["longitude"], origin["altitude_ellipsoid_m"])
        return origin

    def add(self, latitude, longitude, altitude, fresh_rtk, reason="", metadata=None):
        if fresh_rtk is not True:
            return {"accepted": False, "reason": reason or "fresh_rtk_not_proven"}
        try:
            latitude, longitude, altitude = _position(latitude, longitude, altitude)
        except ValueError as error:
            return {"accepted": False, "reason": str(error)}
        if self.origin is None:
            origin = {"version": 1, "frame": "WGS84 local ENU", "latitude": latitude,
                      "longitude": longitude, "altitude_ellipsoid_m": altitude,
                      "selection": "first raw/ROS/diagnostic qualified fresh RTK sample",
                      "accuracy_claim": "relative antenna coordinates only; absolute accuracy unverified",
                      "metadata": metadata or {}}
            # Exclusive creation is deliberate: concurrent analysts may not reset
            # the reference. A concurrent winner is loaded and used unchanged.
            try:
                with open(self.origin_path, "x", encoding="utf-8") as stream:
                    json.dump(origin, stream, ensure_ascii=False, indent=2, allow_nan=False)
                    stream.write("\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                self.origin = origin
            except FileExistsError:
                with open(self.origin_path, encoding="utf-8") as stream:
                    self.origin = self._validate(json.load(stream))
        east, north, up = to_enu(latitude, longitude, altitude, self.origin)
        return {"accepted": True, "reason": "", "east_m": east, "north_m": north, "up_m": up}
