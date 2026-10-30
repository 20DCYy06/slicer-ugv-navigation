"""Conservative live-correction policy, independent of ROS message classes."""
import math


def apply_rtk_policy(snapshot, ntrip_state, serial_write_state, max_age=5.0):
    """Mutate one newly computed model snapshot; never rewrite raw quality.

    Ordinary measured GNSS may be useful without RTK. An augmented receiver
    solution requires recent successful complete RTCM writes in this serial
    session when this node is configured to supply its live corrections.
    """
    diag = snapshot["diagnostics"]
    stats = ntrip_state or {"enabled": False}
    diag.update({"ntrip_" + key: value for key, value in stats.items()})
    diag.update(serial_write_state or {"rtcm_serial_write_age_s": None,
                                      "rtcm_serial_write_session": ""})
    age = diag["rtcm_serial_write_age_s"]
    session_ok = bool(diag.get("session")) and diag.get("rtcm_serial_write_session") == diag["session"]
    write_ok = (isinstance(age, (int, float)) and not isinstance(age, bool)
                and math.isfinite(age) and 0 <= age <= max_age and session_ok)
    enabled = stats.get("enabled") is True
    chain_reason = ""
    if not enabled:
        chain_reason = "live_corrections_not_enabled"
    elif stats.get("fatal"):
        chain_reason = "rtcm_forwarding_fatal"
    elif not write_ok:
        chain_reason = "rtcm_serial_write_missing_or_stale"
    reason = diag["invalid_reason"]
    if not reason and diag["gga_quality"] != 4:
        reason = "receiver_not_rtk_fixed"
    if not reason:
        reason = chain_reason
    diag["fresh_rtk_usable"] = not reason
    diag["fresh_rtk_reason"] = reason
    if enabled and diag["gga_quality"] in (2, 4, 5) and snapshot["fix"]["valid"] and chain_reason:
        snapshot["fix"].update(valid=False, status=-1, latitude=math.nan, longitude=math.nan, altitude=math.nan)
        diag.update(fix_valid=False, invalid_reason=chain_reason)
        snapshot.update(level=2, message=chain_reason)
    return snapshot
