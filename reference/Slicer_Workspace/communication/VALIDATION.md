# UGV deployment acceptance — 2026-10-22

## Latest update: low-bandwidth telemetry (supersedes earlier unlimited-rate policy)

External odom is now capped at 5 Hz, AMCL pose at 1 Hz; ready and mission state
publish only on change and remain latched. Full original message types, fields,
source timestamps and frames are preserved. Internal navigation is unchanged.
The UAV receiving example no longer contains `/ugv/test`; no UAV files or
processes were accessed. Incoming `/uav/test` on UGV remains unchanged.

Isolated Noetic Docker check passed: 63 simulated odom input messages became
16 output messages (45990 -> 11680 serialized payload bytes); pose 63 -> 4;
unchanged ready/state 63 -> 1 each. Byte-preservation, rate caps, state changes
and late-subscriber latch checks passed. These are private-master synthetic
measurements, not measured live-radio throughput. Navigation was not started
for this task; only `ugv-state-relay` was restarted.

Backup: `/home/potato/Backups/Slicer/2026-10-22/Slicer_telemetry_rate_backup_20261022_leWFDk/`.
UGV currently uses synchronized systemd-timesyncd, not active Chrony. Relative
UAV/UGV skew and a genuinely shared physical map remain to be accepted manually:
see TIME_AND_FRAMES.md. No clock or map transform was changed.

## Latest update: lowercase interface (supersedes names in the initial report below)

- All four exported topics now use `/ugv/`: odom, pose, chassis_ready,
  mission/state. Relay node: `/ugv/communication/state_relay`.
- Live master audit: NO_UPPERCASE_EXPORTS_PASS; all four lowercase topics
  published by the new relay. Advertised URI uses `192.168.10.33`.
- UGV whitelist: `/uav/test` plus those four lowercase state topics.
  The inbound UAV test topic is deliberately unchanged.
- Existing isolated communication check passed after the rename. All four
  serialized message comparisons passed, including headers; local measured
  delay min/mean/max was 0.578 / 0.693 / 0.891 ms (not radio latency).
- 913 protected file checksums still passed. No navigation/control process
  was started, and no goals or movement commands were sent for this update.
- Communication configuration backup:
  `/home/potato/Backups/Slicer/2026-10-22/Slicer_lowercase_topics_backup_20261022_rIe1jx/`.
- Old relay container was stopped and retained as
  `ugv-state-relay-before-lowercase-20261022`; current relay and sync are running.
- UAV has not been modified; its manual handoff now lists `/ugv/test` and the
  four lowercase exports, following the user's revised receiving whitelist.

The remaining sections are the initial deployment's historical results, not
the current spelling of interface names. Current instructions: README.md and
UAV_HANDOFF.md.

## Completed

- Full pre-change workspace backup verified:
  `/home/potato/Backups/Slicer/2026-10-22/Slicer_communication_backup_20261022_GvA5W8/`.
- `communication/run.sh check`: COMMUNICATION_CHECK_PASS; all four relay
  message byte streams identical including Header.seq/stamp/frame_id.
  Isolated local delivery min/mean/max: 0.731 / 0.912 / 1.110 ms.
  This is not a measurement of radio latency or a zero-latency guarantee.
- `$NAV check`: NAVIGATION_CHECK_PASS, existing navigation/control/ROS tests
  passed in network-isolated Noetic Docker.
- All 913 protected source/config/firmware/driver file hashes unchanged.
  The NAV launcher intentionally differs only by a four-line check-mode
  loopback master/IP override. Production mapping/nav addresses unchanged.
- Original `config/fkie_ugv_sync.launch` path and root Dockerfile retained.
- UGV sync loaded `/uav/test` plus all four `/UGV/...` topics. Shared master and
  discovery were NOT restarted. Only `ugv-sync` was reloaded.
- Relay advertises its node at `http://192.168.10.33:45799/` in this run
  (port is dynamic). No global namespace is applied to navigation.
- Actual navigation launched using existing `my_room.yaml`, with permission,
  no goals/patrol. `/odom` remained published by `/slicer_ekf`, frames
  `odom -> base_link`; `/move_base`, AMCL and original driver remained present.
- Eight-second live observation: source/export odom 156/156 messages;
  ready 155/155; AMCL 4/3 (subscriber setup boundary, not a rate assertion).
  Matching captured input/output samples were byte-identical.
- Final separate read-only observation: 41 `/cmd_vel` samples all zero;
  no active patrol node. `chassis_ready=True` was observed; software readiness
  is NOT physical proof of Motor OFF. No motion test was performed.
- Temporary navigation container stopped after acceptance. Communication
  containers remain running; unrelated industrial-cup container untouched.
- No old files deleted, moved or cleaned in this turn, per latest scope.

## Pending / intentionally not claimed

- UAV SSH/files/processes were not accessed or changed. Its receiving
  whitelist adaptation and reload are manual: see `UAV_HANDOFF.md`.
- Actual receiving data and radio latency on UAV are not accepted yet.
- `/UGV/mission/state` is advertised but has no samples without a real patrol
  publisher; no mission was started or fake mission state injected.
- `/UGV/pose` requires an actual AMCL source. Stopping navigation removes
  live odom/AMCL producers, not the four relay output advertisements.

## Changed / added files

Existing files changed:

1. `config/fkie_ugv_sync.launch`: load centralized communication whitelist.
2. `catkin_ws/src/slicer_navigation/scripts/run_navigation.sh`: check-only IP fix.

New files are confined to `communication/`: network/relay/sync YAML settings,
transparent relay, native Docker manager, isolated verification, launch wrapper,
Dockerfile copy and README/UAV handoff/this acceptance report. All existing
Docker configurations, hardware source, calibration, maps and NAV tests remain.
