# `dev-alx` to `sunny/dev` port report

Port date: 2026-07-25

## Result

The original local `dev` was renamed to `dev-alx` at `ba2e53ed0f`. A new local
`dev` was created from upstream `sunny/dev` at `543ade4eb9` and continues to
track `sunny/dev`.

The local feature work was manually ported and adapted as eleven focused
commits. The functional boundaries follow the useful `dev-alx` commits instead
of flattening the work into one large integration commit.

## Ported commit series

| New `dev` commit | Ported intent from `dev-alx` |
|---|---|
| `e842e174f0` Enable radar tracks on comma four | `e22824956d`, the earlier radar UI/toggle experiments, `1d06b73f7e`, and `54420c0268` |
| `a91844ca58` Use MRR35 radar tracks for Niro EV HDA2 | `111888f569` plus the opendbc content formerly selected by `b71d04ffbd` |
| `a7f12dcf05` Show radar tracks on comma four for Niro EV HDA2 | `5bd35e4081` and the Niro-specific automatic setup |
| `12e90a23d6` Sync stock ACC speed for curve and speed limit targets | `df6cc9c161`, `c0ef29c0ae`, and the relevant ICBM integration from `2c0a5c5a4d` |
| `a31afa56bf` Anticipate stock ACC curve slowdown from vision | `c8ac4b6532`, `59f1dab8cb`, and `ca339f092d` vision behavior |
| `a450e9f92a` Anticipate stock ACC curves from maps | `69c90fda70`, `59f1dab8cb`, `ca339f092d`, `6d2ec4650e`, and `439bfb5d89` |
| `f1370ff97a` Feed anticipated map speed limits to ICBM | Remaining map-speed-limit anticipation from `2c0a5c5a4d` |
| `2b104bc20c` Show cellular network status on Mici home | `5abba70c80` |
| `4e936e0c0b` Add network mode selector | `84cc4807b4`, adapted to the new modem architecture |
| `ab1b1040ae` Add radar tracks to sunnylink settings | The old toggle metadata intent, represented on the current schema |
| `a6f04a2b5e` Rebuild packaged source after port | Initial attempt to invalidate upstream prebuilts; superseded by the packaged-artifact correction described below |

Merge-only commits and version/release merges from `dev-alx` were not recreated.

## What was ported

### Radar tracks and Kia Niro EV HDA2

- Added the `RadarTracks` parameter and Mici toggle.
- Subscribed the UI to `liveTracks`.
- Added radar-track drawing to both standard and Mici model renderers.
- Draws the tracked lead in green and other tracks in pink.
- Added MRR35 radar flagging, CAN parsing, DBC generation, radar-bus selection,
  and availability detection.
- Detects the Niro EV HDA2 configuration and enables its associated radar,
  intelligent cruise, speed-limit, vision-curve, and map-curve features.
- Added the radar-track option to the sunnylink visuals page and regenerated
  `settings_ui.json`.

### Stock ACC, ICBM, vision curves, and map curves

- Intelligent cruise button management now considers the resolved speed limit,
  anticipated map speed limits, and vision/map curve targets.
- Preserved speed synchronization while avoiding interference with manual
  cruise-button input.
- Added earlier vision-curve slowdown, target holding, release behavior, and
  stock-ACC response/deceleration buffers.
- Added earlier map-curve anticipation, dropout holding, release tuning, and an
  outside-lane target-speed allowance.
- Uses OSM country traffic side when available, with driver-monitoring
  right-hand-drive state as a fallback.
- Added focused ICBM, vision-controller, and map-controller tests.

### Cellular UI and network mode

- Shows dynamic cellular/operator state on the Mici home screen.
- Added `NetworkMode` with Wi-Fi, cellular, and offline states.
- Wi-Fi mode controls NetworkManager wireless state.
- Cellular mode controls the upstream PPP modem state machine.
- Offline/Wi-Fi modes stop PPP; cellular mode resumes normal modem operation.

## Differences required for `sunny/dev`

### Repository and data-model changes

- All old root-level source paths were translated to the nested `openpilot/`
  layout.
- MRR35 changes were applied directly to vendored `opendbc_repo/`. The old
  `.gitmodules` pointer to an alxwest opendbc fork is not applicable.
- Radar rendering uses `leadOne.present`, matching the new radar schema, instead
  of the old `leadOne.status`.
- The sunnylink toggle was added to
  `settings_ui_src/pages/visuals.yaml` and compiled into JSON. The obsolete
  `params_metadata.json` path was not restored.

### Longitudinal integration changes

- Imports, subscriptions, and controller calls were adapted to current cereal,
  planner, driver-monitoring, and smart-cruise interfaces.
- `driverMonitoringState` is now supplied to the map controller for the
  traffic-side fallback.
- The older local `plannerd` map-health exclusion was not duplicated because
  current upstream already excludes `liveMapDataSP` there. `selfdrived` still
  needed its own subscription/health handling for ICBM input.

### Cellular and eSIM changes

- The original selector toggled NetworkManager's `WwanEnabled`. Current
  `sunny/dev` manages cellular through a dedicated PPP modem state machine, so
  the port adds a disabled modem state and mode-aware PPP stop/resume behavior.
- Upstream already falls back to modem state when no primary network route is
  available. The `660cadae31` behavior is therefore already present and was not
  applied twice.
- The old eSIM `reboot_modem()` path no longer exists. Current upstream
  serializes modem work and detects ICCID changes, so the obsolete reboot call
  was not recreated.

### Packaged-build handling

`sunny/dev` includes a `prebuilt` marker because the deployment tree omits
`SConstruct` and the SCons source-build description. Removing the marker causes
`build.py` to fail with `No SConstruct file found`; this was confirmed during
device deployment.

The final port therefore restores the marker and carries a rebuilt aarch64
`openpilot/common/params_pyx.so` containing the new `RadarTracks` and
`NetworkMode` keys. Python source changes continue to run directly from the
checkout, and the MRR35 DBC is generated dynamically by the current opendbc
loader.

## Validation

Completed checks:

- Git whitespace/error check over the complete port.
- Python bytecode compilation for every changed Python module.
- Sunnylink YAML-to-JSON compilation and generated-file consistency check.
- On-device native params rebuild and smoke test for `RadarTracks`,
  `NetworkMode`, and an existing key.
- Static inspection of the branch ancestry, changed-file set, and commit
  boundaries.

The new focused unit tests were added, but could not be executed in this host
environment. The checkout contains Linux/aarch64 packaged native modules while
the host is macOS/arm64, and the system Python is 3.14 rather than the project's
3.12 environment; required modules such as pytest, capnp, numpy, pyray, and
jeepney are unavailable here.

Recommended device/CI follow-up:

1. Exercise Niro EV HDA2 radar parsing and verify lead/track overlay placement.
2. Road-test stock-ACC curve entry, speed-limit anticipation, manual button
   override, and target release.
3. Switch among Wi-Fi, cellular, and offline modes while checking PPP teardown,
   reconnection, and Mici operator/status updates.
