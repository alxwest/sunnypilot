# `dev` compared with upstream `sunny/dev`

Comparison date: 2026-07-25

## Branch points

| Branch | Commit | Meaning |
|---|---|---|
| `sunny/dev` | `543ade4eb9` | Upstream packaged build |
| `sunny/master-dev` | `7801bdf0cc` | Source commit embedded in that packaged build |
| `dev-alx` | `ba2e53ed0f` | Original local development branch before this port |
| `dev` | `a6f04a2b5e` plus this documentation commit | `sunny/dev` plus the ported local work |

The upstream build reports:

- openpilot `0.11.2`
- sunnypilot `2026.003.000` development
- packaged build `2026.07.25-4590`

The old `dev-alx` branch reports openpilot `0.11.1` and sunnypilot
`2026.001.000`.

`sunny/dev` is a packaged deployment tree with history separate from the source
branch. It has no merge base with `dev-alx`; its embedded source commit on
`sunny/master-dev` shares source ancestor `f93481d0d4` with `dev-alx`.

## Changes in the new `dev`

The new `dev` starts exactly at `sunny/dev` and adds eleven functional and
deployment commits:

1. Radar-track rendering on comma four/Mici.
2. MRR35 raw radar-track support for Kia Niro EV HDA2.
3. Automatic Niro EV HDA2 feature setup.
4. Stock-ACC synchronization for curve and speed-limit targets.
5. Earlier vision-curve anticipation.
6. Earlier map-curve anticipation and release tuning.
7. Anticipated map speed limits for intelligent cruise button management.
8. Cellular network status on the Mici home screen.
9. A Wi-Fi/cellular/offline network-mode selector.
10. Radar-track control in the current sunnylink settings schema.
11. A rebuilt packaged `params_pyx.so` containing the added parameter keys,
    while retaining the `prebuilt` marker required by the deployment tree.

The source diff from `sunny/dev` is approximately 32 modified or added source
files, with about 937 insertions and 70 deletions, plus the deleted empty
`prebuilt` marker and documentation.

## Notable upstream differences

The port is not a mechanical replay of `dev-alx`. Important upstream changes
include:

- openpilot and sunnypilot source packages now live beneath `openpilot/`.
- opendbc is vendored in `opendbc_repo/`; the old `.gitmodules` fork-pointer
  commit no longer applies.
- radar data now uses `LeadData.present` instead of `LeadData.status`.
- sunnylink settings are generated from YAML rather than the old
  `params_metadata.json`.
- longitudinal planner, model, driver-monitoring, and health-check interfaces
  changed.
- hardware modules moved under `openpilot/common/hardware`.
- upstream cellular service uses its own PPP modem state machine rather than
  NetworkManager WWAN control.
- upstream already detects cellular service when no primary route exists, so
  that old local patch did not need a duplicate implementation.
- the old eSIM modem-reboot hook no longer exists; upstream modem locking and
  ICCID-change handling supersede it.
- `sunny/dev` intentionally omits SCons build files, so native changes must be
  supplied as updated packaged artifacts rather than rebuilt by the launcher.

See [DEV_ALX_PORT_REPORT.md](DEV_ALX_PORT_REPORT.md) for the commit mapping,
adaptations, and validation report.
