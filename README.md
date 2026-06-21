# esphome_victron_nimble

Minimal ESPHome firmware to validate **hosted NimBLE** Victron BLE on the **Guition ESP32-P4 + ESP32-C6** (SDIO) board — without LVGL, MQTT, or the full guition-dashboard4 stack.

This repo exists to prove scan-only Victron decoding and measure **internal heap** savings before merging into the main dashboard project.

## Why a separate repo?

The main dashboard (`guition-dashboard4`) uses ESPHome's **Bluedroid** stack (`esp32_ble`, `ble_client`, `esp32_ble_tracker`) plus external `esphome-victron_ble`. That costs significant internal RAM on P4.

The planned migration is **not** to port those generic components to NimBLE. Instead:

1. **`nimble_host`** — shared hosted NimBLE stack on P4+C6 (one copy per firmware).
2. **Per-device custom components** — only the BLE surface each device needs.
3. **Victron** — scan + manufacturer-data decrypt only (no GATT connection slot).

A working ELM327 NimBLE reference (GATT client pattern) lives on the esphome fork:

[guition-dashboard-nimble on `nimble-test`](https://github.com/mikesloper/esphome/tree/nimble-test/guition-dashboard-nimble)

## Hardware

- Board: Guition JC1060P470 (ESP32-P4 + ESP32-C6 via `esp32_hosted` SDIO)
- Config: `guition-esp32-p4.yaml` (display/touch omitted for fast compile)

## Components

| Component | Role |
|-----------|------|
| `components/nimble_host/` | Enables hosted NimBLE via sdkconfig; deferred start after WiFi |
| `components/nimble_victron/` | Passive scan → bindkey decrypt → publish template sensors |

**Do not add** `esp32_ble`, `esp32_ble_tracker`, or `ble_client` to this project — they pull in Bluedroid and defeat the purpose.

### Supported Victron record types (initial)

- `BATTERY_MONITOR` (SmartShunt / BMV)
- `SOLAR_CHARGER` (SmartSolar MPPT)

Decode logic follows [Fabian-Schmidt/esphome-victron_ble](https://github.com/Fabian-Schmidt/esphome-victron_ble). More record types can be added to `victron_types.h` as needed.

## Quick start

```powershell
cd d:\projects\esphome_victron_nimble
copy secrets.yaml.example secrets.yaml
# Edit secrets.yaml and victron/smartshunt.yaml (MAC + bindkey)

# Use the forked ESPHome (stock 2026.5.3 on PATH may lack P4/hosted fixes)
D:\projects\esphome-fork\esphome\.esphome-venv\Scripts\esphome.exe config victron-test.yaml
D:\projects\esphome-fork\esphome\.esphome-venv\Scripts\esphome.exe compile victron-test.yaml
D:\projects\esphome-fork\esphome\.esphome-venv\Scripts\esphome.exe run victron-test.yaml
```

### Bindkey

From Victron VicConnect: **Settings → Product info → VRM online portal → Encryption data**.

### Device config

Edit `victron/smartshunt.yaml`:

```yaml
nimble_victron:
  mac_address: XX:XX:XX:XX:XX:XX
  bindkey: "32hexchars..."
  battery_voltage_sensor: shunt_battery_voltage
  # ...
```

Multiple `nimble_victron` blocks share one passive scanner (one MAC each).

## Repo layout

```
components/
  nimble_host/       # shared NimBLE host (copy from guition-dashboard-nimble)
  nimble_victron/    # Victron scan decoder
guition-esp32-p4.yaml
wifi-nimble.yaml     # nimble_host.enable 8s after WiFi connect
victron/
  smartshunt.yaml    # example device package
victron-test.yaml    # main build target
```

## Validation checklist

- [ ] WiFi connects; NimBLE enables ~8s later (see `nimble_host` logs)
- [ ] SmartShunt sensors update (voltage, current, SOC, …)
- [ ] Debug sensors show stable **Free Internal Heap** over 10+ minutes
- [ ] Second Victron device (e.g. SmartSolar) if testing multi-device scan

## Merge path → guition-dashboard4

When this repo is stable:

1. Copy `components/` into `guition-dashboard4/esphome/custom_components/` (or keep in esphome fork).
2. Replace `packages/victron/*.yaml` + remove `esphome-victron_ble` external component for those devices.
3. Remove `esp32_ble` / `esp32_ble_tracker` from base yaml once **all** BLE devices are on NimBLE components.
4. Port GATT devices next (Sunster, JDB, Renogy) using the ELM327 `nimble_elm327` pattern from `nimble-test`.

## Related projects

| Project | Purpose |
|---------|---------|
| `guition-dashboard4` | Full LVGL dashboard (Bluedroid today) |
| `esphome` fork `nimble-test` / `guition-dashboard-nimble` | ELM327 NimBLE reference + A/B heap test vs Bluedroid |

## AI / Cursor

See **`AGENTS.md`** for agent context when opening this folder as its own workspace.
