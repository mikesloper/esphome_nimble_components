# Agent context — esphome_victron_nimble

Read this file first when working in this repository.

## Mission

Validate **Victron BLE over hosted NimBLE** on ESP32-P4 + ESP32-C6 (Guition JC1060P470). This is a **minimal test harness**, not the production dashboard.

Success criteria:

- Victron advertisements decrypt and sensors publish correctly.
- Internal heap is measurably better than Bluedroid + `esp32_ble_tracker` + `esphome-victron_ble` (use debug sensors in `victron-test.yaml`).
- Components are reusable when merged into `guition-dashboard4`.

## Architecture (do not regress)

```
WiFi connect → delay 8s → nimble_host.enable
                ↓
         nimble_host (hosted NimBLE on C6 via VHCI)
                ↓
    nimble_victron × N (passive scan, MAC filter, AES-CTR decrypt)
                ↓
         template sensors (API / logs)
```

**Forbidden in this repo:** `esp32_ble`, `esp32_ble_tracker`, `ble_client`, external `esphome-victron_ble`. They use Bluedroid and duplicate the stack we are replacing.

**Required:** `esp32_hosted`, local `nimble_host`, local `nimble_victron`.

## Toolchain

- **ESPHome:** fork at `D:\projects\esphome-fork\esphome` (`.esphome-venv\Scripts\esphome.exe`). Stock PATH ESPHome may lack P4/hosted fixes.
- **Main config:** `victron-test.yaml`
- **Secrets:** `secrets.yaml` (gitignored); copy from `secrets.yaml.example`

## Key files

| Path | Purpose |
|------|---------|
| `components/nimble_host/` | sdkconfig: Bluedroid off, `CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE` on |
| `components/nimble_victron/` | Scan callback, decrypt, publish sensors |
| `victron/smartshunt.yaml` | Device MAC, bindkey, sensor wiring |
| `wifi-nimble.yaml` | Deferred `nimble_host.enable` after WiFi |
| `guition-esp32-p4.yaml` | P4 + C6 SDIO; no display/LVGL |

## Victron implementation notes

- **Scan-only** — no `ble_gap_connect`, no connection slots.
- Manufacturer ID `0x02E1`; product advertisement record type `0x10`.
- Bindkey is 16 bytes; first byte appears in cleartext as `encryption_key_0`.
- Decrypt: AES-CTR with nonce `[counter_lsb, counter_msb, 0, …]` (same as esphome-victron_ble).
- MAC: ESPHome stores `uint64` LSB-first per byte; NimBLE `ble_addr_t.val` matches ELM327 pattern in `nimble_elm327`.
- Deduplicate by `data_counter` (LSB + MSB).
- Record structs: `components/nimble_victron/victron_types.h` (subset of Fabian-Schmidt headers).

When adding record types, copy struct layouts from [esphome-victron_ble `victron_ble.h`](https://github.com/Fabian-Schmidt/esphome-victron_ble/blob/main/components/victron_ble/victron_ble.h) — do not guess field order.

## Reference implementations

| Reference | URL / path | Use for |
|-----------|------------|---------|
| NimBLE host + ELM327 GATT | [mikesloper/esphome `nimble-test/guition-dashboard-nimble`](https://github.com/mikesloper/esphome/tree/nimble-test/guition-dashboard-nimble) | `nimble_host`, connect/notify/write pattern for future devices |
| Victron decode (Bluedroid) | [Fabian-Schmidt/esphome-victron_ble](https://github.com/Fabian-Schmidt/esphome-victron_ble) | Record layouts, sensor scaling, bindkey handling |
| Production dashboard | `d:\projects\guition-dashboard4` | Merge target; `packages/victron/*.yaml` shows which sensors the UI needs |

## Broader migration plan (parent project)

1. **This repo** — Victron on NimBLE (current).
2. **guition-dashboard4** — merge `nimble_host` + `nimble_victron`; drop Victron external component + tracker for those devices.
3. **GATT devices** — `nimble_sunster`, `nimble_jbd_bms`, `nimble_elm327` (one component each); `nimble_renogy` still TODO.
4. **Remove Bluedroid** from base yaml only when all active BLE devices are migrated.
5. **Disabler** — optional during transition; prefer compile-time packages (include only devices in use) for real RAM wins.

Estimated internal RAM win vs full Bluedroid dashboard stack: on the order of **~55–105 KB** (validate with debug sensors, do not assume).

## Coding conventions

- Match existing `nimble_host` / `nimble_elm327` style: ESPHome `Component`, `setup_priority::BUS`, `add_on_sync_callback` for stack-ready work.
- Keep components **minimal** — no port of full esphome-victron_ble sensor enum; wire only sensors needed for yaml.
- Python `__init__.py`: `cv.only_on_esp32`, `bind_key_array` pattern from victron_ble.
- Comments only for non-obvious protocol/crypto details.
- Do not add LVGL, MQTT, or disabler unless explicitly requested.

## Common tasks

### Add SmartSolar

Duplicate `victron/smartshunt.yaml` → `victron/smartsolar.yaml`; wire `pv_power_sensor`, `solar_battery_voltage_sensor`, `solar_battery_current_sensor`; include from `victron-test.yaml`.

### Add second shunt

Second `nimble_victron:` block with different `id`, MAC, bindkey, sensors. Scanner is shared.

### Debug no data

- Confirm bindkey byte 0 matches log warning.
- Confirm MAC matches advertisement (enable DEBUG on `nimble_victron`).
- Confirm NimBLE synced (`nimble_host` log) and scan started.
- Victron must be advertising (device powered, BLE enabled in Victron app).

## What not to do

- Do not enable `esp32_ble` “just for testing” alongside NimBLE.
- Do not pull entire esphome-victron_ble as external component (defeats RAM goal).
- Do not add display/touch to this repo unless asked — slows compile iteration.
- Do not commit `secrets.yaml`.
