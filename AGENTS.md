# OpenIris-ESPIDF — Agent guide

## Build system

- ESP-IDF **v5.4.2**, targets `esp32` / `esp32s3`. Build from an **ESP-IDF Command Prompt** (not plain PowerShell) using `idf.py` commands.
- Python tools are managed by **uv** (not pip). Run all scripts as `uv run tools/some_script.py`.
- Project is named `blink` internally (legacy). Ignore — the firmware is OpenIris.

## Board configuration

- `boards/sdkconfig.base_defaults` is the shared baseline. Each board under `boards/` overlays only differing `CONFIG_*` lines.
- **Always switch board before building:**
  ```
  uv run tools/switchBoardType.py --list
  uv run tools/switchBoardType.py --board <key> [--diff]
  ```
- Board key = relative path from `boards/` with `/` → `_`. Duplicate tail segments collapse.
- Platform-specific components (`usb_device_uvc`) are auto-configured by `switchBoardType.py` based on target.
- To add a new board: create a config file under `boards/`, only the `CONFIG_*` lines that differ from `sdkconfig.base_defaults`.

## Build & flash

```cmd
idf.py set-target <esp32|esp32s3>
idf.py build
idf.py -p COMxx flash monitor
```

- UVC mode requires `GENERAL_INCLUDE_UVC_MODE=y`. Auto-start in UVC: `START_IN_UVC_MODE=y`.
- Disable Wi-Fi for pure wired builds: `GENERAL_ENABLE_WIRELESS=n`.

## Key architecture

| Directory | Role |
|-----------|------|
| `main/` | Entrypoint (`openiris_main.cpp`) |
| `components/` | 20 modules (Camera, WiFi, UVC, CommandManager, StateManager, etc.) |
| `components/usb_device_uvc/` | **Custom fork** of espressif UVC — modified, ships in-repo |
| `tools/` | Python helpers: board switching, setup CLI, device library |
| `tests/` | Hardware-in-the-loop pytest suite |
| `managed_components/` | ESP-IDF component manager deps (esp32-camera, tinyusb, mdns, led_strip, etc.) |

- Event-driven via FreeRTOS queues: `eventQueue`, `ledStateQueue`, `cdcMessageQueue`.
- Commands routed through `CommandManager` (`components/CommandManager/`).
- JSON-over-serial protocol: `{"commands":[{"command":"...","data":{...}}]}`.

## Python tooling (uv)

```cmd
uv run tools/switchBoardType.py --board <key>      # switch board config
uv run tools/setup_openiris.py --port COMxx         # interactive setup CLI
```

Serial connection: 115200 baud, `dtr=false`, `rts=false` (see `tools/openiris_device.py`).

## Testing (hardware-in-the-loop)

- Requires a real board connected via USB serial.
- Copy `tests/.env.example` → `tests/.env` and fill in Wi-Fi credentials.

```cmd
uv run pytest --board=<name> --connection=COM<port>
uv run pytest --board=<name> --connection=COM<port> -k test_name   # single test
uv run pytest --board=<name> --connection=COM<port> --lf            # last failures
```

- Tests use capability markers (`has_capability`, `lacks_capability`) to auto-skip based on board. See `tests/conftest.py` for capability map.
- After the session ends, the board config is **reset** and the board **reboots**.

## CI

- Workflow: `.github/workflows/build-and-release.yml`
- Builds all 12 boards, merges binary, archives. On tag push (`*.*.*`): creates a GitHub release with merged `.bin` zips.
- Uses `espressif/esp-idf-ci-action@v1` with `esp_idf_version: v5.4.2`.

## Versioning

- Version source: `pyproject.toml` (`version` field), synced to `sdkconfig` via `bumpver`.
- Version pattern: `MAJOR.MINOR.PATCH[PYTAGNUM]` (e.g. `0.2.1rc0`).

## Conventions

- C/C++: Google style (`.clang-format`), 4-space indent, Allman braces, 160 col limit, case labels not indented.
- Python: requires `>=3.12`, managed via uv.
- Use `json` for command payloads. Commands are newline-terminated JSON.
- `CONFIG_GENERAL_ADVERTISED_NAME` is the single source for both UVC device name and mDNS hostname.
