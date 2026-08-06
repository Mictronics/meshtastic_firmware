# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository

This is the official **Meshtastic device firmware** (C++/Arduino, built with PlatformIO). It targets ESP32, nRF52, RP2040/RP2350, STM32WL and Linux ("Portduino") hardware, implementing a decentralized LoRa mesh network for text messaging, location sharing, and telemetry.

There are three branches in play here, each building on the last:

- `master` — upstream Meshtastic firmware.
- `mictronics` — a modified version of `master` (maintained by Mictronics).
- `barebones` (current branch) — a heavily modified version of `mictronics`, purpose-built for nodes deployed in remote locations with no direct user monitoring and sometimes no direct connection to the node at all. The goal is minimum mesh functionality plus monitoring of vital node parameters, nothing else: only the Telemetry (device/environment/host/power), remote admin, node info, routing, text message, and time modules are compiled in, plus a handful of environmental/power sensors (BMP280, BME280, BME680, BMP3xx, BMP085, INA219, INA226, INA3221). Screen, GPS, Bluetooth, WiFi, MQTT, I2C scanning, PKI, and the FSM/power-management UI are compiled out via `MESHTASTIC_EXCLUDE_*` macros in `src/configuration.h`. Keep this exclusion set in mind: code guarded by an excluded macro will not build/run on this branch's target environments even though it exists in the tree.

Primary target hardware is RP2040/RP2350 — these boards additionally have an external hardware watchdog and battery voltage monitoring. Some nodes also run on the other supported platforms (nRF52, ESP32, etc.). No node platform on this branch has GPS attached — GPS is compiled out entirely, not just unused at runtime.

## Build

Build system is PlatformIO. `platformio.ini` at the repo root defines shared build flags/lib deps and pulls in per-target config from `variants/*/*/platformio.ini` (each variant is a separate PlatformIO `env`).

```bash
# Build one environment (see platformio.ini for the full commented-out list of env names)
pio run -e native
pio run -e rp2040-lora
pio run -e t-echo

# Helper scripts (used by CI) wrap `pio run` per architecture:
bin/build-esp32.sh <env>
bin/build-nrf52.sh <env>
bin/build-rp2xx0.sh <env>
bin/build-stm32wl.sh <env>
bin/build-native.sh native      # produces a release artifact under ./release/

# Run the natively-built Linux daemon (meshtasticd)
bin/native-run.sh
bin/native-gdbserver.sh         # same, under gdbserver on localhost:2345
```

`build_all.sh` is a repo-owner convenience script: it patches `userPrefs.jsonc` per-device (owner name/short name/role) from a hardcoded table and builds+copies firmware for a fixed list of real deployed nodes into `.builds/`. It is not a generic build entrypoint — don't generalize from it. `userPrefs.jsonc` itself carries this fork's default owner name, channel PSKs, and region — treat it as environment/deployment configuration, not application code.

Static analysis: `bin/check-all.sh [boards...]` runs `pio check` (cppcheck-based) against `src/` for a default board list (override by passing board names as args).

## Test

Native unit tests use PlatformIO's `native` env + the Unity framework. **Read `test/README.md` before adding or modifying tests** — it documents the required test-suite skeleton, the `MockNodeDB` pattern, the `UNIT_TEST`-guarded friend-class shim for exposing private members, and a checklist of common pitfalls (persisted `/prefs/*.bin` state leaking between tests, file-scope static globals not being reset, non-deterministic `rand()`-based jitter, time-dependent rolling averages collapsing to zero in fast test runs, fixed-capacity buffers overflowing under injected test data).

```bash
pio test -e native                       # all suites
pio test -e native -f test_your_module   # one suite
pio test -e native -f test_your_module -vvv   # verbose, shows build errors

./bin/test-native-docker.sh              # run native tests in Docker (needed on macOS/non-Linux hosts)
./bin/test-native-docker.sh -f test_your_module
./bin/test-native-docker.sh --rebuild    # after dependency changes

pio run -e native && ./bin/test-simulator.sh   # simulator integration check
```

Native builds need system libs: `libbluetooth-dev libgpiod-dev libyaml-cpp-dev openssl libssl-dev libulfius-dev liborcania-dev libusb-1.0-0-dev libi2c-dev libuv1-dev` (canonical list: `.github/actions/setup-native/action.yml`).

## Formatting

Formatting/linting is centralized via [Trunk](https://trunk.io) (`.trunk/trunk.yaml`), including `clang-format` for C/C++ (config at `.trunk/configs/.clang-format`). Run `trunk fmt` before submitting changes.

## Architecture

- **`src/mesh/`** — core mesh networking: `Router.cpp`/`FloodingRouter.cpp`/`NextHopRouter.cpp`/`ReliableRouter.cpp` (packet routing strategies), `NodeDB.cpp` (the node database — most modules read/write through the global `nodeDB`), `MeshService.cpp`, `PhoneAPI.cpp`/`StreamAPI.cpp` (client protocol), radio driver interfaces (`RadioLibInterface.cpp` and per-chip `SX126xInterface`/`SX128xInterface`/`LR11x0Interface`/`RF95Interface` etc.), and `generated/` (nanopb-generated protobuf C structs — do not hand-edit).
- **`src/modules/`** — pluggable app-level modules (each typically subclasses `MeshModule` or `SinglePortModule`/`ProtobufModule`). New modules are instantiated in `setupModules()` in `src/modules/Modules.cpp`, each guarded by its `MESHTASTIC_EXCLUDE_*` macro — follow that pattern (guard + conditional `new`) when adding a module.
- **`src/platform/{esp32,nrf52,rp2xx0,stm32wl,portduino}/`** — per-MCU-family HAL implementations. `ARCH_*`/target macros (set per PlatformIO env) select which platform code compiles; `src/target_specific.h` and `src/configuration.h` are the central places these capability flags are defined.
- **`src/graphics/`, `src/power/`, `src/gps/`, `src/detect/`** — screen/UI, power management/battery, GPS, and I2C device auto-detection subsystems respectively (several of these are compiled out on this branch, see above).
- **`variants/<mcu-family>/<board>/`** — per-board pin maps, `platformio.ini` fragments, and (for the `native`/Portduino target) the Linux daemon build config. `variants/native/portduino/platformio.ini` is the Linux (`meshtasticd`) build; `portduino-buildroot/` is a variant of that for buildroot/embedded Linux targets.
- **`protobufs/`** — git submodule of the [meshtastic/protobufs](https://github.com/meshtastic/protobufs) wire-format definitions. Generated C code lives in `src/mesh/generated/`; regenerate via `bin/regen-protos.sh` (requires the nanopb 0.4.9 toolchain) — never hand-edit generated files.
- **`bin/`** — build/release tooling (per-arch build wrappers, `buildinfo.py` for version strings, `regen-protos.sh`, install/flash scripts, native test/run helpers). Treat scripts here as the source of truth for how CI actually builds/tests, ahead of any docs.

Feature-set gating throughout the codebase is driven by `MESHTASTIC_EXCLUDE_*` preprocessor defines set in `src/configuration.h` (or per-variant build flags) — when touching any module/sensor/interface, check whether it's wrapped in one of these before assuming it's active on a given build target.
