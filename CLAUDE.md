# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository

Official device firmware for Meshtastic, an open-source LoRa mesh networking project. C++ (Arduino framework) built with PlatformIO, targeting ESP32, nRF52, RP2040/RP2350, STM32WL, and Linux (`native`/`portduino`) platforms.

This branch (`mictronics`) is a customized fork for the Lilygo T-Echo running as a remotely located router — power-optimized, screen support removed, and hardcoded config that restores router behavior after factory reset (see `userPrefs.jsonc`).

## Build

Builds are per-board PlatformIO environments. `platformio.ini` at the repo root defines shared `[env]` settings and library groups (`environmental_base`, etc.); each board's actual `[env:<name>]` lives in `variants/<arch>/<board>/platformio.ini` and is pulled in via `extra_configs` or discovered by PlatformIO's default variant globbing. Boards extend an arch-level base (e.g. `esp32_base`, `native_base`) defined in `variants/<arch>/platformio.ini`.

```bash
# Build a specific board environment
pio run -e t-echo
pio run -e native            # Linux/portduino build (meshtasticd)

# Convenience wrappers (used by CI, handle packaging/output paths)
./bin/build-esp32.sh <env>
./bin/build-nrf52.sh <env>
./bin/build-rp2xx0.sh <env>
./bin/build-stm32wl.sh <env>
./bin/build-native.sh native   # produces ./release/meshtasticd-native-<version>

# Run the native build directly (no device needed)
./bin/native-run.sh
./bin/native-gdbserver.sh      # under gdbserver on localhost:2345
```

List available board environments with `pio project data` or by browsing `variants/`. There is no single default env — `platformio.ini`'s `default_envs` is commented out; always pass `-e <env>`.

Static analysis (cppcheck via PlatformIO) for a curated board list:
```bash
./bin/check-all.sh                 # default board list
./bin/check-all.sh tbeam t-echo    # specific boards
```

Formatting is enforced via [Trunk](https://trunk.io) (`.trunk/trunk.yaml`), which wraps `clang-format` for C/C++ plus linters for Python, shell, YAML, Markdown, etc.:
```bash
trunk fmt
```

## Tests

Unit tests use Unity and run on the host via the `native` PlatformIO environment (see `test/README.md` for the full authoring guide — read it before adding a new suite).

```bash
pio test -e native                          # all suites
pio test -e native -f test_your_module       # one suite
pio test -e native -f test_your_module -vvv  # verbose, shows build errors

./bin/test-native-docker.sh                  # run in Docker (recommended off-Linux, closest to CI)
./bin/test-native-docker.sh -f test_your_module
./bin/test-native-docker.sh --rebuild        # after dependency changes

pio run -e native && ./bin/test-simulator.sh # simulator integration check
```

Native build system deps (Ubuntu/Debian; canonical list in `.github/actions/setup-native/action.yml`):
```bash
sudo apt-get install -y libbluetooth-dev libgpiod-dev libyaml-cpp-dev openssl libssl-dev \
  libulfius-dev liborcania-dev libusb-1.0-0-dev libi2c-dev libuv1-dev
```

Test suites live under `test/test_<name>/`, one `test_main.cpp` per suite, built automatically under `[env:native]` — no per-suite `platformio.ini`. Key conventions from `test/README.md`:
- Wrap test bodies in the same `#if <FEATURE_GUARD>` the module under test uses; provide an empty `#else` suite so it still builds when the feature is compiled out.
- `initializeTestEnvironment()` (from `TestUtil.h`) must be called first in `setup()`; the runner must `exit(UNITY_END())`.
- Modules with a global singleton pointer (`extern FooModule *fooModule;`) or persisted state under `/prefs/*.bin` need explicit reset in `setUp`/`tearDown` — state and file-scope `static` globals otherwise leak between tests.
- `friend class FooTestShim;` guarded by `#ifdef UNIT_TEST` is the standard way to expose protected/private members to tests.

## Architecture

- **`src/mesh/`** — core mesh networking: `NodeDB` (node database/persistence), `MeshService`, packet routing (`FloodingRouter`, `NextHopRouter`, `ReliableRouter`), radio interfaces (`RadioLibInterface` and per-chip subclasses like `LR11x0Interface`, `LLCC68Interface`), crypto (`CryptoEngine`, `aes-ccm`), channels, and transport backends (`api/`, `http/`, `eth/`). Protobuf-generated message code lives in `src/mesh/generated/` and is **committed to the repo** (not gitignored) — regenerate via `bin/regen-protos.sh` after editing `protobufs/` (a git submodule tracking `meshtastic/protobufs`), which requires the nanopb 0.4.9 toolchain.
- **`src/modules/`** — pluggable application-level modules (admin, telemetry via `Telemetry/`, canned messages, store-and-forward, position, routing, etc.), registered centrally in `Modules.cpp`. New modules typically subclass `MeshModule` (`src/mesh/MeshModule.h`).
- **`src/graphics/`** — display rendering (`Screen`, OLED/E-Ink drivers); `src/graphics/niche/InkHUD/` is a separate self-contained UI framework for E-Ink devices with its own PlatformioConfig.ini.
- **`src/concurrency/`** — a lightweight cooperative pseudo-threading layer (`OSThread`/`Thread`/`ThreadController`, `Periodic`) used instead of a real RTOS abstraction across all platforms, plus platform-specific locks/semaphores.
- **`src/platform/`** and per-arch code (guarded by `ARCH_ESP32` / `ARCH_NRF52` / `ARCH_PORTDUINO` / etc. and by feature macros like `MESHTASTIC_EXCLUDE_GPS`, `HAS_WIFI`, `HAS_ETHERNET`) — most cross-platform behavior differences are handled via preprocessor guards rather than runtime polymorphism; check `main.cpp` for the canonical set of guards when adding platform-conditional code.
- **`variants/<arch>/<board>/`** — per-board pin mappings/config (`variant.h`) and the board's PlatformIO env definition. Adding a new board means adding a variant directory and (usually) a line under `extra_configs` in the root `platformio.ini`.
- **`protobufs/`** and **`meshtestic/`** are git submodules; run `git submodule update --init` if they appear empty.
- **`userPrefs.jsonc`** — build-time default overrides (e.g. hardcoded channel PSKs/names for this fork's router config) consumed via `bin/build-userprefs-json.py` into `USERPREFS_*` macros; treat its contents as environment-specific, not general firmware behavior.

## Contributing conventions

- Format with `trunk fmt` before submitting changes (wraps `clang-format` for C++).
- No CI workflow files are present in this branch's `.github/` (only `actions/` composite actions) — the reference CI behavior is what `check-all.sh`, `pio test -e native`, and `bin/test-native-docker.sh` do locally.
