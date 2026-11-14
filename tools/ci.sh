#!/usr/bin/env bash
# Run locally in the pinned Nordic container, or from GitHub Actions.
set -eo pipefail
source /opt/toolchain-env.sh
set -u

app="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sdk="${DIALSCROLL_SDK_DIR:-${RUNNER_TEMP:-/tmp}/dialscroll-ncs}"
sdk_revision=4c3fc0d44534f387a2d623eb1c85cbd2ffc8bcb6
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
mkdir -p "$app/artifacts"
exec > >(tee "$app/artifacts/ci.log") 2>&1

python -m unittest discover -s "$app/tests/uf2" -v
df -h "$app" "$sdk/.." 2>/dev/null || df -h "$app"

if [[ ! -d "$sdk/nrf/.git" ]]; then
    git clone --branch v3.2.4 --depth 1 \
        https://github.com/nrfconnect/sdk-nrf "$sdk/nrf"
fi
if [[ "$(git -C "$sdk/nrf" rev-parse HEAD)" != "$sdk_revision" ]]; then
    printf 'SDK checkout must be at %s\n' "$sdk_revision" >&2
    exit 1
fi
if [[ ! -f "$sdk/.west/config" ]]; then
    west init -l "$sdk/nrf"
fi
cd "$sdk"
west config manifest.project-filter -- '-matter'
west update --narrow -o=--depth=1
west zephyr-export
west manifest --freeze --active-only > "$app/artifacts/west-manifest.yml"
export ZEPHYR_BASE="$sdk/zephyr"
df -h "$sdk"

python "$ZEPHYR_BASE/scripts/twister" \
    -T "$app/tests" -p qemu_cortex_m3 --inline-logs -O "$app/twister-out"
python - "$app/twister-out/twister.json" <<'PY'
import json
from pathlib import Path
import sys

report = json.loads(Path(sys.argv[1]).read_text())
suites = report["testsuites"]
cases = [case for suite in suites for case in suite["testcases"]]
if len(suites) != 2 or len(cases) != 32 or any(case.get("status") != "passed" for case in cases):
    raise SystemExit("Expected two suites and 32 executed, passing ztests")
print("Verified: two suites, 32 passing ztests")
PY

image_directory="$(basename "$app")"
mkdir -p "$app/artifacts/firmware"
for profile in sd nosd diagnostic diagnostic-ble startup-diagnostic; do
    variant=nosd
    conf="$app/prj.conf;$app/prj_nosd.conf"
    case "$profile" in
        sd)
            variant=sd
            build=build
            name=blescroll
            conf="$app/prj.conf"
            ;;
        nosd)
            build=build-nosd
            name=blescroll_nosd
            ;;
        diagnostic)
            build=build-nosd-diagnostic
            name=blescroll_nosd_diagnostic
            conf+=";$app/prj_diagnostic.conf"
            ;;
        diagnostic-ble)
            build=build-nosd-diagnostic-ble
            name=blescroll_nosd_diagnostic_ble
            conf+=";$app/prj_diagnostic.conf;$app/prj_diagnostic_ble.conf"
            ;;
        startup-diagnostic)
            build=build-nosd-startup-diagnostic
            name=blescroll_nosd_startup_diagnostic
            conf+=";$app/prj_diagnostic.conf;$app/prj_encoder_startup_diagnostic.conf"
            ;;
    esac
    printf 'Building %s\n' "$profile"
    west -z "$ZEPHYR_BASE" build --sysbuild -p always \
        -b promicro_nrf52840/nrf52840/uf2 -d "$app/$build" "$app" \
        -- "-DCONF_FILE=$conf"
    output="$app/$build/$image_directory"
    python "$app/tools/verify_uf2.py" "$output/$name.uf2" --variant "$variant"
    cp "$output/$name.uf2" "$app/artifacts/firmware/"
    cp "$output/zephyr/zephyr.hex" "$app/artifacts/firmware/$name.hex"
    cp "$output/zephyr/zephyr.elf" "$app/artifacts/firmware/$name.elf"
    cp "$output/zephyr/.config" "$app/artifacts/firmware/$name.config"
done
