#!/usr/bin/env bash
set -euo pipefail

: "${GEM5_BIN:=build/ARM/gem5.opt}"
: "${FS_CONFIG:=configs/example/arm/fs_bigLITTLE.py}"
: "${KERNEL:?set KERNEL path}"
: "${DISK:?set DISK path}"
: "${BOOT_SCRIPT:?set BOOT_SCRIPT path (microbench launcher)}"
: "${OUTDIR:=m5out-vlpi-direct}"

"${GEM5_BIN}" \
  --debug-flags=ITS \
  --outdir="${OUTDIR}" \
  "${FS_CONFIG}" \
  --kernel "${KERNEL}" \
  --disk "${DISK}" \
  --script "${BOOT_SCRIPT}" \
  --num-cpus=1 \
  --param "system.realview.gic.gicv4=True" \
  --param "system.realview.gic.its.direct_vlpi=True"

python3 tests/gem5/arm_vlpi_tests/check_vlpi_result.py \
  --terminal "${OUTDIR}/system.terminal" \
  --debug "${OUTDIR}/simout" \
  --mode direct
