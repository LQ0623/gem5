#!/usr/bin/env bash
set -euo pipefail
python3 tests/gem5/arm_vlpi_tests/check_vlpi_result.py \
  --terminal tests/gem5/arm_vlpi_tests/fixtures/terminal.ok \
  --debug tests/gem5/arm_vlpi_tests/fixtures/simout.trap \
  --mode trap
python3 tests/gem5/arm_vlpi_tests/check_vlpi_result.py \
  --terminal tests/gem5/arm_vlpi_tests/fixtures/terminal.ok \
  --debug tests/gem5/arm_vlpi_tests/fixtures/simout.direct \
  --mode direct
