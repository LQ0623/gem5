# GIC-700 vLPI 直注入实现说明（gem5）

## 修改目的

打通闭环：

`MSI -> ITS 翻译 -> (trap-based: pINTID 注入 EL2 / direct: 直写 VPENDBASER) -> EL1 中断处理 -> EOI`

本次改动提供了两种路径，且可通过参数切换：

- `system.realview.gic.its.direct_vlpi=False`：trap-based（通过 pINTID 到 EL2）
- `system.realview.gic.its.direct_vlpi=True`：direct（ITS 直写 vPE pending 表）

## 关键实现点

1. **GICD_TYPER.DVIS**
   - 在 GICv4 模式下置位 DVIS（表示支持 direct virtual LPI injection）。

2. **GITS_TYPER 关键字段与状态保存**
   - 当 GICv4 打开时，补齐 `GITS_TYPER` 的虚拟化相关能力位（`VMOVP/Virtual/Physical/SEIS/CCT`）。
   - 将新增/关键运行态寄存器纳入 checkpoint：`gitsTranslater`、`directVlpi`。

3. **vLPI 直注入**
   - ITS 翻译结果如果是 virtual interrupt：
     - direct 模式：写 `GICR_VPENDBASER` 指向的 pending 位图。
     - trap 模式：注入 `pINTID`（`ITTE.intNumHyp`）给 EL2。

## 新增测试文件

- `tests/gem5/arm_vlpi_tests/run_vlpi_trap.sh`
- `tests/gem5/arm_vlpi_tests/run_vlpi_direct.sh`
- `tests/gem5/arm_vlpi_tests/check_vlpi_result.py`

### 约定的 guest 输出标记

微基准（你的 guest 程序）需在串口输出下列字符串，便于自动检查：

- `vLPI IRQ INTID=<id>`
- `latency=<cycles_or_ns>`
- `EOI done`

### trap/direct 区分依据

脚本使用 ITS debug 输出检查路径：

- trap: `vLPI trap inject pINTID=`
- direct: `vLPI direct inject vINTID=`

## 编译与运行

### 1) 编译 gem5

```bash
scons build/ARM/gem5.opt -j$(nproc)
```

### 2) 运行 trap-based 模式

```bash
KERNEL=<Image> \
DISK=<disk.img> \
BOOT_SCRIPT=<guest-run-vlpi-bench.rcS> \
GEM5_BIN=build/ARM/gem5.opt \
./tests/gem5/arm_vlpi_tests/run_vlpi_trap.sh
```

### 3) 运行 direct 模式

```bash
KERNEL=<Image> \
DISK=<disk.img> \
BOOT_SCRIPT=<guest-run-vlpi-bench.rcS> \
GEM5_BIN=build/ARM/gem5.opt \
./tests/gem5/arm_vlpi_tests/run_vlpi_direct.sh
```

### 4) 结果判定

`check_vlpi_result.py` 成功返回：

- `PASS: trap mode closed-loop verified`
- `PASS: direct mode closed-loop verified`

分别对应两种注入路径闭环打通。


## 附带的最小可运行检查样例

仓库已附带一组最小样例文件（不依赖真实 guest）用于验证检查器脚本本身：

- `tests/gem5/arm_vlpi_tests/fixtures/terminal.ok`
- `tests/gem5/arm_vlpi_tests/fixtures/simout.trap`
- `tests/gem5/arm_vlpi_tests/fixtures/simout.direct`
- `tests/gem5/arm_vlpi_tests/run_checker_tests.sh`

运行：

```bash
./tests/gem5/arm_vlpi_tests/run_checker_tests.sh
```

期望输出两行 `PASS`，分别对应 trap/direct。
