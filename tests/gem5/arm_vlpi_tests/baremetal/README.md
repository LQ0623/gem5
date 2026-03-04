# ARM vLPI 裸机闭环测试（完整文件版）

你要的裸机运行文件都在这里：

- `start.S`（启动与异常向量）
- `vlpi_baremetal.c`（闭环逻辑）
- `link.ld`（链接脚本）
- `Makefile`（构建 + 运行）

## 构建

```bash
cd tests/gem5/arm_vlpi_tests/baremetal
make
```

如果工具链是 `aarch64-linux-gnu-`：

```bash
make CROSS_COMPILE=aarch64-linux-gnu-
```

## 运行（direct）

```bash
make run-direct
```

## 运行（trap）

```bash
make run-trap
```

## 期望串口关键字

- `vLPI IRQ INTID=`
- `latency=`
- `EOI done`

程序流程：

1. EL1 启动并安装向量表。
2. 使能 GIC CPU IF + LPI。
3. 由裸机写 ITS 命令队列：`MAPD/VMAPP/VMAPTI/SYNC`。
4. 写 `GITS_TRANSLATER` 触发事件。
5. 进入 IRQ handler，读取 `ICC_IAR1_EL1`，打印 INTID/latency，写 `ICC_EOIR1_EL1`。

> 注意：裸机通过 CPU 直接写 `GITS_TRANSLATER`，请求中没有 MSI StreamID。
> 当前实现会回退使用 `device_id=0`，因此程序里 `DEVICE_ID` 固定为 `0`。
