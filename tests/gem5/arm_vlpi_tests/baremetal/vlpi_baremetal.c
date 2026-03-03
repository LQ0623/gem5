#include <stdint.h>

#ifndef GICD_BASE
#define GICD_BASE 0x2c000000UL
#endif
#ifndef GICR_BASE
#define GICR_BASE 0x2c010000UL
#endif
#ifndef GITS_BASE
#define GITS_BASE 0x2e010000UL
#endif
#ifndef UART_BASE
#define UART_BASE 0x1c090000UL
#endif

#define GICD_CTLR (GICD_BASE + 0x0000UL)

#define GICR_CTLR      (GICR_BASE + 0x0000UL)
#define GICR_WAKER     (GICR_BASE + 0x0014UL)
#define GICR_PROPBASER (GICR_BASE + 0x0070UL)
#define GICR_PENDBASER (GICR_BASE + 0x0078UL)
#define GICR_VPROPBASER (GICR_BASE + 0x20070UL)
#define GICR_VPENDBASER (GICR_BASE + 0x20078UL)

#define GITS_CTLR      (GITS_BASE + 0x0000UL)
#define GITS_TYPER     (GITS_BASE + 0x0008UL)
#define GITS_CBASER    (GITS_BASE + 0x0080UL)
#define GITS_CWRITER   (GITS_BASE + 0x0088UL)
#define GITS_BASER0    (GITS_BASE + 0x0100UL)
#define GITS_BASER1    (GITS_BASE + 0x0108UL)
#define GITS_BASER2    (GITS_BASE + 0x0110UL)
#define GITS_TRANSLATER (GITS_BASE + 0x10040UL)

#define GICR_CTLR_ENABLE_LPIS (1u << 0)
#define GICR_WAKER_ProcessorSleep (1u << 1)
#define GICR_WAKER_ChildrenAsleep (1u << 2)
#define GICD_CTLR_ENABLE_GRP1NS (1u << 1)

#define ICC_PMR_EL1_INIT 0xffu

#define DEVICE_ID 1u
#define EVENT_ID  8192u
#define VPE_ID    0u
#define PINTID    8192u

#define LPI_IDBITS 13u

#define CMD_MAPD   0x08u
#define CMD_VMAPP  0x29u
#define CMD_VMAPTI 0x2Au
#define CMD_SYNC   0x05u

#define TABLE_BASE           0x90000000UL
#define CMDQ_BASE            0x90004000UL
#define DEVICE_TABLE_BASE    0x90008000UL
#define VPE_TABLE_BASE       0x9000C000UL
#define COLLECTION_TABLE_BASE 0x90010000UL
#define LPI_PROP_BASE        0x90014000UL
#define LPI_PEND_BASE        0x90018000UL

struct its_cmd {
    uint64_t raw[4];
};

static volatile uint64_t t_enter = 0;
static volatile uint64_t t_irq = 0;
static volatile uint32_t last_intid = 0;

static inline void mmio_write32(uint64_t addr, uint32_t val)
{
    *(volatile uint32_t *)addr = val;
}

static inline void mmio_write64(uint64_t addr, uint64_t val)
{
    *(volatile uint64_t *)addr = val;
}

static inline uint32_t mmio_read32(uint64_t addr)
{
    return *(volatile uint32_t *)addr;
}

static inline uint64_t mmio_read64(uint64_t addr)
{
    return *(volatile uint64_t *)addr;
}

static inline uint64_t read_cntpct(void)
{
    uint64_t v;
    asm volatile("mrs %0, cntpct_el0" : "=r"(v));
    return v;
}

static inline uint32_t gic_read_iar1(void)
{
    uint64_t v;
    asm volatile("mrs %0, icc_iar1_el1" : "=r"(v));
    return (uint32_t)v;
}

static inline void gic_write_eoir1(uint32_t intid)
{
    asm volatile("msr icc_eoir1_el1, %0" : : "r"((uint64_t)intid));
    asm volatile("isb");
}

static inline void dsb_sy(void)
{
    asm volatile("dsb sy" ::: "memory");
}

static inline void isb(void)
{
    asm volatile("isb");
}

static inline void uart_putc(char c)
{
    *(volatile uint32_t *)(UART_BASE) = (uint32_t)c;
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void uart_put_u64(uint64_t x)
{
    char buf[32];
    int i = 0;

    if (x == 0) {
        uart_putc('0');
        return;
    }

    while (x && i < (int)sizeof(buf)) {
        buf[i++] = '0' + (x % 10);
        x /= 10;
    }

    while (i--)
        uart_putc(buf[i]);
}

static void memzero(void *ptr, uint64_t n)
{
    volatile uint8_t *p = (volatile uint8_t *)ptr;
    while (n--)
        *p++ = 0;
}

static void gic_cpuif_enable(void)
{
    uint64_t x;

    asm volatile("mrs %0, icc_sre_el1" : "=r"(x));
    x |= 1;
    asm volatile("msr icc_sre_el1, %0" : : "r"(x));
    isb();

    asm volatile("msr icc_pmr_el1, %0" : : "r"((uint64_t)ICC_PMR_EL1_INIT));
    asm volatile("msr icc_igrpen1_el1, %0" : : "r"(1ULL));
    isb();
}

static void gic_enable_lpis(void)
{
    uint32_t waker = mmio_read32(GICR_WAKER);

    waker &= ~GICR_WAKER_ProcessorSleep;
    mmio_write32(GICR_WAKER, waker);
    while (mmio_read32(GICR_WAKER) & GICR_WAKER_ChildrenAsleep)
        ;

    mmio_write32(GICD_CTLR, GICD_CTLR_ENABLE_GRP1NS);

    uint64_t prop = (LPI_PROP_BASE & 0xFFFFFFFFFF000ULL) | LPI_IDBITS;
    uint64_t pend = (LPI_PEND_BASE & 0xFFFFFFFFF0000ULL);

    // Keep vLPI and LPI pending storage aligned for this minimal test.
    mmio_write64(GICR_PROPBASER, prop);
    mmio_write64(GICR_PENDBASER, pend);
    mmio_write64(GICR_VPROPBASER, prop);
    mmio_write64(GICR_VPENDBASER, pend);

    mmio_write32(GICR_CTLR, GICR_CTLR_ENABLE_LPIS);
}

static uint64_t its_baser_value(uint64_t phys, uint8_t size)
{
    uint64_t v = 0;
    v |= (1ULL << 63);
    v |= ((phys >> 12) & ((1ULL << 36) - 1)) << 12;
    v |= size;
    return v;
}

static void emit_cmd(struct its_cmd *q, uint32_t idx, uint32_t type,
                     uint32_t dev, uint32_t event, uint32_t pint,
                     uint64_t d2, uint64_t d3)
{
    q[idx].raw[0] = ((uint64_t)dev << 32) | type;
    q[idx].raw[1] = ((uint64_t)pint << 32) | event;
    q[idx].raw[2] = d2;
    q[idx].raw[3] = d3;
}

static void its_program_tables_and_map(void)
{
    struct its_cmd *cmdq = (struct its_cmd *)CMDQ_BASE;

    memzero((void *)TABLE_BASE, 0x30000);

    // Enable one vLPI config entry (index = INTID - 8192).
    ((volatile uint8_t *)LPI_PROP_BASE)[EVENT_ID - 8192] = 0x03;

    mmio_write64(GITS_BASER0, its_baser_value(DEVICE_TABLE_BASE, 0));
    mmio_write64(GITS_BASER1, its_baser_value(VPE_TABLE_BASE, 0));
    mmio_write64(GITS_BASER2, its_baser_value(COLLECTION_TABLE_BASE, 0));

    mmio_write64(GITS_CBASER, (1ULL << 63) | ((CMDQ_BASE >> 12) << 12) | 0);
    mmio_write32(GITS_CTLR, 1);

    uint64_t typer = mmio_read64(GITS_TYPER);
    uint64_t pta = (typer >> 19) & 1ULL;
    uint64_t rd_base_field = pta ? (GICR_BASE >> 16) : 0;

    emit_cmd(cmdq, 0, CMD_MAPD, DEVICE_ID, 15, 0,
             (1ULL << 63) |
                 (((uint64_t)DEVICE_TABLE_BASE + 0x1000) &
                  0x000FFFFFFFFFFF00ULL),
             0);

    emit_cmd(cmdq, 1, CMD_VMAPP, 0, 0, 0,
             (1ULL << 63) | (rd_base_field << 16) | VPE_ID, 0);

    emit_cmd(cmdq, 2, CMD_VMAPTI, DEVICE_ID, EVENT_ID, PINTID, VPE_ID, 0);

    emit_cmd(cmdq, 3, CMD_SYNC, DEVICE_ID, 0, 0, (rd_base_field << 16), 0);

    dsb_sy();
    mmio_write64(GITS_CWRITER, (uint64_t)(4u << 5));
}

void el1_irq_handler(void)
{
    uint32_t intid = gic_read_iar1();
    t_irq = read_cntpct();
    last_intid = intid;

    uart_puts("vLPI IRQ INTID=");
    uart_put_u64(intid);
    uart_puts("\nlatency=");
    uart_put_u64(t_irq - t_enter);
    uart_puts("\n");

    gic_write_eoir1(intid);
    uart_puts("EOI done\n");
}

int main(void)
{
    uart_puts("[vlpi-baremetal-closed-loop] start\n");

    gic_cpuif_enable();
    gic_enable_lpis();
    its_program_tables_and_map();

    t_enter = read_cntpct();
    mmio_write32(GITS_TRANSLATER, EVENT_ID);

    while (last_intid == 0)
        asm volatile("wfi");

    uart_puts("[vlpi-baremetal-closed-loop] done\n");
    for (;;)
        asm volatile("wfi");
}
