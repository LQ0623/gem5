/*
 * Copyright (c) 2019-2022 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2018 Metempsy Technology Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dev/arm/gic_v3_redistributor.hh"
#include <algorithm>

#include "arch/arm/isa.hh"
#include "arch/arm/utility.hh"
#include "base/compiler.hh"
#include "base/logging.hh"
#include "debug/GIC.hh"
#include "dev/arm/gic_v3_cpu_interface.hh"
#include "dev/arm/gic_v3_distributor.hh"
#include "dev/arm/gic_v3_its.hh"

namespace gem5
{

using namespace ArmISA;

const AddrRange Gicv3Redistributor::GICR_IPRIORITYR(SGI_base + 0x0400,
                                                    SGI_base + 0x0420);

Gicv3Redistributor::Gicv3Redistributor(Gicv3 * gic, uint32_t cpu_id)
    : gic(gic),
      distributor(nullptr),
      cpuInterface(nullptr),
      cpuId(cpu_id),
      memProxy(nullptr),
      peInLowPowerState(true),
      irqGroup(Gicv3::SGI_MAX + Gicv3::PPI_MAX, 0),
      irqEnabled(Gicv3::SGI_MAX + Gicv3::PPI_MAX, false),
      irqPending(Gicv3::SGI_MAX + Gicv3::PPI_MAX, false),
      irqPendingIspendr(Gicv3::SGI_MAX + Gicv3::PPI_MAX, false),
      irqActive(Gicv3::SGI_MAX + Gicv3::PPI_MAX, false),
      irqPriority(Gicv3::SGI_MAX + Gicv3::PPI_MAX, 0),
      irqConfig(Gicv3::SGI_MAX + Gicv3::PPI_MAX, Gicv3::INT_EDGE_TRIGGERED),
      irqGrpmod(Gicv3::SGI_MAX + Gicv3::PPI_MAX, 0),
      irqNsacr(Gicv3::SGI_MAX + Gicv3::PPI_MAX, 0),
      DPG1S(false),
      DPG1NS(false),
      DPG0(false),
      EnableLPIs(false),
      lpiConfigurationTablePtr(0),
      lpiIDBits(0),
      lpiPendingTablePtr(0),
      vLpiConfigurationTablePtr(0),
      vLpiIDBits(0),
      vLpiPendingTablePtr(0),
      vLpiPendingTableValid(false),
      vLpiPendingTableDirty(false),
      vLpiPendingLast(false),
      residentVpeId(0xffff),
      residentVptAddr(0),
      lpiSyncBusyReads(0),
      addrRangeSize(gic->params().gicv4 ? 0x40000 : 0x20000)
{
}

void
Gicv3Redistributor::init()
{
    distributor = gic->getDistributor();
    cpuInterface = gic->getCPUInterface(cpuId);

    memProxy = &gic->getSystem()->physProxy;
}

uint64_t
Gicv3Redistributor::read(Addr addr, size_t size, bool is_secure_access)
{
    if (GICR_IPRIORITYR.contains(addr)) { // Interrupt Priority Registers
        uint64_t value = 0;
        int first_intid = addr - GICR_IPRIORITYR.start();

        for (int i = 0, int_id = first_intid; i < size; i++, int_id++) {
            uint8_t prio = irqPriority[int_id];

            if (!distributor->DS && !is_secure_access) {
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    // RAZ/WI for non-secure accesses for secure interrupts
                    continue;
                } else {
                    // NS view
                    prio = (prio << 1) & 0xff;
                }
            }

            value |= prio << (i * 8);
        }

        return value;
    }

    switch (addr) {
      case GICR_CTLR: { // Control Register
          uint64_t value = 0;

          if (DPG1S) {
              value |= GICR_CTLR_DPG1S;
          }

          if (DPG1NS) {
              value |= GICR_CTLR_DPG1NS;
          }

          if (DPG0) {
              value |= GICR_CTLR_DPG0;
          }

          if (EnableLPIs) {
              value |= GICR_CTLR_ENABLE_LPIS;
          }

          return value;
      }

      case GICR_IIDR: // Implementer Identification Register
        //return 0x43b; // r0p0 GIC-500
        return 0;

      case GICR_TYPER: { // Type Register
          /*
           * Affinity_Value   [63:32] == X
           * (The identity of the PE associated with this Redistributor)
           * CommonLPIAff     [25:24] == 01
           * (All Redistributors with the same Aff3 value must share an
           * LPI Configuration table)
           * Processor_Number [23:8]  == X
           * (A unique identifier for the PE)
           * DPGS             [5]     == 1
           * (GICR_CTLR.DPG* bits are supported)
           * Last             [4]     == X
           * (This Redistributor is the highest-numbered Redistributor in
           * a series of contiguous Redistributor pages)
           * DirectLPI        [3]     == 1
           * (direct injection of LPIs supported)
           * VLPIS            [1]     == 1
           * (virtual LPIs supported)
           * PLPIS            [0]     == 1
           * (physical LPIs supported)
           */
          uint64_t affinity = getAffinity();
          int last = cpuId == (gic->getSystem()->threads.size() - 1);
          const uint64_t vlpis = gic->params().gicv4 ? (1 << 1) : 0;
          const uint64_t directLpi = gic->params().gicv4 ? (1 << 3) : 0;
          return (affinity << 32) | (1 << 24) | (cpuId << 8) |
              (1 << 5) | (last << 4) | directLpi | vlpis | (1 << 0);
      }

      case GICR_WAKER: // Wake Register
        if (!distributor->DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return 0;
        }

        if (peInLowPowerState) {
            return GICR_WAKER_ChildrenAsleep | GICR_WAKER_ProcessorSleep;
        } else {
            return 0;
        }

      case GICR_PIDR0: { // Peripheral ID0 Register
          return 0x92; // Part number, bits[7:0]
      }

      case GICR_PIDR1: { // Peripheral ID1 Register
          uint8_t des_0 = 0xB; // JEP106 identification code, bits[3:0]
          uint8_t part_1 = 0x4; // Part number, bits[11:8]
          return (des_0 << 4) | (part_1 << 0);
      }

      case GICR_PIDR2: { // Peripheral ID2 Register
          return gic->getDistributor()->gicdPidr2;
      }

      case GICR_PIDR3: // Peripheral ID3 Register
        return 0x0; // Implementation defined

      case GICR_PIDR4: { // Peripheral ID4 Register
          uint8_t size = 0x4; // 64 KB software visible page
          uint8_t des_2 = 0x4; // ARM implementation
          return (size << 4) | (des_2 << 0);
      }

      case GICR_PIDR5: // Peripheral ID5 Register
      case GICR_PIDR6: // Peripheral ID6 Register
      case GICR_PIDR7: // Peripheral ID7 Register
        return 0; // RES0

      case GICR_IGROUPR0: { // Interrupt Group Register 0
          uint64_t value = 0;

          if (!distributor->DS && !is_secure_access) {
              // RAZ/WI for non-secure accesses
              return 0;
          }

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              value |= (irqGroup[int_id] << int_id);
          }

          return value;
      }

      case GICR_ISENABLER0: // Interrupt Set-Enable Register 0
      case GICR_ICENABLER0: { // Interrupt Clear-Enable Register 0
          uint64_t value = 0;

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              if (irqEnabled[int_id]) {
                  value |= (1 << int_id);
              }
          }

          return value;
      }

      case GICR_ISPENDR0: // Interrupt Set-Pending Register 0
      case GICR_ICPENDR0: { // Interrupt Clear-Pending Register 0
          uint64_t value = 0;

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              value |= (irqPending[int_id] << int_id);
          }

          return value;
      }

      case GICR_ISACTIVER0: // Interrupt Set-Active Register 0
      case GICR_ICACTIVER0: { // Interrupt Clear-Active Register 0
          uint64_t value = 0;

          for (int int_id = 0; int_id < 8 * size; int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              value |=  irqActive[int_id] << int_id;
          }

          return value;
      }

      case GICR_ICFGR0: // SGI Configuration Register
      case GICR_ICFGR1: { // PPI Configuration Register
          uint64_t value = 0;
          uint32_t first_int_id = addr == GICR_ICFGR0 ? 0 : Gicv3::SGI_MAX;

          for (int i = 0, int_id = first_int_id; i < 32;
               i = i + 2, int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              if (irqConfig[int_id] == Gicv3::INT_EDGE_TRIGGERED) {
                  value |= (0x2) << i;
              }
          }

          return value;
      }

      case GICR_IGRPMODR0: { // Interrupt Group Modifier Register 0
          uint64_t value = 0;

          if (distributor->DS) {
              value = 0;
          } else {
              if (!is_secure_access) {
                  // RAZ/WI for non-secure accesses
                  value = 0;
              } else {
                  for (int int_id = 0; int_id < 8 * size; int_id++) {
                      value |= irqGrpmod[int_id] << int_id;
                  }
              }
          }

          return value;
      }

      case GICR_NSACR: { // Non-secure Access Control Register
          uint64_t value = 0;

          if (distributor->DS) {
              // RAZ/WI
              value = 0;
          } else {
              if (!is_secure_access) {
                  // RAZ/WI
                  value = 0;
              } else {
                  for (int i = 0, int_id = 0; i < 8 * size;
                       i = i + 2, int_id++) {
                      value |= irqNsacr[int_id] << i;
                  }
              }
          }

          return value;
      }

      case GICR_PROPBASER: // Redistributor Properties Base Address Register
        // OuterCache, bits [58:56]
        //   000 Memory type defined in InnerCache field
        // Physical_Address, bits [51:12]
        //   Bits [51:12] of the physical address containing the LPI
        //   Configuration table
        // Shareability, bits [11:10]
        //   00 Non-shareable
        // InnerCache, bits [9:7]
        //   000 Device-nGnRnE
        // IDbits, bits [4:0]
        //   limited by GICD_TYPER.IDbits
        return lpiConfigurationTablePtr | lpiIDBits;

      // Redistributor LPI Pending Table Base Address Register
      case GICR_PENDBASER:
        // PTZ, bit [62]
        //   Pending Table Zero
        // OuterCache, bits [58:56]
        //   000 Memory type defined in InnerCache field
        // Physical_Address, bits [51:16]
        //   Bits [51:16] of the physical address containing the LPI Pending
        //   table
        // Shareability, bits [11:10]
        //   00 Non-shareable
        // InnerCache, bits [9:7]
        //   000 Device-nGnRnE
        return lpiPendingTablePtr;

      // Redistributor Synchronize Register
      case GICR_SYNCR:
        // 最小 Busy 生命周期：失效命令后返回一次 Busy=1，再恢复 0。
        if (lpiSyncBusyReads) {
            lpiSyncBusyReads--;
            return 1;
        }
        return 0;

      case GICR_VPROPBASER:
        return vLpiConfigurationTablePtr | vLpiIDBits;

      case GICR_VPENDBASER:
        return vpendbaserReadValue();

      default:
        gic->reserved("Gicv3Redistributor::read(): invalid offset %#x\n", addr);
        return 0; // RES0
    }
}

void
Gicv3Redistributor::write(Addr addr, uint64_t data, size_t size,
                          bool is_secure_access)
{
    if (GICR_IPRIORITYR.contains(addr)) { // Interrupt Priority Registers
        int first_intid = addr - GICR_IPRIORITYR.start();

        for (int i = 0, int_id = first_intid; i < size; i++, int_id++) {
            uint8_t prio = bits(data, (i + 1) * 8 - 1, (i * 8));

            if (!distributor->DS && !is_secure_access) {
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    // RAZ/WI for non-secure accesses for secure interrupts
                    continue;
                } else {
                    // NS view
                    prio = 0x80 | (prio >> 1);
                }
            }

            irqPriority[int_id] = prio;
            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d priority %d\n", int_id, irqPriority[int_id]);
        }

        return;
    }

    switch (addr) {
      case GICR_CTLR: {
          // GICR_TYPER.LPIS is 0 so EnableLPIs is RES0
          EnableLPIs = data & GICR_CTLR_ENABLE_LPIS;
          DPG1S = data & GICR_CTLR_DPG1S;
          DPG1NS = data & GICR_CTLR_DPG1NS;
          DPG0 = data & GICR_CTLR_DPG0;
          break;
      }

      case GICR_WAKER: // Wake Register
      {
        if (!distributor->DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return;
        }

        bool pe_was_low_power = peInLowPowerState;
        peInLowPowerState = data & GICR_WAKER_ProcessorSleep;
        if (!pe_was_low_power && peInLowPowerState) {
            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "PE entering in low power state\n");
            updateDistributor();
        } else if (pe_was_low_power && !peInLowPowerState) {
            DPRINTF(GIC, "Gicv3Redistributor::write(): powering up PE\n");
            cpuInterface->deassertWakeRequest();
            updateDistributor();
        }
        break;
      }

      case GICR_IGROUPR0: // Interrupt Group Register 0
        if (!distributor->DS && !is_secure_access) {
            // RAZ/WI for non-secure accesses
            return;
        }

        for (int int_id = 0; int_id < 8 * size; int_id++) {
            irqGroup[int_id] = data & (1 << int_id) ? 1 : 0;
            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d group %d\n", int_id, irqGroup[int_id]);
        }

        break;

      case GICR_ISENABLER0: // Interrupt Set-Enable Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool enable = data & (1 << int_id) ? 1 : 0;

            if (enable) {
                irqEnabled[int_id] = true;
            }

            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d enable %i\n", int_id, irqEnabled[int_id]);
        }

        break;

      case GICR_ICENABLER0: // Interrupt Clear-Enable Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool disable = data & (1 << int_id) ? 1 : 0;

            if (disable) {
                irqEnabled[int_id] = false;
            }

            DPRINTF(GIC, "Gicv3Redistributor::write(): "
                    "int_id %d enable %i\n", int_id, irqEnabled[int_id]);
        }

        break;

      case GICR_ISPENDR0: // Interrupt Set-Pending Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool pending = data & (1 << int_id) ? 1 : 0;

            if (pending) {
                DPRINTF(GIC, "Gicv3Redistributor::write() "
                        "(GICR_ISPENDR0): int_id %d (PPI) "
                        "pending bit set\n", int_id);
                irqPending[int_id] = true;
                irqPendingIspendr[int_id] = true;
            }
        }

        updateDistributor();
        break;

      case GICR_ICPENDR0:// Interrupt Clear-Pending Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool clear = data & (1 << int_id) ? 1 : 0;

            if (clear && treatAsEdgeTriggered(int_id)) {
                irqPending[int_id] = false;
            }
        }

        break;

      case GICR_ISACTIVER0: // Interrupt Set-Active Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool activate = data & (1 << int_id) ? 1 : 0;

            if (activate) {
                if (!irqActive[int_id]) {
                    DPRINTF(GIC, "Gicv3Redistributor::write(): "
                            "int_id %d active set\n", int_id);
                }

                irqActive[int_id] = true;
            }
        }

        break;

      case GICR_ICACTIVER0: // Interrupt Clear-Active Register 0
        for (int int_id = 0; int_id < 8 * size; int_id++) {
            if (!distributor->DS && !is_secure_access) {
                // RAZ/WI for non-secure accesses for secure interrupts
                if (getIntGroup(int_id) != Gicv3::G1NS) {
                    continue;
                }
            }

            bool clear = data & (1 << int_id) ? 1 : 0;

            if (clear) {
                if (irqActive[int_id]) {
                    DPRINTF(GIC, "Gicv3Redistributor::write(): "
                            "int_id %d active cleared\n", int_id);
                }

                irqActive[int_id] = false;
            }
        }

        break;

      case GICR_ICFGR0: // SGI Configuration Register
        // WI
        return;
      case GICR_ICFGR1: { // PPI Configuration Register
          int first_intid = Gicv3::SGI_MAX;

          for (int i = 0, int_id = first_intid; i < 8 * size;
               i = i + 2, int_id++) {
              if (!distributor->DS && !is_secure_access) {
                  // RAZ/WI for non-secure accesses for secure interrupts
                  if (getIntGroup(int_id) != Gicv3::G1NS) {
                      continue;
                  }
              }

              irqConfig[int_id] = data & (0x2 << i) ?
                                  Gicv3::INT_EDGE_TRIGGERED :
                                  Gicv3::INT_LEVEL_SENSITIVE;
              DPRINTF(GIC, "Gicv3Redistributor::write(): "
                      "int_id %d (PPI) config %d\n",
                      int_id, irqConfig[int_id]);
          }

          break;
      }

      case GICR_IGRPMODR0: { // Interrupt Group Modifier Register 0
          if (distributor->DS) {
              // RAZ/WI if secutiry disabled
          } else {
              for (int int_id = 0; int_id < 8 * size; int_id++) {
                  if (!is_secure_access) {
                      // RAZ/WI for non-secure accesses
                      continue;
                  }

                  irqGrpmod[int_id] = bits(data, int_id);
              }
          }

          break;
      }

      case GICR_NSACR: { // Non-secure Access Control Register
          if (distributor->DS) {
              // RAZ/WI
          } else {
              if (!is_secure_access) {
                  // RAZ/WI
              } else {
                  for (int i = 0, int_id = 0; i < 8 * size;
                       i = i + 2, int_id++) {
                      irqNsacr[int_id] = (data >> i) & 0x3;
                  }
              }
          }

          break;
      }

      case GICR_SETLPIR: // Set LPI Pending Register
        setClrLPI(data, true);
        break;

      case GICR_CLRLPIR: // Clear LPI Pending Register
        setClrLPI(data, false);
        break;

      case GICR_PROPBASER: { // Redistributor Properties Base Address Register
          // OuterCache, bits [58:56]
          //   000 Memory type defined in InnerCache field
          // Physical_Address, bits [51:12]
          //   Bits [51:12] of the physical address containing the LPI
          //   Configuration table
          // Shareability, bits [11:10]
          //   00 Non-shareable
          // InnerCache, bits [9:7]
          //   000 Device-nGnRnE
          // IDbits, bits [4:0]
          //   limited by GICD_TYPER.IDbits (= 0xf)
          lpiConfigurationTablePtr = data & 0xFFFFFFFFFF000;
          lpiIDBits = data & 0x1f;

          // 0xf here matches the value of GICD_TYPER.IDbits.
          // TODO - make GICD_TYPER.IDbits a parameter instead of a hardcoded
          // value
          if (lpiIDBits > 0xf) {
              lpiIDBits = 0xf;
          }

          break;
      }

      // Redistributor LPI Pending Table Base Address Register
      case GICR_PENDBASER:
        // PTZ, bit [62]
        //   Pending Table Zero
        // OuterCache, bits [58:56]
        //   000 Memory type defined in InnerCache field
        // Physical_Address, bits [51:16]
        //   Bits [51:16] of the physical address containing the LPI Pending
        //   table
        // Shareability, bits [11:10]
        //   00 Non-shareable
        // InnerCache, bits [9:7]
        //   000 Device-nGnRnE
        lpiPendingTablePtr = data & 0xFFFFFFFFF0000;
        break;

      case GICR_VPROPBASER: {
          vLpiConfigurationTablePtr = data & 0xFFFFFFFFFF000;
          vLpiIDBits = data & 0x1f;
          if (vLpiIDBits > 0xf) {
              vLpiIDBits = 0xf;
          }
          /*
           * Minimal cache model:
           * VPROPBASER update changes backing table, so drop all cached vLPI
           * config entries and require fresh fetch after explicit sync points.
           */
          vLpiConfigCache.clear();
          if (gic->getIts()) {
              gic->getIts()->syncPendingVirtualLpis(this);
          }
          break;
      }

      case GICR_VPENDBASER: {
        const bool oldValid = vLpiPendingTableValid;
        const bool newValid = bits(data, 63);

        if (oldValid && newValid) {
            const uint64_t oldNonValid = vpendbaserReadValue() &
                ~GICR_VPENDBASER_VALID;
            const uint64_t newNonValid = data & ~GICR_VPENDBASER_VALID;
            if (oldNonValid != newNonValid) {
                /*
                 * 架构上 Valid=1 时修改其余字段受限。
                 * 当前实现采用“warn + ignore”，避免 silent state corruption。
                 */
                warn("GICR_VPENDBASER cpu=%u write while Valid=1 "
                     "changed non-Valid fields; ignored\n", cpuId);
            }
            break;
        }

        if (oldValid && !newValid) {
            scheduleVpeOff();
            break;
        }

        if (!oldValid && newValid) {
            scheduleVpeOn(data);
            break;
        }

        /*
         * Valid == 0 -> no vPE scheduled.
         * Software may prepare a future schedule-on value while idle.
         */
        vLpiPendingTablePtr = data & 0xFFFFFFFFF0000ULL;
        residentVptAddr = 0;
        residentVpeId = 0xffff;
        break;
      }

      case GICR_INVLPIR: { // Redistributor Invalidate LPI Register
          // 对单个 vINTID 失效，并触发 SYNCR Busy 脉冲。
          invalidateVLPIConfigOneImpl(data & 0xffffffff, true);
          break;
      }

      case GICR_INVALLR: { // Redistributor Invalidate All Register
          // 全量失效，并触发 SYNCR Busy 脉冲。
          invalidateVLPIConfigAllImpl(true);
          break;
      }

      default:
        gic->reserved("Gicv3Redistributor::write(): invalid offset %#x\n", addr);
        break;
    }
}

void
Gicv3Redistributor::sendPPInt(uint32_t int_id)
{
    assert((int_id >= Gicv3::SGI_MAX) &&
           (int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX));
    irqPending[int_id] = true;
    irqPendingIspendr[int_id] = false;
    DPRINTF(GIC, "Gicv3Redistributor::sendPPInt(): "
            "int_id %d (PPI) pending bit set\n", int_id);
    updateDistributor();
}

void
Gicv3Redistributor::clearPPInt(uint32_t int_id)
{
    assert((int_id >= Gicv3::SGI_MAX) &&
           (int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX));

    if (isLevelSensitive(int_id)) {
        irqPending[int_id] = false;
    }
}

void
Gicv3Redistributor::sendSGI(uint32_t int_id, Gicv3::GroupId group, bool ns)
{
    assert(int_id < Gicv3::SGI_MAX);
    Gicv3::GroupId int_group = getIntGroup(int_id);

    bool forward = false;

    if (ns) {
        // Non-Secure EL1 and EL2 access
        int nsaccess = irqNsacr[int_id];
        if (int_group == Gicv3::G0S) {

            forward = distributor->DS || (nsaccess >= 1);

        } else if (int_group == Gicv3::G1S) {
            forward = ((group == Gicv3::G1S || group == Gicv3::G1NS ) &&
                      nsaccess == 2);
        } else {
            // G1NS
            forward = group == Gicv3::G1NS;
        }
    } else {
        // Secure EL1 and EL3 access
        forward = (group == int_group) ||
            (group == Gicv3::G1S && int_group == Gicv3::G0S &&
            distributor->DS);
    }

    if (!forward) return;

    irqPending[int_id] = true;
    irqPendingIspendr[int_id] = false;
    DPRINTF(GIC, "Gicv3ReDistributor::sendSGI(): "
            "int_id %d (SGI) pending bit set\n", int_id);
    updateDistributor();
}

Gicv3::IntStatus
Gicv3Redistributor::intStatus(uint32_t int_id) const
{
    assert(int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX);

    if (irqPending[int_id]) {
        if (irqActive[int_id]) {
            return Gicv3::INT_ACTIVE_PENDING;
        }

        return Gicv3::INT_PENDING;
    } else if (irqActive[int_id]) {
        return Gicv3::INT_ACTIVE;
    } else {
        return Gicv3::INT_INACTIVE;
    }
}

void
Gicv3Redistributor::updateDistributor()
{
    distributor->update();
}

/*
 * Recalculate the highest priority pending interrupt after a
 * change to redistributor state.
 */
void
Gicv3Redistributor::update()
{
    if (gic->blockIntUpdate())
        return;

    for (int int_id = 0; int_id < Gicv3::SGI_MAX + Gicv3::PPI_MAX; int_id++) {
        Gicv3::GroupId int_group = getIntGroup(int_id);
        bool group_enabled = distributor->groupEnabled(int_group);

        if (irqPending[int_id] && irqEnabled[int_id] &&
                !irqActive[int_id] && group_enabled) {
            if ((irqPriority[int_id] < cpuInterface->hppi.prio) ||
                /*
                 * Multiple pending ints with same priority.
                 * Implementation choice which one to signal.
                 * Our implementation selects the one with the lower id.
                 */
                (irqPriority[int_id] == cpuInterface->hppi.prio &&
                 int_id < cpuInterface->hppi.intid)) {
                cpuInterface->hppi.intid = int_id;
                cpuInterface->hppi.prio = irqPriority[int_id];
                cpuInterface->hppi.group = int_group;
            }
        }
    }

    // Check LPIs
    if (EnableLPIs) {

        const uint32_t largest_lpi_id = 1 << (lpiIDBits + 1);
        const uint32_t number_lpis = largest_lpi_id - SMALLEST_LPI_ID + 1;
        const size_t table_size = largest_lpi_id / 8;
        auto lpi_pending_table = std::make_unique<uint8_t[]>(table_size);
        auto lpi_config_table = std::make_unique<uint8_t[]>(number_lpis);

        memProxy->readBlob(lpiPendingTablePtr,
                           lpi_pending_table.get(),
                           table_size);

        memProxy->readBlob(lpiConfigurationTablePtr,
                           lpi_config_table.get(),
                           number_lpis);

        for (int lpi_id = SMALLEST_LPI_ID; lpi_id < largest_lpi_id;
             lpi_id++) {
            uint32_t lpi_pending_entry_byte = lpi_id / 8;
            uint8_t lpi_pending_entry_bit_position = lpi_id % 8;
            bool lpi_is_pending = lpi_pending_table[lpi_pending_entry_byte] &
                                  1 << lpi_pending_entry_bit_position;
            uint32_t lpi_configuration_entry_index = lpi_id - SMALLEST_LPI_ID;

            LPIConfigurationTableEntry config_entry =
                lpi_config_table[lpi_configuration_entry_index];

            bool lpi_is_enable = config_entry.enable;

            // LPIs are always Non-secure Group 1 interrupts,
            // in a system where two Security states are enabled.
            Gicv3::GroupId lpi_group = Gicv3::G1NS;
            bool group_enabled = distributor->groupEnabled(lpi_group);

            if (lpi_is_pending && lpi_is_enable && group_enabled) {
                uint8_t lpi_priority = config_entry.priority << 2;

                if ((lpi_priority < cpuInterface->hppi.prio) ||
                    (lpi_priority == cpuInterface->hppi.prio &&
                     lpi_id < cpuInterface->hppi.intid)) {
                    cpuInterface->hppi.intid = lpi_id;
                    cpuInterface->hppi.prio = lpi_priority;
                    cpuInterface->hppi.group = lpi_group;
                }
            }
        }
    }

    if (peInLowPowerState) {
        if (cpuInterface->havePendingInterrupts()) {
            cpuInterface->assertWakeRequest();
            cpuInterface->clearPendingInterrupts();
        }
    } else {
        cpuInterface->update();
    }
}

uint8_t
Gicv3Redistributor::readEntryLPI(uint32_t lpi_id)
{
    Addr lpi_pending_entry_ptr = lpiPendingTablePtr + (lpi_id / 8);

    uint8_t lpi_pending_entry;
    memProxy->readBlob(lpi_pending_entry_ptr,
                       &lpi_pending_entry,
                       sizeof(lpi_pending_entry));

    return lpi_pending_entry;
}

void
Gicv3Redistributor::writeEntryLPI(uint32_t lpi_id, uint8_t lpi_pending_entry)
{
    Addr lpi_pending_entry_ptr = lpiPendingTablePtr + (lpi_id / 8);

    memProxy->writeBlob(lpi_pending_entry_ptr,
                        &lpi_pending_entry,
                        sizeof(lpi_pending_entry));
}

bool
Gicv3Redistributor::isPendingLPI(uint32_t lpi_id)
{
    // Fetch the LPI pending entry from memory
    uint8_t lpi_pending_entry = readEntryLPI(lpi_id);

    uint8_t lpi_pending_entry_bit_position = lpi_id % 8;
    bool is_set = lpi_pending_entry & (1 << lpi_pending_entry_bit_position);

    return is_set;
}

void
Gicv3Redistributor::setClrLPI(uint64_t data, bool set)
{
    if (!EnableLPIs) {
        // Writes to GICR_SETLPIR or GICR_CLRLPIR have not effect if
        // GICR_CTLR.EnableLPIs == 0.
        return;
    }

    uint32_t lpi_id = data & 0xffffffff;
    uint32_t largest_lpi_id = 1 << (lpiIDBits + 1);

    if (lpi_id > largest_lpi_id) {
        // Writes to GICR_SETLPIR or GICR_CLRLPIR have not effect if
        // pINTID value specifies an unimplemented LPI.
        return;
    }

    // Fetch the LPI pending entry from memory
    uint8_t lpi_pending_entry = readEntryLPI(lpi_id);

    uint8_t lpi_pending_entry_bit_position = lpi_id % 8;
    bool is_set = lpi_pending_entry & (1 << lpi_pending_entry_bit_position);

    if (set) {
        if (is_set) {
            // Writes to GICR_SETLPIR have not effect if the pINTID field
            // corresponds to an LPI that is already pending.
            return;
        }

        lpi_pending_entry |= 1 << (lpi_pending_entry_bit_position);
    } else {
        if (!is_set) {
            // Writes to GICR_SETLPIR have not effect if the pINTID field
            // corresponds to an LPI that is not pending.
            return;
        }

        lpi_pending_entry &= ~(1 << (lpi_pending_entry_bit_position));

        // Remove the pending state from the cpu interface
        cpuInterface->resetHppi(lpi_id);
    }

    writeEntryLPI(lpi_id, lpi_pending_entry);

    updateDistributor();
}

uint64_t
Gicv3Redistributor::vpendbaserReadValue() const
{
    // 把内部状态回编码为 VPENDBASER，可被测试直接观测。
    uint64_t value = vLpiPendingTablePtr;
    if (vLpiPendingTableDirty) {
        value |= GICR_VPENDBASER_DIRTY;
    }
    if (vLpiPendingLast) {
        value |= GICR_VPENDBASER_PENDING_LAST;
    }
    if (vLpiPendingTableValid) {
        value |= GICR_VPENDBASER_VALID;
    }
    return value;
}

bool
Gicv3Redistributor::residentLrHasPendingVintid(uint32_t vintId) const
{
    if (!vLpiPendingTableValid || residentVptAddr == 0 ||
        vintId < SMALLEST_LPI_ID) {
        return false;
    }

    // 扫描 LR，识别“只在 LR 中存在的 pending 分量”。
    for (int lrIdx = 0; lrIdx < Gicv3CPUInterface::VIRTUAL_NUM_LIST_REGS;
         lrIdx++) {
        const uint64_t lrRaw =
            cpuInterface->isa->readMiscRegNoEffect(
                MISCREG_ICH_LR0_EL2 + lrIdx);
        const uint64_t state = bits(lrRaw, 63, 62);
        const uint32_t lrVintId = bits(lrRaw, 31, 0);
        if ((state == Gicv3CPUInterface::ICH_LR_EL2_STATE_PENDING ||
             state == Gicv3CPUInterface::ICH_LR_EL2_STATE_ACTIVE_PENDING) &&
            lrVintId == vintId) {
            return true;
        }
    }

    return false;
}

bool
Gicv3Redistributor::residentLrHasPendingState() const
{
    if (!vLpiPendingTableValid || residentVptAddr == 0) {
        return false;
    }

    for (int lrIdx = 0; lrIdx < Gicv3CPUInterface::VIRTUAL_NUM_LIST_REGS;
         lrIdx++) {
        const uint64_t lrRaw =
                cpuInterface->isa->readMiscRegNoEffect(
                    MISCREG_ICH_LR0_EL2 + lrIdx);
        const uint64_t state = bits(lrRaw, 63, 62);
        const uint32_t vintId = bits(lrRaw, 31, 0);
        if ((state == Gicv3CPUInterface::ICH_LR_EL2_STATE_PENDING ||
             state == Gicv3CPUInterface::ICH_LR_EL2_STATE_ACTIVE_PENDING) &&
            vintId >= SMALLEST_LPI_ID) {
            return true;
        }
    }

    return false;
}

Gicv3Redistributor::CachedVLPIConfig
Gicv3Redistributor::getCachedVLPIConfig(uint32_t vintId)
{
    // 若没有显式失效，同一 vINTID 会复用旧配置（用于建模缓存可见性）。
    auto cacheIt = vLpiConfigCache.find(vintId);
    if (cacheIt != vLpiConfigCache.end()) {
        return cacheIt->second;
    }

    CachedVLPIConfig cached;
    cached.enable = true;
    cached.priority = 0xa0;

    if (vLpiConfigurationTablePtr) {
        // 缓存未命中时才从 VPROPBASER 对应内存读取配置。
        const LPIConfigurationTableEntry cfg =
            memProxy->read<LPIConfigurationTableEntry>(
                vLpiConfigurationTablePtr + vintId - SMALLEST_LPI_ID);
        cached.priority = static_cast<uint8_t>(cfg.priority << 2);
        cached.enable = cfg.enable;
    }

    vLpiConfigCache[vintId] = cached;
    return cached;
}

void
Gicv3Redistributor::invalidateVLPIConfigOneImpl(uint32_t vintId,
                                                bool pulseSyncBusy)
{
    if (vintId >= SMALLEST_LPI_ID) {
        vLpiConfigCache.erase(vintId);
    }

    if (pulseSyncBusy) {
        /*
         * 最小 GICR_SYNCR.Busy 生命周期：
         * 每次失效写入产生一次“可被读到”的 Busy=1。
         */
        lpiSyncBusyReads = 1;
    }
}

void
Gicv3Redistributor::invalidateVLPIConfigAllImpl(bool pulseSyncBusy)
{
    vLpiConfigCache.clear();

    if (pulseSyncBusy) {
        /*
         * 最小 GICR_SYNCR.Busy 生命周期：
         * 每次失效写入产生一次“可被读到”的 Busy=1。
         */
        lpiSyncBusyReads = 1;
    }
}

void
Gicv3Redistributor::invalidateVLPIConfigOne(uint32_t vintId)
{
    invalidateVLPIConfigOneImpl(vintId, false);
}

void
Gicv3Redistributor::invalidateVLPIConfigAll()
{
    invalidateVLPIConfigAllImpl(false);
}

void
Gicv3Redistributor::syncResidentPendingStateToVpt()
{
    if (!vLpiPendingTableValid || residentVptAddr == 0) {
        return;
    }

    bool lrUpdated = false;

    /*
     * Minimal deschedule sync:
     * Any resident LR still carrying a pending component gets reflected back
     * into the vPT so later schedule-on can replay it.
     */
    for (int lrIdx = 0; lrIdx < Gicv3CPUInterface::VIRTUAL_NUM_LIST_REGS;
         lrIdx++) {
        uint64_t lrRaw =
            cpuInterface->isa->readMiscRegNoEffect(
                MISCREG_ICH_LR0_EL2 + lrIdx);
        const uint64_t state = bits(lrRaw, 63, 62);
        const uint32_t vintId = bits(lrRaw, 31, 0);

        if ((state != Gicv3CPUInterface::ICH_LR_EL2_STATE_PENDING) &&
            (state != Gicv3CPUInterface::ICH_LR_EL2_STATE_ACTIVE_PENDING)) {
            continue;
        }

        if (vintId < SMALLEST_LPI_ID) {
            continue;
        }

        setClrVLPI(residentVptAddr, vintId, true);

        if (state == Gicv3CPUInterface::ICH_LR_EL2_STATE_PENDING) {
            lrRaw = insertBits(lrRaw, 63, 62,
                Gicv3CPUInterface::ICH_LR_EL2_STATE_INVALID);
        } else {
            lrRaw = insertBits(lrRaw, 63, 62,
                Gicv3CPUInterface::ICH_LR_EL2_STATE_ACTIVE);
        }

        cpuInterface->isa->setMiscRegNoEffect(
            MISCREG_ICH_LR0_EL2 + lrIdx, lrRaw);
        lrUpdated = true;
    }

    if (lrUpdated) {
        cpuInterface->virtualUpdate();
    }
}

bool
Gicv3Redistributor::vptHasPendingState() const
{
    if (vLpiPendingTablePtr == 0 || vLpiIDBits > 0xf) {
        return false;
    }

    const uint32_t maxVintId = 1u << (vLpiIDBits + 1);
    if (maxVintId <= SMALLEST_LPI_ID) {
        return false;
    }

    for (uint32_t vint = SMALLEST_LPI_ID; vint < maxVintId; vint += 8) {
        const uint8_t pendingByte = memProxy->read<uint8_t>(
            vLpiPendingTablePtr + (vint / 8));
        uint8_t mask = 0xff;
        if (vint + 8 > maxVintId) {
            mask = (1u << (maxVintId - vint)) - 1u;
        }
        if (pendingByte & mask) {
            return true;
        }
    }

    return false;
}

void
Gicv3Redistributor::scheduleVpeOn(uint64_t data)
{
    // Valid:0->1，进入 resident：建立 vPE 身份并触发 replay。
    vLpiPendingTablePtr = data & 0xFFFFFFFFF0000ULL;
    vLpiPendingTableValid = true;
    vLpiPendingTableDirty = false;
    residentVptAddr = vLpiPendingTablePtr;
    residentVpeId = bits(data, 15, 0);

    auto *its = gic->getIts();
    if (its) {
        uint16_t matchedVpeId = residentVpeId;
        if (its->findVPEForRedistributor(
                this,
                residentVptAddr,
                matchedVpeId)) {
            // 以 ITS 运行态映射为准修正 vPEID，避免软件编码与运行态不一致。
            residentVpeId = matchedVpeId;
        }

        // 重新 schedule 时清掉上一 non-resident 区间的 default doorbell 闩锁。
        const uint32_t doorbell =
                        its->clearDefaultDoorbellPending(residentVpeId);
        if (doorbell != Gicv3::INTID_SPURIOUS) {
            setClrLPI(doorbell, false);
        }

        its->syncPendingVirtualLpis(this);
    }

    /*
     * 最小 schedule 语义：
     * 1) 先 replay vPT pending；
     * 2) replay 后 Dirty 清零；
     * 3) 后续仅当 resident 直注入产生“LR/表临时分离”时再置 Dirty。
     */
    vLpiPendingLast = vptHasPendingState() || residentLrHasPendingState();
    vLpiPendingTableDirty = false;

    DPRINTF(GIC, "GICR_VPENDBASER schedule-on cpu=%u"
                 " vpt=%#llx vpe=%u dirty=%d pendinglast=%d\n",
            cpuId, (unsigned long long)residentVptAddr, residentVpeId,
            vLpiPendingTableDirty, vLpiPendingLast);
}

void
Gicv3Redistributor::scheduleVpeOff()
{
    if (!vLpiPendingTableValid) {
        return;
    }

    // Valid:1->0，进入 non-resident：先把 LR 中 pending 分量回写到 vPT。
    syncResidentPendingStateToVpt();
    vLpiPendingLast = vptHasPendingState() || residentLrHasPendingState();
    vLpiPendingTableDirty = false;
    vLpiPendingTableValid = false;

    DPRINTF(GIC, "GICR_VPENDBASER schedule-off cpu=%u vpt=%#llx"
                 " vpe=%u dirty=%d pendinglast=%d\n",
            cpuId, (unsigned long long)vLpiPendingTablePtr, residentVpeId,
            vLpiPendingTableDirty, vLpiPendingLast);

    residentVptAddr = 0;
    residentVpeId = 0xffff;
}


bool
Gicv3Redistributor::isPendingVLPI(Addr vptAddr, uint32_t vintId)
{
    if (!vptAddr || vintId < SMALLEST_LPI_ID) {
        return false;
    }

    const Addr pendingByteAddr = vptAddr + (vintId / 8);
    const uint8_t pendingByte = memProxy->read<uint8_t>(pendingByteAddr);
    return pendingByte & (1 << (vintId % 8));
}

void
Gicv3Redistributor::setClrVLPI(Addr vptAddr, uint32_t vintId, bool set)
{
    if (!vptAddr || vintId < SMALLEST_LPI_ID) {
        return;
    }

    const Addr pendingByteAddr = vptAddr + (vintId / 8);
    uint8_t pendingByte = memProxy->read<uint8_t>(pendingByteAddr);
    const uint8_t bitMask = 1 << (vintId % 8);

    const bool clearResidentLr = !set && vLpiPendingTableValid &&
        residentVpeId != 0xffff && residentVptAddr == vptAddr;
    DPRINTF(GIC, "setClrVLPI cpu=%u vintid=%u set=%d vpt=%#llx resident_vpe=%u resident_vpt=%#llx clear_lr=%d\n",
            cpuId, vintId, set, (unsigned long long)vptAddr, residentVpeId,
            (unsigned long long)residentVptAddr, clearResidentLr);


    if (set) {
        pendingByte |= bitMask;
    } else {
        pendingByte &= ~bitMask;
        if (clearResidentLr) {
            // CLEAR/DISCARD 需要同步清理 resident LR，避免“表清了但仍可注入”。
            cpuInterface->clearPendingVirtualLPI(vintId);
        }
    }

    memProxy->writeBlob(pendingByteAddr, &pendingByte, sizeof(pendingByte));

    if (vptAddr == vLpiPendingTablePtr) {
        if (set) {
            vLpiPendingLast = true;
        } else {
            vLpiPendingLast =
                vptHasPendingState() || residentLrHasPendingState();
        }
    }
}

bool
Gicv3Redistributor::isVPEResident(uint16_t vpeId, Addr vptAddr) const
{
    return gic->params().gicv4 && !peInLowPowerState && vLpiPendingTableValid &&
        residentVpeId == vpeId && residentVptAddr == vptAddr &&
        vptAddr != 0;
}

bool
Gicv3Redistributor::injectOrPendVLPI(uint16_t vpeId, uint32_t vintId,
                                     Addr vptAddr, uint8_t vptIdBits,
                                     uint32_t doorbellIntid,
                                     Gicv3::GroupId group)
{
    if (vintId < SMALLEST_LPI_ID) {
        return false;
    }

    const uint32_t largestVintId = 1u << (std::min<uint8_t>(vptIdBits, 0xf) + 1);
    if (vintId >= largestVintId) {
        return false;
    }

    // 配置读取走缓存：只有显式失效/同步后才保证看到新属性。
    const CachedVLPIConfig cachedCfg = getCachedVLPIConfig(vintId);
    const uint8_t priority = cachedCfg.priority;
    const bool enabled = cachedCfg.enable;

    const bool wasPending = isPendingVLPI(vptAddr, vintId);
    setClrVLPI(vptAddr, vintId, true);
    vLpiPendingLast = true;
    const bool resident = isVPEResident(vpeId, vptAddr);
    DPRINTF(GIC, "injectOrPendVLPI cpu=%u vpe=%u vintid=%u"
                 " resident=%d vpt=%#llx group=%d\n",
            cpuId, vpeId, vintId,
            resident, (unsigned long long)vptAddr, group);

    if (!resident) {
        // non-resident：不直注入，只保留 pending，并按条件请求 default doorbell。
        if (enabled && !wasPending && gic->getIts()) {
            uint32_t defaultDoorbellIntid = doorbellIntid;
            if (gic->getIts()->requestDefaultDoorbell(
                    vpeId, defaultDoorbellIntid)) {
                /*
                 * 最小 default doorbell 语义：
                 * - 仅实现 default doorbell（不含 individual doorbell）；
                 * - non-resident 同一 residency interval 最多触发一次。
                 */
                setClrLPI(defaultDoorbellIntid, true);
                DPRINTF(GIC, "injectOrPendVLPI cpu=%u vintid=%u"
                             " action=doorbell intid=%u\n",
                        cpuId, vintId, defaultDoorbellIntid);
            }
        }

        DPRINTF(GIC, "injectOrPendVLPI cpu=%u vintid=%u"
                     " action=pend_nonresident\n",
                cpuId, vintId);
        return false;
    }

    if (!enabled) {
        DPRINTF(GIC, "injectOrPendVLPI cpu=%u vintid=%u action=disabled\n",
                cpuId, vintId);
        return false;
    }

    if (!cpuInterface->injectVirtualLPI(vintId, priority, group)) {
        // LR 满时保留 pending，等待后续 EOI/DIR 或 schedule/replay 消费。
        DPRINTF(GIC, "injectOrPendVLPI cpu=%u vintid=%u"
                " action=lr_unavailable\n",
                cpuId, vintId);
        return false;
    }

    DPRINTF(GIC, "injectOrPendVLPI cpu=%u vintid=%u action=delivered_clear_pending\n",
            cpuId, vintId);
    /*
     * Direct-injection delivery semantics:
     * - clear the vPT pending bit after successful LR injection
     * - but do NOT clear resident LR pending state here.
     *
     * setClrVLPI(..., false) is used by CLEAR/DISCARD style maintenance
     * commands and may clear resident LR state (via clearPendingVirtualLPI).
     * Using it here would erase the just-delivered LR before EL1 sees it.
     */
    if (vptAddr && vintId >= SMALLEST_LPI_ID) {
        const Addr pendingByteAddr = vptAddr + (vintId / 8);
        uint8_t pendingByte = memProxy->read<uint8_t>(pendingByteAddr);
        pendingByte &= ~(1 << (vintId % 8));
        memProxy->writeBlob(pendingByteAddr, &pendingByte,
            sizeof(pendingByte));
    }
    vLpiPendingTableDirty = true;
    vLpiPendingLast = true;
    return true;
}

void
Gicv3Redistributor::syncPendingVLPI(uint16_t vpeId, Addr vptAddr, uint8_t vptIdBits)
{
    // Group-sensitive resident replay is handled by the ITS-side
    // virtualIrq state. This fallback helper intentionally does not
    // infer group from the pending table alone.
    (void)vpeId;
    (void)vptAddr;
    (void)vptIdBits;
}

Gicv3::GroupId
Gicv3Redistributor::getIntGroup(int int_id) const
{
    assert(int_id < (Gicv3::SGI_MAX + Gicv3::PPI_MAX));

    if (distributor->DS) {
        if (irqGroup[int_id] == 0) {
            return Gicv3::G0S;
        } else {
            return Gicv3::G1NS;
        }
    } else {
        if (irqGrpmod[int_id] == 0 && irqGroup[int_id] == 0) {
            return Gicv3::G0S;
        } else if (irqGrpmod[int_id] == 0 && irqGroup[int_id] == 1) {
            return Gicv3::G1NS;
        } else if (irqGrpmod[int_id] == 1 && irqGroup[int_id] == 0) {
            return Gicv3::G1S;
        } else if (irqGrpmod[int_id] == 1 && irqGroup[int_id] == 1) {
            return Gicv3::G1NS;
        }
    }

    GEM5_UNREACHABLE;
}

void
Gicv3Redistributor::activateIRQ(uint32_t int_id)
{
    if (treatAsEdgeTriggered(int_id)) {
        irqPending[int_id] = false;
    }
    irqActive[int_id] = true;
}

void
Gicv3Redistributor::deactivateIRQ(uint32_t int_id)
{
    irqActive[int_id] = false;
}

uint32_t
Gicv3Redistributor::getAffinity() const
{
    ThreadContext *tc = gic->getSystem()->threads[cpuId];
    return gem5::ArmISA::getAffinity(gic->getSystem(), tc);
}

bool
Gicv3Redistributor::canBeSelectedFor1toNInterrupt(Gicv3::GroupId group) const
{
    if (peInLowPowerState) {
        return false;
    }

    if (!distributor->groupEnabled(group)) {
        return false;
    }

    if ((group == Gicv3::G1S) && DPG1S) {
        return false;
    }

    if ((group == Gicv3::G1NS) && DPG1NS) {
        return false;
    }

    if ((group == Gicv3::G0S) && DPG0) {
        return false;
    }

    return true;
}

void
Gicv3Redistributor::copy(Gicv3Registers *from, Gicv3Registers *to)
{
    const auto affinity = getAffinity();
    // SGI_Base regs
    gic->copyRedistRegister(from, to, affinity, GICR_CTLR);
    gic->copyRedistRegister(from, to, affinity, GICR_WAKER);

    gic->clearRedistRegister(to, affinity, GICR_ICENABLER0);
    gic->clearRedistRegister(to, affinity, GICR_ICPENDR0);
    gic->clearRedistRegister(to, affinity, GICR_ICACTIVER0);

    gic->copyRedistRegister(from, to, affinity, GICR_ISENABLER0);
    gic->copyRedistRegister(from, to, affinity, GICR_ISPENDR0);
    gic->copyRedistRegister(from, to, affinity, GICR_ISACTIVER0);
    gic->copyRedistRegister(from, to, affinity, GICR_ICFGR0);
    gic->copyRedistRegister(from, to, affinity, GICR_ICFGR1);
    gic->copyRedistRegister(from, to, affinity, GICR_IGRPMODR0);
    gic->copyRedistRegister(from, to, affinity, GICR_NSACR);

    gic->copyRedistRange(from, to, affinity,
        GICR_IPRIORITYR.start(), GICR_IPRIORITYR.size());

    // RD_Base regs
    gic->copyRedistRegister(from, to, affinity, GICR_PROPBASER);
    gic->copyRedistRegister(from, to, affinity, GICR_PENDBASER);
}

void
Gicv3Redistributor::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(peInLowPowerState);
    SERIALIZE_CONTAINER(irqGroup);
    SERIALIZE_CONTAINER(irqEnabled);
    SERIALIZE_CONTAINER(irqPending);
    SERIALIZE_CONTAINER(irqPendingIspendr);
    SERIALIZE_CONTAINER(irqActive);
    SERIALIZE_CONTAINER(irqPriority);
    SERIALIZE_CONTAINER(irqConfig);
    SERIALIZE_CONTAINER(irqGrpmod);
    SERIALIZE_CONTAINER(irqNsacr);
    SERIALIZE_SCALAR(DPG1S);
    SERIALIZE_SCALAR(DPG1NS);
    SERIALIZE_SCALAR(DPG0);
    SERIALIZE_SCALAR(EnableLPIs);
    SERIALIZE_SCALAR(lpiConfigurationTablePtr);
    SERIALIZE_SCALAR(lpiIDBits);
    SERIALIZE_SCALAR(lpiPendingTablePtr);
    SERIALIZE_SCALAR(vLpiConfigurationTablePtr);
    SERIALIZE_SCALAR(vLpiIDBits);
    SERIALIZE_SCALAR(vLpiPendingTablePtr);
    SERIALIZE_SCALAR(vLpiPendingTableValid);
    SERIALIZE_SCALAR(vLpiPendingTableDirty);
    SERIALIZE_SCALAR(vLpiPendingLast);
    SERIALIZE_SCALAR(residentVpeId);
    SERIALIZE_SCALAR(residentVptAddr);
    SERIALIZE_SCALAR(lpiSyncBusyReads);
}

void
Gicv3Redistributor::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(peInLowPowerState);
    UNSERIALIZE_CONTAINER(irqGroup);
    UNSERIALIZE_CONTAINER(irqEnabled);
    UNSERIALIZE_CONTAINER(irqPending);
    UNSERIALIZE_CONTAINER(irqPendingIspendr);
    UNSERIALIZE_CONTAINER(irqActive);
    UNSERIALIZE_CONTAINER(irqPriority);
    UNSERIALIZE_CONTAINER(irqConfig);
    UNSERIALIZE_CONTAINER(irqGrpmod);
    UNSERIALIZE_CONTAINER(irqNsacr);
    UNSERIALIZE_SCALAR(DPG1S);
    UNSERIALIZE_SCALAR(DPG1NS);
    UNSERIALIZE_SCALAR(DPG0);
    UNSERIALIZE_SCALAR(EnableLPIs);
    UNSERIALIZE_SCALAR(lpiConfigurationTablePtr);
    UNSERIALIZE_SCALAR(lpiIDBits);
    UNSERIALIZE_SCALAR(lpiPendingTablePtr);
    UNSERIALIZE_SCALAR(vLpiConfigurationTablePtr);
    UNSERIALIZE_SCALAR(vLpiIDBits);
    UNSERIALIZE_SCALAR(vLpiPendingTablePtr);
    UNSERIALIZE_SCALAR(vLpiPendingTableValid);
    UNSERIALIZE_SCALAR(vLpiPendingTableDirty);
    UNSERIALIZE_SCALAR(vLpiPendingLast);
    UNSERIALIZE_SCALAR(residentVpeId);
    UNSERIALIZE_SCALAR(residentVptAddr);
    UNSERIALIZE_SCALAR(lpiSyncBusyReads);
    vLpiConfigCache.clear();
}
} // namespace gem5
