/*
 * Copyright (c) 2019 ARM Limited
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

#include "dev/arm/gic_v3_its.hh"

#include <cassert>
#include <functional>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/AddrRanges.hh"
#include "debug/Drain.hh"
#include "debug/GIC.hh"
#include "debug/ITS.hh"
#include "dev/arm/gic_v3.hh"
#include "dev/arm/gic_v3_distributor.hh"
#include "dev/arm/gic_v3_redistributor.hh"
#include "mem/packet_access.hh"
#include "params/Gicv3Its.hh"

#define COMMAND(x, method) { x, DispatchEntry(#x, method) }

namespace gem5
{

const AddrRange Gicv3Its::GITS_BASER(0x0100, 0x0140);

const uint32_t Gicv3Its::CTLR_QUIESCENT = 0x80000000;

uint64_t
Gicv3Its::virtualIrqKey(uint32_t deviceId, uint32_t eventId) const
{
    // 统一键值编码，保证命令路径和翻译路径都用同一索引。
    return (static_cast<uint64_t>(deviceId) << 32) | eventId;
}

Gicv3Its::VirtualPE *
Gicv3Its::findVPE(uint16_t vpeId)
{
    auto it = virtualPes.find(vpeId);
    return it == virtualPes.end() ? nullptr : &it->second;
}

const Gicv3Its::VirtualPE *
Gicv3Its::findVPE(uint16_t vpeId) const
{
    auto it = virtualPes.find(vpeId);
    return it == virtualPes.end() ? nullptr : &it->second;
}

Gicv3Its::VirtualIrqEntry *
Gicv3Its::findVirtualIrq(uint32_t deviceId, uint32_t eventId)
{
    auto it = virtualIrqs.find(virtualIrqKey(deviceId, eventId));
    return it == virtualIrqs.end() ? nullptr : &it->second;
}

const Gicv3Its::VirtualIrqEntry *
Gicv3Its::findVirtualIrq(uint32_t deviceId, uint32_t eventId) const
{
    auto it = virtualIrqs.find(virtualIrqKey(deviceId, eventId));
    return it == virtualIrqs.end() ? nullptr : &it->second;
}

void
Gicv3Its::syncPendingVirtualLpis(Gicv3Redistributor *rd)
{
    /*
     * 该 helper 是“schedule-on/replay”的核心桥接点：
     * redistributor 进入 resident 后，通过 ITS 的 vIRQ 映射表扫描
     * 当前 vPE 的 pending 位并尝试注入，从而把表状态转换回 LR 状态。
     */
    if (!rd) {
        return;
    }

    const uint16_t residentVpeId = rd->residentVPEID();
    const Addr residentVptAddr = rd->residentVPTAddr();
    DPRINTF(ITS, "syncPendingVirtualLpis cpu=%u resident_vpe=%u resident_vpt=%#llx\n",
            rd->processorNumber(), residentVpeId,
            (unsigned long long)residentVptAddr);
    if (residentVpeId == 0xffff || residentVptAddr == 0) {
        return;
    }

    const auto *vpe = findVPE(residentVpeId);
    if (!vpe || !vpe->valid || vpe->vptAddr != residentVptAddr ||
        getRedistributor(vpe->rdBase) != rd) {
        return;
    }

    for (const auto &entry : virtualIrqs) {
        const auto &virtualIrq = entry.second;
        if (!virtualIrq.valid || virtualIrq.vpeid != residentVpeId) {
            continue;
        }

        if (!rd->isPendingVLPI(vpe->vptAddr, virtualIrq.vintid)) {
            continue;
        }

        DPRINTF(ITS, "syncPendingVirtualLpis replay cpu=%u vpe=%u vintid=%u group=%d\n",
                rd->processorNumber(), residentVpeId, virtualIrq.vintid,
                virtualIrq.group);
        if (!rd->injectOrPendVLPI(residentVpeId, virtualIrq.vintid,
                                  vpe->vptAddr, vpe->vptIdBits,
                                  virtualIrq.doorbellIntid,
                                  virtualIrq.group)) {
            break;
        }
    }
}

bool
Gicv3Its::findVPEForRedistributor(Gicv3Redistributor *rd, Addr vptAddr,
                                 uint16_t &vpeId, uint8_t *vptIdBits) const
{
    if (!rd || vptAddr == 0) {
        return false;
    }

    for (const auto &entry : virtualPes) {
        const auto &vpe = entry.second;
        if (!vpe.valid || vpe.vptAddr != vptAddr) {
            continue;
        }

        if (const_cast<Gicv3Its *>(this)->getRedistributor(vpe.rdBase) != rd) {
            continue;
        }

        vpeId = entry.first;
        if (vptIdBits) {
            *vptIdBits = vpe.vptIdBits;
        }
        return true;
    }

    return false;
}

bool
Gicv3Its::requestDefaultDoorbell(uint16_t vpeId, uint32_t &doorbellIntid)
{
    auto *vpe = findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        return false;
    }

    /*
     * Prefer VMAPP-provided default doorbell; if absent, conservatively
     * fall back to any valid VMAPTI/VMAPI-provided doorbell for this vPE.
     * This keeps a minimal but robust model when software programs only the
     * per-vIRQ path.
     */
    uint32_t resolvedDoorbell = vpe->doorbellIntid;
    if (resolvedDoorbell == Gicv3::INTID_SPURIOUS ||
        resolvedDoorbell < Gicv3Redistributor::SMALLEST_LPI_ID) {
        for (const auto &entry : virtualIrqs) {
            const auto &virt = entry.second;
            if (!virt.valid || virt.vpeid != vpeId) {
                continue;
            }
            if (virt.doorbellIntid == Gicv3::INTID_SPURIOUS ||
                virt.doorbellIntid < Gicv3Redistributor::SMALLEST_LPI_ID) {
                continue;
            }
            resolvedDoorbell = virt.doorbellIntid;
            break;
        }
    }

    doorbellIntid = resolvedDoorbell;
    if (doorbellIntid == Gicv3::INTID_SPURIOUS ||
        doorbellIntid < Gicv3Redistributor::SMALLEST_LPI_ID) {
        return false;
    }

    if (vpe->defaultDoorbellPending) {
        // 同一 non-resident residency interval 内只允许一次 doorbell。
        return false;
    }

    vpe->defaultDoorbellPending = true;
    DPRINTF(ITS, "requestDefaultDoorbell vpe=%u doorbell=%u action=arm\n",
            vpeId, doorbellIntid);
    return true;
}

uint32_t
Gicv3Its::clearDefaultDoorbellPending(uint16_t vpeId)
{
    auto *vpe = findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        return Gicv3::INTID_SPURIOUS;
    }

    vpe->defaultDoorbellPending = false;
    uint32_t resolvedDoorbell = vpe->doorbellIntid;
    if (resolvedDoorbell == Gicv3::INTID_SPURIOUS ||
        resolvedDoorbell < Gicv3Redistributor::SMALLEST_LPI_ID) {
        for (const auto &entry : virtualIrqs) {
            const auto &virt = entry.second;
            if (!virt.valid || virt.vpeid != vpeId) {
                continue;
            }
            if (virt.doorbellIntid == Gicv3::INTID_SPURIOUS ||
                virt.doorbellIntid < Gicv3Redistributor::SMALLEST_LPI_ID) {
                continue;
            }
            resolvedDoorbell = virt.doorbellIntid;
            break;
        }
    }

    DPRINTF(ITS, "clearDefaultDoorbellPending vpe=%u doorbell=%u\n",
            vpeId, resolvedDoorbell);
    return resolvedDoorbell;
}

void
Gicv3Its::sendVirtualSGI(uint16_t vpeId, uint32_t vintId)
{
    /*
     * vSGI 统一注入入口（最小模型）：
     * - 直接 MMIO 写 GITS_SGIR 走这里；
     * - guest ICC_SGI1R_EL1 trap+translate 后也应落到这里。
     * 这样可以保证两条路径共享同一份 pending/replay 真相。
     */
    auto *vpe = findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        warn("GITS_SGIR ignored: vPEID %u not mapped\n", vpeId);
        return;
    }
    if (vintId >= Gicv3::SGI_MAX) {
        warn("GITS_SGIR ignored: vINTID %u out of SGI range\n", vintId);
        return;
    }

    auto *rd = getRedistributor(vpe->rdBase);
    rd->injectOrPendVSGI(vpeId, vpe->vptAddr, vpe->vptIdBits, vintId);
}

ItsProcess::ItsProcess(Gicv3Its &_its)
  : its(_its), coroutine(nullptr)
{
}

ItsProcess::~ItsProcess()
{
}

void
ItsProcess::reinit()
{
    coroutine.reset(new Coroutine(
        std::bind(&ItsProcess::main, this, std::placeholders::_1)));
}

const std::string
ItsProcess::name() const
{
    return its.name();
}

ItsAction
ItsProcess::run(PacketPtr pkt)
{
    assert(coroutine != nullptr);
    assert(*coroutine);
    return (*coroutine)(pkt).get();
}

void
ItsProcess::doRead(Yield &yield, Addr addr, void *ptr, size_t size)
{
    ItsAction a;
    a.type = ItsActionType::SEND_REQ;

    RequestPtr req = std::make_shared<Request>(
        addr, size, 0, its.requestorId);

    req->taskId(context_switch_task_id::DMA);

    a.pkt = new Packet(req, MemCmd::ReadReq);
    a.pkt->dataStatic(ptr);

    a.delay = 0;

    PacketPtr pkt = yield(a).get();

    assert(pkt);
    assert(pkt->getSize() >= size);

    delete pkt;
}

void
ItsProcess::doWrite(Yield &yield, Addr addr, void *ptr, size_t size)
{
    ItsAction a;
    a.type = ItsActionType::SEND_REQ;

    RequestPtr req = std::make_shared<Request>(
        addr, size, 0, its.requestorId);

    req->taskId(context_switch_task_id::DMA);

    a.pkt = new Packet(req, MemCmd::WriteReq);
    a.pkt->dataStatic(ptr);

    a.delay = 0;

    PacketPtr pkt = yield(a).get();

    assert(pkt);
    assert(pkt->getSize() >= size);

    delete pkt;
}

void
ItsProcess::terminate(Yield &yield)
{
    ItsAction a;
    a.type = ItsActionType::TERMINATE;
    a.pkt = NULL;
    a.delay = 0;
    yield(a);
}

void
ItsProcess::writeDeviceTable(Yield &yield, uint32_t device_id, DTE dte)
{
    const Addr base = its.pageAddress(Gicv3Its::DEVICE_TABLE);
    const Addr address = base + (device_id * sizeof(dte));

    DPRINTF(ITS, "Writing DTE at address %#x: %#x\n", address, dte);

    doWrite(yield, address, &dte, sizeof(dte));
}

void
ItsProcess::writeIrqTranslationTable(
    Yield &yield, const Addr itt_base, uint32_t event_id, ITTE itte)
{
    const Addr address = itt_base + (event_id * sizeof(itte));

    doWrite(yield, address, &itte, sizeof(itte));

    DPRINTF(ITS, "Writing ITTE at address %#x: %#x\n", address, itte);
}

void
ItsProcess::writeIrqCollectionTable(
    Yield &yield, uint32_t collection_id, CTE cte)
{
    const Addr base = its.pageAddress(Gicv3Its::COLLECTION_TABLE);
    const Addr address = base + (collection_id * sizeof(cte));

    doWrite(yield, address, &cte, sizeof(cte));

    DPRINTF(ITS, "Writing CTE at address %#x: %#x\n", address, cte);
}

uint64_t
ItsProcess::readDeviceTable(Yield &yield, uint32_t device_id)
{
    uint64_t dte;
    const Addr base = its.pageAddress(Gicv3Its::DEVICE_TABLE);
    const Addr address = base + (device_id * sizeof(dte));

    doRead(yield, address, &dte, sizeof(dte));

    DPRINTF(ITS, "Reading DTE at address %#x: %#x\n", address, dte);
    return dte;
}

uint64_t
ItsProcess::readIrqTranslationTable(
    Yield &yield, const Addr itt_base, uint32_t event_id)
{
    uint64_t itte;
    const Addr address = itt_base + (event_id * sizeof(itte));

    doRead(yield, address, &itte, sizeof(itte));

    DPRINTF(ITS, "Reading ITTE at address %#x: %#x\n", address, itte);
    return itte;
}

uint64_t
ItsProcess::readIrqCollectionTable(Yield &yield, uint32_t collection_id)
{
    uint64_t cte;
    const Addr base = its.pageAddress(Gicv3Its::COLLECTION_TABLE);
    const Addr address = base + (collection_id * sizeof(cte));

    doRead(yield, address, &cte, sizeof(cte));

    DPRINTF(ITS, "Reading CTE at address %#x: %#x\n", address, cte);
    return cte;
}

ItsTranslation::ItsTranslation(Gicv3Its &_its)
  : ItsProcess(_its)
{
    reinit();
    its.pendingTranslations++;
    its.gitsControl.quiescent = 0;
}

ItsTranslation::~ItsTranslation()
{
    assert(its.pendingTranslations >= 1);
    its.pendingTranslations--;
    if (!its.pendingTranslations && !its.pendingCommands)
        its.gitsControl.quiescent = 1;
}

void
ItsTranslation::main(Yield &yield)
{
    PacketPtr pkt = yield.get();

    const uint32_t device_id = pkt->req->streamId();
    const uint32_t event_id = pkt->getLE<uint32_t>();

    auto result = translateLPI(yield, device_id, event_id);

    /*
     * 翻译后统一在这里分流：
     * - physical: 直接置位 LPI pending
     * - virtual : 交给 injectOrPendVLPI 决定是 direct inject 还是
     *   non-resident pending + doorbell
     */
    if (result.type == Gicv3Its::PHYSICAL_INTERRUPT) {
        result.redistributor->setClrLPI(result.intid, true);
    } else {
        result.redistributor->injectOrPendVLPI(
            result.vpeid, result.intid, result.vptAddr,
            result.vptIdBits, result.doorbellIntid, result.group);
    }

    its.gitsTranslater = event_id;
    terminate(yield);
}

Gicv3Its::TranslatedInt
ItsTranslation::translateLPI(Yield &yield, uint32_t device_id,
                             uint32_t event_id)
{
    if (its.deviceOutOfRange(device_id)) {
        terminate(yield);
    }
    DTE dte = readDeviceTable(yield, device_id);
    if (!dte.valid || its.idOutOfRange(event_id, dte.ittRange)) {
        terminate(yield);
    }
    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, event_id);
    if (!itte.valid) {
        terminate(yield);
    }

    DPRINTF(ITS, "translateLPI dev=%u event=%u type=%s vpe=%u intid=%u\n",
        device_id, event_id,
        itte.intType == Gicv3Its::VIRTUAL_INTERRUPT ? "virtual" : "physical",
        itte.vpeid, itte.intNum);

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        // virtual 路径必须再通过运行态映射校验，避免表项与运行态脱节。
        const auto *virtualIrq = its.findVirtualIrq(device_id, event_id);
        if (!virtualIrq || !virtualIrq->valid) {
            terminate(yield);
        }
        const auto *vpe = its.findVPE(virtualIrq->vpeid);
        if (!vpe || !vpe->valid) {
            terminate(yield);
        }
        Gicv3Its::TranslatedInt result;
        result.type = Gicv3Its::VIRTUAL_INTERRUPT;
        result.intid = virtualIrq->vintid;
        result.doorbellIntid = virtualIrq->doorbellIntid;
        result.vpeid = virtualIrq->vpeid;
        result.vptAddr = vpe->vptAddr;
        result.vptIdBits = vpe->vptIdBits;
        result.group = virtualIrq->group;
        result.redistributor = its.getRedistributor(vpe->rdBase);
        return result;
    }
    const auto collection_id = itte.icid;
    if (its.collectionOutOfRange(collection_id)) {
        terminate(yield);
    }
    CTE cte = readIrqCollectionTable(yield, collection_id);
    if (!cte.valid) {
        terminate(yield);
    }
    Gicv3Its::TranslatedInt result;
    result.type = Gicv3Its::PHYSICAL_INTERRUPT;
    result.intid = itte.intNum;
    result.redistributor = its.getRedistributor(cte);
    return result;
}

ItsCommand::DispatchTable ItsCommand::cmdDispatcher =
{
    COMMAND(CLEAR, &ItsCommand::clear),
    COMMAND(DISCARD, &ItsCommand::discard),
    COMMAND(INT, &ItsCommand::doInt),
    COMMAND(INV, &ItsCommand::inv),
    COMMAND(INVALL, &ItsCommand::invall),
    COMMAND(MAPC, &ItsCommand::mapc),
    COMMAND(MAPD, &ItsCommand::mapd),
    COMMAND(MAPI, &ItsCommand::mapi),
    COMMAND(MAPTI, &ItsCommand::mapti),
    COMMAND(MOVALL, &ItsCommand::movall),
    COMMAND(MOVI, &ItsCommand::movi),
    COMMAND(SYNC, &ItsCommand::sync),
    COMMAND(VINVALL, &ItsCommand::vinvall),
    COMMAND(VMAPI, &ItsCommand::vmapi),
    COMMAND(VMAPP, &ItsCommand::vmapp),
    COMMAND(VMAPTI, &ItsCommand::vmapti),
    COMMAND(VMOVI, &ItsCommand::vmovi),
    COMMAND(VMOVP, &ItsCommand::vmovp),
    COMMAND(VSYNC, &ItsCommand::vsync),
    COMMAND(VSGI, &ItsCommand::vsgi),
};

ItsCommand::ItsCommand(Gicv3Its &_its)
  : ItsProcess(_its)
{
    reinit();
    its.pendingCommands = true;

    its.gitsControl.quiescent = 0;
}

ItsCommand::~ItsCommand()
{
    its.pendingCommands = false;

    if (!its.pendingTranslations)
        its.gitsControl.quiescent = 1;
}

std::string
ItsCommand::commandName(uint32_t cmd)
{
    const auto entry = cmdDispatcher.find(cmd);
    return entry != cmdDispatcher.end() ? entry->second.name : "INVALID";
}

void
ItsCommand::main(Yield &yield)
{
    ItsAction a;
    a.type = ItsActionType::INITIAL_NOP;
    a.pkt = nullptr;
    a.delay = 0;
    yield(a);

    while (its.gitsCwriter.offset != its.gitsCreadr.offset) {
        CommandEntry command;

        // Reading the command from CMDQ
        readCommand(yield, command);

        processCommand(yield, command);

        its.incrementReadPointer();
    }

    terminate(yield);
}

void
ItsCommand::readCommand(Yield &yield, CommandEntry &command)
{
    // read the command pointed by GITS_CREADR
    const Addr cmd_addr =
        (its.gitsCbaser.physAddr << 12) + (its.gitsCreadr.offset << 5);

    doRead(yield, cmd_addr, &command, sizeof(command));

    DPRINTF(ITS, "Command %s read from queue at address: %#x\n",
            commandName(command.type), cmd_addr);
    DPRINTF(ITS, "dw0: %#x dw1: %#x dw2: %#x dw3: %#x\n",
            command.raw[0], command.raw[1], command.raw[2], command.raw[3]);
}

void
ItsCommand::processCommand(Yield &yield, CommandEntry &command)
{
    const auto entry = cmdDispatcher.find(command.type);

    if (entry != cmdDispatcher.end()) {
        // Execute the command
        entry->second.exec(this, yield, command);
    } else {
        panic("Unrecognized command type: %u", command.type);
    }
}

void
ItsCommand::clear(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);
    if (!dte.valid || idOutOfRange(command, dte)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);
    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        // CLEAR 在 virtual 路径只清 pending，不删除映射。
        const auto *virtualIrq =
                its.findVirtualIrq(command.deviceId, command.eventId);
        const auto *vpe = virtualIrq ?
                        its.findVPE(virtualIrq->vpeid) : nullptr;
        if (!virtualIrq || !virtualIrq->valid || !vpe || !vpe->valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        its.getRedistributor(vpe->rdBase)->setClrVLPI(vpe->vptAddr,
                                                      virtualIrq->vintid,
                                                      false);
        return;
    }

    const auto collection_id = itte.icid;
    CTE cte = readIrqCollectionTable(yield, collection_id);
    if (!cte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    its.getRedistributor(cte)->setClrLPI(itte.intNum, false);
}

void
ItsCommand::discard(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);
    if (!dte.valid || idOutOfRange(command, dte)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);
    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        // DISCARD 在 virtual 路径既清 pending，也移除映射。
        const auto *virtualIrq =
                its.findVirtualIrq(command.deviceId, command.eventId);
        const auto *vpe = virtualIrq ?
                        its.findVPE(virtualIrq->vpeid) : nullptr;
        if (virtualIrq && virtualIrq->valid && vpe && vpe->valid) {
            its.getRedistributor(vpe->rdBase)->setClrVLPI(vpe->vptAddr,
                                                          virtualIrq->vintid,
                                                          false);
        }
        its.virtualIrqs.erase(its.virtualIrqKey(command.deviceId, command.eventId));
    } else {
        const auto collection_id = itte.icid;
        Gicv3Its::CTE cte = readIrqCollectionTable(yield, collection_id);
        if (!cte.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }
        its.getRedistributor(cte)->setClrLPI(itte.intNum, false);
    }

    itte.valid = 0;
    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);
}

void
ItsCommand::doInt(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);
    if (!dte.valid || idOutOfRange(command, dte)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);
    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        // vLPI direct injection 主入口：后续 resident/non-resident 由 RD 决策。
        const auto *virtualIrq =
                its.findVirtualIrq(command.deviceId, command.eventId);
        const auto *vpe = virtualIrq ?
                        its.findVPE(virtualIrq->vpeid) : nullptr;
        if (!virtualIrq || !virtualIrq->valid || !vpe || !vpe->valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        its.getRedistributor(vpe->rdBase)->injectOrPendVLPI(
            virtualIrq->vpeid, virtualIrq->vintid, vpe->vptAddr,
            vpe->vptIdBits, virtualIrq->doorbellIntid, virtualIrq->group);
        return;
    }

    const auto collection_id = itte.icid;
    CTE cte = readIrqCollectionTable(yield, collection_id);
    if (!cte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    its.getRedistributor(cte)->setClrLPI(itte.intNum, true);
}

void
ItsCommand::inv(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);

    if (!dte.valid || idOutOfRange(command, dte)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    const auto collection_id = itte.icid;
    CTE cte = readIrqCollectionTable(yield, collection_id);

    if (!cte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }
    // Do nothing since caching is currently not supported in
    // Redistributor
}

void
ItsCommand::invall(Yield &yield, CommandEntry &command)
{
    if (collectionOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    const auto icid = bits(command.raw[2], 15, 0);

    CTE cte = readIrqCollectionTable(yield, icid);

    if (!cte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }
    // Do nothing since caching is currently not supported in
    // Redistributor
}

void
ItsCommand::mapc(Yield &yield, CommandEntry &command)
{
    if (collectionOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    CTE cte = 0;
    cte.valid = bits(command.raw[2], 63);
    cte.rdBase = bits(command.raw[2], 50, 16);

    const auto icid = bits(command.raw[2], 15, 0);

    writeIrqCollectionTable(yield, icid, cte);
}

void
ItsCommand::mapd(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command) || sizeOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = 0;
    dte.valid = bits(command.raw[2], 63);
    dte.ittAddress = mbits(command.raw[2], 51, 8);
    dte.ittRange = bits(command.raw[1], 4, 0);

    writeDeviceTable(yield, command.deviceId, dte);
}

void
ItsCommand::mapi(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (collectionOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);

    if (!dte.valid || idOutOfRange(command, dte) ||
        its.lpiOutOfRange(command.eventId)) {

        its.incrementReadPointer();
        terminate(yield);
    }

    Gicv3Its::ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    itte.valid = 1;
    itte.intType = Gicv3Its::PHYSICAL_INTERRUPT;
    itte.intNum = command.eventId;
    itte.icid = bits(command.raw[2], 15, 0);

    writeIrqTranslationTable(
        yield, dte.ittAddress, command.eventId, itte);
}

void
ItsCommand::mapti(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (collectionOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);

    const auto pintid = bits(command.raw[1], 63, 32);

    if (!dte.valid || idOutOfRange(command, dte) ||
        its.lpiOutOfRange(pintid)) {

        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    itte.valid = 1;
    itte.intType = Gicv3Its::PHYSICAL_INTERRUPT;
    itte.intNum = pintid;
    itte.icid = bits(command.raw[2], 15, 0);

    writeIrqTranslationTable(
        yield, dte.ittAddress, command.eventId, itte);
}

void
ItsCommand::movall(Yield &yield, CommandEntry &command)
{
    const uint64_t rd1 = bits(command.raw[2], 50, 16);
    const uint64_t rd2 = bits(command.raw[3], 50, 16);

    if (rd1 != rd2) {
        Gicv3Redistributor * redist1 = its.getRedistributor(rd1);
        Gicv3Redistributor * redist2 = its.getRedistributor(rd2);

        its.moveAllPendingState(redist1, redist2);
    }
}

void
ItsCommand::movi(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (collectionOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);

    if (!dte.valid || idOutOfRange(command, dte)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    if (!itte.valid || itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        its.incrementReadPointer();
        terminate(yield);
    }

    const auto collection_id1 = itte.icid;
    CTE cte1 = readIrqCollectionTable(yield, collection_id1);

    if (!cte1.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    const auto collection_id2 = bits(command.raw[2], 15, 0);
    CTE cte2 = readIrqCollectionTable(yield, collection_id2);

    if (!cte2.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    Gicv3Redistributor *first_redist = its.getRedistributor(cte1);
    Gicv3Redistributor *second_redist = its.getRedistributor(cte2);

    if (second_redist != first_redist) {
        // move pending state of the interrupt from one redistributor
        // to the other.
        if (first_redist->isPendingLPI(itte.intNum)) {
            first_redist->setClrLPI(itte.intNum, false);
            second_redist->setClrLPI(itte.intNum, true);
        }
    }

    itte.icid = collection_id2;
    writeIrqTranslationTable(
        yield, dte.ittAddress, command.eventId, itte);
}

void
ItsCommand::sync(Yield &yield, CommandEntry &command)
{
    // No ITS-side caches are modelled in this minimal implementation.
}

void
ItsCommand::vinvall(Yield &yield, CommandEntry &command)
{
    const uint16_t vpeId = bits(command.raw[2], 15, 0);
    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VINVALL rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }

    auto *vpe = its.findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        warn("ITS VINVALL rejected: vPEID %u not mapped\n", vpeId);
        its.incrementReadPointer();
        terminate(yield);
    }

    /*
     * 最小 invalidation 语义：
     * 仅使目标 RD 的 vLPI 配置缓存失效，不模拟更复杂缓存层级。
     * 这样能保证“修改属性后需显式失效/同步才生效”的可观测行为。
     */
    its.getRedistributor(vpe->rdBase)->invalidateVLPIConfigAll();
}

void
ItsCommand::vmapi(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);
    const uint16_t vpeId = bits(command.raw[2], 15, 0);
    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VMAPI rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }
    const auto *vpe = its.findVPE(vpeId);
    if (!dte.valid || idOutOfRange(command, dte) || !vpe || !vpe->valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);
    const uint32_t doorbellIntid = bits(command.raw[1], 63, 32);
    itte.valid = 1;
    itte.intType = Gicv3Its::VIRTUAL_INTERRUPT;
    itte.vpeid = vpeId;
    itte.intNum = command.eventId & 0x3fff;
    itte.intNumHyp = doorbellIntid & 0x3fff;
    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);

    // VMAPI 在该最小实现中同时更新 ITTE 与运行态 virtualIrqs。
    Gicv3Its::VirtualIrqEntry virt;
    virt.valid = true;
    // Minimal implementation: virtual LPIs are currently modelled as Group1NS only.
    virt.group = Gicv3::G1NS;
    virt.vintid = command.eventId;
    virt.doorbellIntid = doorbellIntid;
    virt.vpeid = vpeId;
    its.virtualIrqs[its.virtualIrqKey(command.deviceId, command.eventId)] = virt;
}

void
ItsCommand::vmapp(Yield &yield, CommandEntry &command)
{
    const uint16_t vpeId = bits(command.raw[1], 15, 0);
    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VMAPP rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }

    Gicv3Its::VirtualPE &vpe = its.virtualPes[vpeId];
    vpe.valid = bits(command.raw[2], 63);
    vpe.vptIdBits = bits(command.raw[2], 59, 53);
    vpe.vptAddr = command.raw[2] & 0xFFFFFFFFF0000ULL;
    vpe.rdBase = bits(command.raw[3], 50, 16);
    vpe.doorbellIntid = bits(command.raw[3], 31, 0);
    /*
     * default doorbell 闩锁在 VMAPP 时清零，表示进入新的 residency 区间。
     * 若不清零，会导致上一轮 non-resident 的一次性 doorbell 状态泄漏。
     */
    vpe.defaultDoorbellPending = false;

    if (vpe.valid) {
        its.syncPendingVirtualLpis(its.getRedistributor(vpe.rdBase));
    }
}

void
ItsCommand::vmapti(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);
    const uint16_t vpeId = bits(command.raw[2], 15, 0);
    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VMAPTI rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }
    const auto *vpe = its.findVPE(vpeId);
    const uint32_t vintid = bits(command.raw[1], 63, 32);
    if (!dte.valid || idOutOfRange(command, dte) || !vpe || !vpe->valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);
    itte.valid = 1;
    itte.intType = Gicv3Its::VIRTUAL_INTERRUPT;
    itte.vpeid = vpeId;
    itte.intNum = vintid & 0x3fff;
    itte.intNumHyp = bits(command.raw[3], 13, 0);
    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);

    // VMAPTI 建立 (DeviceID, EventID) 到 (vINTID, vPE) 的核心映射。
    Gicv3Its::VirtualIrqEntry virt;
    virt.valid = true;
    // Minimal implementation: virtual LPIs are currently modelled as Group1NS only.
    virt.group = Gicv3::G1NS;
    virt.vintid = vintid;
    virt.doorbellIntid = bits(command.raw[3], 31, 0);
    virt.vpeid = vpeId;
    its.virtualIrqs[its.virtualIrqKey(command.deviceId, command.eventId)] = virt;
}

void
ItsCommand::vmovi(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);
    if (!dte.valid || idOutOfRange(command, dte)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);
    if (!itte.valid || itte.intType != Gicv3Its::VIRTUAL_INTERRUPT) {
        its.incrementReadPointer();
        terminate(yield);
    }

    auto *virt = its.findVirtualIrq(command.deviceId, command.eventId);
    const uint16_t newVpeId = bits(command.raw[2], 15, 0);
    if (its.vpeOutOfRange(newVpeId)) {
        warn("ITS VMOVI rejected: vPEID %u out of range (max %u)\n",
             newVpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }

    const auto *oldVpe = virt ? its.findVPE(virt->vpeid) : nullptr;
    const auto *newVpe = its.findVPE(newVpeId);
    if (!virt || !virt->valid || !newVpe || !newVpe->valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    // VMOVI 需要迁移“映射 + pending 状态”，否则会出现迁移后丢中断。
    bool hadPending = false;
    if (oldVpe && oldVpe->valid) {
        auto *oldRedist = its.getRedistributor(oldVpe->rdBase);
        hadPending = oldRedist->isPendingVLPI(oldVpe->vptAddr, virt->vintid);

        /*
         * source vPE 若 resident，pending 可能只在 LR 中而不在表里。
         * 先把 LR pending 分量回写到 vPT，再执行迁移，避免在飞中断丢失。
         */
        if (oldRedist->isVPEResident(virt->vpeid, oldVpe->vptAddr) &&
            oldRedist->residentLrHasPendingVintid(virt->vintid)) {
            oldRedist->setClrVLPI(oldVpe->vptAddr, virt->vintid, true);
            hadPending = true;
        }

        if (hadPending) {
            oldRedist->setClrVLPI(oldVpe->vptAddr, virt->vintid, false);
        }
    }

    if (hadPending) {
        its.getRedistributor(newVpe->rdBase)->setClrVLPI(
            newVpe->vptAddr, virt->vintid, true);
    }

    virt->vpeid = newVpeId;
    itte.vpeid = newVpeId;
    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);
    its.syncPendingVirtualLpis(its.getRedistributor(newVpe->rdBase));
}

void
ItsCommand::vmovp(Yield &yield, CommandEntry &command)
{
    const uint16_t vpeId = bits(command.raw[1], 15, 0);
    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VMOVP rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }

    auto *vpe = its.findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        warn("ITS VMOVP rejected: vPEID %u not mapped\n", vpeId);
        its.incrementReadPointer();
        terminate(yield);
        return;
    }

    auto *oldRedist = its.getRedistributor(vpe->rdBase);
    if (oldRedist->isVPEResident(vpeId, vpe->vptAddr)) {
        // 统一策略：resident vPE 不允许 VMOVP，避免与在飞注入语义冲突。
        warn("ITS VMOVP rejected: resident vPE %u cannot migrate\n", vpeId);
        its.incrementReadPointer();
        terminate(yield);
    }

    /*
     * 非 resident VMOVP 最小策略：
     * 仅更新 vPE->RD 绑定关系，不在这里做额外“猜测性注入”；
     * 迁移后的可见行为由后续 sync/replay 统一收敛。
     */
    vpe->rdBase = bits(command.raw[2], 50, 16);
    its.syncPendingVirtualLpis(its.getRedistributor(vpe->rdBase));
}

void
ItsCommand::vsync(Yield &yield, CommandEntry &command)
{
    const uint16_t vpeId = bits(command.raw[1], 15, 0);
    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VSYNC rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }

    auto *vpe = its.findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        warn("ITS VSYNC rejected: vPEID %u not mapped\n", vpeId);
        its.incrementReadPointer();
        terminate(yield);
    }

    /*
     * 最小 VSYNC 语义：
     * 强制丢弃目标 RD 的 vLPI 配置缓存，使后续注入读取到新属性。
     * 与 VINVALL 一起构成“显式同步后生效”的可验证路径。
     */
    its.getRedistributor(vpe->rdBase)->invalidateVLPIConfigAll();
}

void
ItsCommand::vsgi(Yield &yield, CommandEntry &command)
{
    const uint16_t vpeId = bits(command.raw[1], 15, 0);
    const uint32_t vintId = bits(command.raw[1], 31, 16);
    const bool enable = bits(command.raw[2], 0);
    const bool group1Ns = bits(command.raw[2], 1);
    const bool clear = bits(command.raw[2], 2);
    const uint8_t priority = bits(command.raw[2], 15, 8);

    if (its.vpeOutOfRange(vpeId)) {
        warn("ITS VSGI rejected: vPEID %u out of range (max %u)\n",
             vpeId, Gicv3Its::MAX_VPEID);
        its.incrementReadPointer();
        terminate(yield);
    }

    auto *vpe = its.findVPE(vpeId);
    if (!vpe || !vpe->valid) {
        warn("ITS VSGI rejected: vPEID %u not mapped\n", vpeId);
        its.incrementReadPointer();
        terminate(yield);
    }

    if (vintId >= Gicv3::SGI_MAX) {
        warn("ITS VSGI rejected: vINTID %u out of SGI range\n", vintId);
        its.incrementReadPointer();
        terminate(yield);
    }

    /*
     * 第一阶段统一策略：仅支持 Group1NS，其他 group 直接 reject，
     * 避免软件误以为 Group0/Group1S 已被建模。
     */
    if (!group1Ns) {
        warn("ITS VSGI rejected: only Group1NS is supported in stage-1\n");
        its.incrementReadPointer();
        terminate(yield);
    }

    its.getRedistributor(vpe->rdBase)->configureVirtualSGI(
        vpeId, vpe->vptAddr, vpe->vptIdBits, vintId, enable,
        Gicv3::G1NS, priority, clear);
}

Gicv3Its::Gicv3Its(const Gicv3ItsParams &params)
 : BasicPioDevice(params, params.pio_size),
   dmaPort(name() + ".dma", *this),
   gitsControl(CTLR_QUIESCENT),
   gitsTyper(params.gits_typer),
   gitsCbaser(0), gitsCreadr(0),
   gitsCwriter(0), gitsIidr(0),
   gitsTranslater(0), gitsSgir(0),
   tableBases(NUM_BASER_REGS, 0),
   requestorId(params.system->getRequestorId(this)),
   gic(nullptr),
   commandEvent([this] { checkCommandQueue(); }, name()),
   pendingCommands(false),
   pendingTranslations(0)
{
    BASER device_baser = 0;
    device_baser.type = DEVICE_TABLE;
    device_baser.entrySize = sizeof(uint64_t) - 1;
    tableBases[0] = device_baser;

    BASER icollect_baser = 0;
    icollect_baser.type = COLLECTION_TABLE;
    icollect_baser.entrySize = sizeof(uint64_t) - 1;
    tableBases[1] = icollect_baser;
}

void
Gicv3Its::setGIC(Gicv3 *_gic)
{
    assert(!gic);
    gic = _gic;
}

AddrRangeList
Gicv3Its::getAddrRanges() const
{
    assert(pioSize != 0);
    AddrRangeList ranges;
    DPRINTF(AddrRanges, "registering range: %#x-%#x\n", pioAddr, pioSize);
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

Tick
Gicv3Its::read(PacketPtr pkt)
{
    const Addr addr = pkt->getAddr() - pioAddr;
    uint64_t value = 0;

    DPRINTF(GIC, "%s register at addr: %#x\n", __func__, addr);

    switch (addr) {
      case GITS_CTLR:
        value = gitsControl;
        break;

      case GITS_IIDR:
        value = gitsIidr;
        break;

      case GITS_TYPER:
        value = gitsTyper;
        break;

      case GITS_TYPER + 4:
        value = gitsTyper.high;
        break;

      case GITS_CBASER:
        value = gitsCbaser;
        break;

      case GITS_CBASER + 4:
        value = gitsCbaser.high;
        break;

      case GITS_CWRITER:
        value = gitsCwriter;
        break;

      case GITS_CWRITER + 4:
        value = gitsCwriter.high;
        break;

      case GITS_CREADR:
        value = gitsCreadr;
        break;

      case GITS_CREADR + 4:
        value = gitsCreadr.high;
        break;

      case GITS_PIDR2:
        value = gic->getDistributor()->gicdPidr2;
        break;

      case GITS_TRANSLATER:
        value = gitsTranslater;
        break;

      case GITS_SGIR:
        value = gitsSgir;
        break;

      case GITS_SGIR + 4:
        value = bits(gitsSgir, 63, 32);
        break;

      default:
        if (GITS_BASER.contains(addr)) {
            auto relative_addr = addr - GITS_BASER.start();
            auto baser_index = relative_addr / sizeof(uint64_t);

            value = tableBases[baser_index];
            break;
        } else {
            panic("Unrecognized register access\n");
        }
    }

    pkt->setUintX(value, ByteOrder::little);
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Gicv3Its::write(PacketPtr pkt)
{
    Addr addr = pkt->getAddr() - pioAddr;

    DPRINTF(GIC, "%s register at addr: %#x\n", __func__, addr);

    switch (addr) {
      case GITS_CTLR:
        assert(pkt->getSize() == sizeof(uint32_t));
        gitsControl = (pkt->getLE<uint32_t>() & ~CTLR_QUIESCENT);
        // We should check here if the ITS has been disabled, and if
        // that's the case, flush GICv3 caches to external memory.
        // This is not happening now, since LPI caching is not
        // currently implemented in gem5.
        break;

      case GITS_IIDR:
        panic("GITS_IIDR is Read Only\n");

      case GITS_TYPER:
        panic("GITS_TYPER is Read Only\n");

      case GITS_CBASER:
        if (pkt->getSize() == sizeof(uint32_t)) {
            gitsCbaser.low = pkt->getLE<uint32_t>();
        } else {
            assert(pkt->getSize() == sizeof(uint64_t));
            gitsCbaser = pkt->getLE<uint64_t>();
        }

        gitsCreadr = 0; // Cleared when CBASER gets written

        checkCommandQueue();
        break;

      case GITS_CBASER + 4:
        assert(pkt->getSize() == sizeof(uint32_t));
        gitsCbaser.high = pkt->getLE<uint32_t>();

        gitsCreadr = 0; // Cleared when CBASER gets written

        checkCommandQueue();
        break;

      case GITS_CWRITER:
        if (pkt->getSize() == sizeof(uint32_t)) {
            gitsCwriter.low = pkt->getLE<uint32_t>();
        } else {
            assert(pkt->getSize() == sizeof(uint64_t));
            gitsCwriter = pkt->getLE<uint64_t>();
        }

        checkCommandQueue();
        break;

      case GITS_CWRITER + 4:
        assert(pkt->getSize() == sizeof(uint32_t));
        gitsCwriter.high = pkt->getLE<uint32_t>();

        checkCommandQueue();
        break;

      case GITS_CREADR:
        panic("GITS_READR is Read Only\n");

      case GITS_TRANSLATER:
        if (gitsControl.enabled) {
            translate(pkt);
        }
        break;

      case GITS_SGIR:
        if (pkt->getSize() == sizeof(uint32_t)) {
            gitsSgir = (gitsSgir & 0xffffffff00000000ULL) |
                       pkt->getLE<uint32_t>();
        } else {
            assert(pkt->getSize() == sizeof(uint64_t));
            gitsSgir = pkt->getLE<uint64_t>();
        }
        if (gitsControl.enabled) {
            const uint16_t vpeId = bits(gitsSgir, 15, 0);
            const uint32_t vintId = bits(gitsSgir, 19, 16);
            sendVirtualSGI(vpeId, vintId);
        }
        break;

      case GITS_SGIR + 4:
        assert(pkt->getSize() == sizeof(uint32_t));
        gitsSgir = insertBits(gitsSgir, 63, 32, pkt->getLE<uint32_t>());
        if (gitsControl.enabled) {
            const uint16_t vpeId = bits(gitsSgir, 15, 0);
            const uint32_t vintId = bits(gitsSgir, 19, 16);
            sendVirtualSGI(vpeId, vintId);
        }
        break;

      default:
        if (GITS_BASER.contains(addr)) {
            auto relative_addr = addr - GITS_BASER.start();
            auto baser_index = relative_addr / sizeof(uint64_t);

            const uint64_t table_base = tableBases[baser_index];
            const uint64_t w_mask = tableBases[baser_index].type ?
                BASER_WMASK : BASER_WMASK_UNIMPL;
            const uint64_t val = pkt->getLE<uint64_t>() & w_mask;

            tableBases[baser_index] = table_base | val;
            break;
        } else {
            panic("Unrecognized register access\n");
        }
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

bool
Gicv3Its::idOutOfRange(uint32_t event_id, uint8_t itt_range) const
{
    const uint32_t id_bits = gitsTyper.idBits;
    return event_id >= (1ULL << (id_bits + 1)) ||
        event_id >= ((1ULL << itt_range) + 1);
}

bool
Gicv3Its::deviceOutOfRange(uint32_t device_id) const
{
    return device_id >= (1ULL << (gitsTyper.devBits + 1));
}

bool
Gicv3Its::sizeOutOfRange(uint32_t size) const
{
    return size > gitsTyper.idBits;
}

bool
Gicv3Its::collectionOutOfRange(uint32_t collection_id) const
{
    // If GITS_TYPER.CIL == 0, ITS supports 16-bit CollectionID
    // Otherwise, #bits is specified by GITS_TYPER.CIDbits
    const auto cid_bits = gitsTyper.cil == 0 ?
        16 : gitsTyper.cidBits + 1;

    return collection_id >= (1ULL << cid_bits);
}

bool
Gicv3Its::lpiOutOfRange(uint32_t intid) const
{
    return intid >= (1ULL << (Gicv3Distributor::IDBITS + 1)) ||
           (intid < Gicv3Redistributor::SMALLEST_LPI_ID &&
            intid != Gicv3::INTID_SPURIOUS);
}

bool
Gicv3Its::vpeOutOfRange(uint32_t vpeId) const
{
    // 统一 vPEID 上限判断，供所有虚拟命令共享错误模型。
    return vpeId > MAX_VPEID;
}

DrainState
Gicv3Its::drain()
{
    if (!pendingCommands && !pendingTranslations) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "GICv3 ITS not drained\n");
        return DrainState::Draining;
    }
}

void
Gicv3Its::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(gitsControl);
    SERIALIZE_SCALAR(gitsTyper);
    SERIALIZE_SCALAR(gitsCbaser);
    SERIALIZE_SCALAR(gitsCreadr);
    SERIALIZE_SCALAR(gitsCwriter);
    SERIALIZE_SCALAR(gitsIidr);
    SERIALIZE_SCALAR(gitsTranslater);
    SERIALIZE_SCALAR(gitsSgir);
    SERIALIZE_CONTAINER(tableBases);
    const uint32_t numVirtualPes = virtualPes.size();
    SERIALIZE_SCALAR(numVirtualPes);
    uint32_t virtualPeIndex = 0;
    for (const auto &entry : virtualPes) {
        paramOut(cp, csprintf("virtualPes_%u_vpeid", virtualPeIndex), entry.first);
        paramOut(cp, csprintf("virtualPes_%u_valid", virtualPeIndex), entry.second.valid);
        paramOut(cp, csprintf("virtualPes_%u_rdBase", virtualPeIndex), entry.second.rdBase);
        paramOut(cp, csprintf("virtualPes_%u_vptAddr", virtualPeIndex), entry.second.vptAddr);
        paramOut(cp, csprintf("virtualPes_%u_vptIdBits", virtualPeIndex), entry.second.vptIdBits);
        paramOut(cp, csprintf("virtualPes_%u_doorbellIntid", virtualPeIndex), entry.second.doorbellIntid);
        paramOut(cp, csprintf("virtualPes_%u_defaultDoorbellPending",
                virtualPeIndex), entry.second.defaultDoorbellPending);
        virtualPeIndex++;
    }
    const uint32_t numVirtualIrqs = virtualIrqs.size();
    SERIALIZE_SCALAR(numVirtualIrqs);
    uint32_t virtualIrqIndex = 0;
    for (const auto &entry : virtualIrqs) {
        paramOut(cp, csprintf("virtualIrqs_%u_key", virtualIrqIndex), entry.first);
        paramOut(cp, csprintf("virtualIrqs_%u_valid", virtualIrqIndex), entry.second.valid);
        paramOut(cp, csprintf("virtualIrqs_%u_vintid", virtualIrqIndex), entry.second.vintid);
        paramOut(cp, csprintf("virtualIrqs_%u_doorbellIntid", virtualIrqIndex), entry.second.doorbellIntid);
        paramOut(cp, csprintf("virtualIrqs_%u_vpeid", virtualIrqIndex), entry.second.vpeid);
        paramOut(cp, csprintf("virtualIrqs_%u_group", virtualIrqIndex), entry.second.group);
        virtualIrqIndex++;
    }
}
void
Gicv3Its::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(gitsControl);
    UNSERIALIZE_SCALAR(gitsTyper);
    UNSERIALIZE_SCALAR(gitsCbaser);
    UNSERIALIZE_SCALAR(gitsCreadr);
    UNSERIALIZE_SCALAR(gitsCwriter);
    UNSERIALIZE_SCALAR(gitsIidr);
    UNSERIALIZE_SCALAR(gitsTranslater);
    UNSERIALIZE_SCALAR(gitsSgir);
    UNSERIALIZE_CONTAINER(tableBases);
    virtualPes.clear();
    uint32_t numVirtualPes;
    UNSERIALIZE_SCALAR(numVirtualPes);
    for (uint32_t i = 0; i < numVirtualPes; ++i) {
        uint16_t vpeid;
        VirtualPE vpe;
        paramIn(cp, csprintf("virtualPes_%u_vpeid", i), vpeid);
        paramIn(cp, csprintf("virtualPes_%u_valid", i), vpe.valid);
        paramIn(cp, csprintf("virtualPes_%u_rdBase", i), vpe.rdBase);
        paramIn(cp, csprintf("virtualPes_%u_vptAddr", i), vpe.vptAddr);
        paramIn(cp, csprintf("virtualPes_%u_vptIdBits", i), vpe.vptIdBits);
        paramIn(cp, csprintf("virtualPes_%u_doorbellIntid", i), vpe.doorbellIntid);
        paramIn(cp, csprintf("virtualPes_%u_defaultDoorbellPending", i),
                vpe.defaultDoorbellPending);
        virtualPes[vpeid] = vpe;
    }
    virtualIrqs.clear();
    uint32_t numVirtualIrqs;
    UNSERIALIZE_SCALAR(numVirtualIrqs);
    for (uint32_t i = 0; i < numVirtualIrqs; ++i) {
        uint64_t key;
        VirtualIrqEntry irq;
        paramIn(cp, csprintf("virtualIrqs_%u_key", i), key);
        paramIn(cp, csprintf("virtualIrqs_%u_valid", i), irq.valid);
        paramIn(cp, csprintf("virtualIrqs_%u_vintid", i), irq.vintid);
        paramIn(cp, csprintf("virtualIrqs_%u_doorbellIntid", i), irq.doorbellIntid);
        paramIn(cp, csprintf("virtualIrqs_%u_vpeid", i), irq.vpeid);
        paramIn(cp, csprintf("virtualIrqs_%u_group", i), irq.group);
        virtualIrqs[key] = irq;
    }
}

void
Gicv3Its::incrementReadPointer()
{
    // Make the reader point to the next element
    gitsCreadr.offset = gitsCreadr.offset + 1;

    // Check for wrapping
    if (gitsCreadr.offset == maxCommands()) {
        gitsCreadr.offset = 0;
    }
}

uint64_t
Gicv3Its::maxCommands() const
{
    return (4096 * (gitsCbaser.size + 1)) / sizeof(ItsCommand::CommandEntry);
}

void
Gicv3Its::checkCommandQueue()
{
    if (!gitsControl.enabled || !gitsCbaser.valid)
        return;

    // If GITS_CWRITER gets set by sw to a value bigger than the
    // allowed one, the command queue should stop processing commands
    // until the register gets reset to an allowed one
    if (gitsCwriter.offset >= maxCommands()) {
        return;
    }

    if (gitsCwriter.offset != gitsCreadr.offset) {
        // writer and reader pointing to different command
        // entries: queue not empty.
        DPRINTF(ITS, "Reading command from queue\n");

        if (!pendingCommands) {
            auto *cmd_proc = new ItsCommand(*this);

            runProcess(cmd_proc, nullptr);
        } else {
            DPRINTF(ITS, "Waiting for pending command to finish\n");
        }
    }
}

Port &
Gicv3Its::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma") {
        return dmaPort;
    }
    return BasicPioDevice::getPort(if_name, idx);
}

void
Gicv3Its::recvReqRetry()
{
    assert(!packetsToRetry.empty());

    while (!packetsToRetry.empty()) {
        ItsAction a = packetsToRetry.front();

        assert(a.type == ItsActionType::SEND_REQ);

        if (!dmaPort.sendTimingReq(a.pkt))
            break;

        packetsToRetry.pop();
    }
}

bool
Gicv3Its::recvTimingResp(PacketPtr pkt)
{
    // @todo: We need to pay for this and not just zero it out
    pkt->headerDelay = pkt->payloadDelay = 0;

    ItsProcess *proc =
        safe_cast<ItsProcess *>(pkt->popSenderState());

    runProcessTiming(proc, pkt);

    return true;
}

ItsAction
Gicv3Its::runProcess(ItsProcess *proc, PacketPtr pkt)
{
    if (sys->isAtomicMode()) {
        return runProcessAtomic(proc, pkt);
    } else if (sys->isTimingMode()) {
        return runProcessTiming(proc, pkt);
    } else {
        panic("Not in timing or atomic mode\n");
    }
}

ItsAction
Gicv3Its::runProcessTiming(ItsProcess *proc, PacketPtr pkt)
{
    ItsAction action = proc->run(pkt);

    switch (action.type) {
      case ItsActionType::SEND_REQ:
        action.pkt->pushSenderState(proc);

        if (packetsToRetry.empty() &&
            dmaPort.sendTimingReq(action.pkt)) {

        } else {
            packetsToRetry.push(action);
        }
        break;

      case ItsActionType::TERMINATE:
        delete proc;
        if (!pendingCommands && !commandEvent.scheduled()) {
            schedule(commandEvent, clockEdge());
        }
        break;

      default:
        panic("Unknown action\n");
    }

    return action;
}

ItsAction
Gicv3Its::runProcessAtomic(ItsProcess *proc, PacketPtr pkt)
{
    ItsAction action;
    Tick delay = 0;
    bool terminate = false;

    do {
        action = proc->run(pkt);

        switch (action.type) {
          case ItsActionType::SEND_REQ:
            delay += dmaPort.sendAtomic(action.pkt);
            pkt = action.pkt;
            break;

          case ItsActionType::TERMINATE:
            delete proc;
            terminate = true;
            break;

          default:
            panic("Unknown action\n");
        }

    } while (!terminate);

    action.delay = delay;

    return action;
}

void
Gicv3Its::translate(PacketPtr pkt)
{
    DPRINTF(ITS, "Starting Translation Request\n");

    auto *proc = new ItsTranslation(*this);
    runProcess(proc, pkt);
}

Gicv3Redistributor*
Gicv3Its::getRedistributor(uint64_t rd_base)
{
    if (gitsTyper.pta == 1) {
        // RDBase is a redistributor address
        return gic->getRedistributorByAddr(rd_base << 16);
    } else {
        // RDBase is a redistributor number
        return gic->getRedistributor(rd_base);
    }
}

Addr
Gicv3Its::pageAddress(Gicv3Its::ItsTables table)
{
    auto base_it = std::find_if(
        tableBases.begin(), tableBases.end(),
        [table] (const BASER &b) { return b.type == table; }
    );

    panic_if(base_it == tableBases.end(),
        "ITS Table not recognised\n");

    const BASER base = *base_it;

    // real address depends on page size
    switch (base.pageSize) {
      case SIZE_4K:
      case SIZE_16K:
        return mbits(base, 47, 12);
      case SIZE_64K:
        return mbits(base, 47, 16) | (bits(base, 15, 12) << 48);
      default:
        panic("Unsupported page size\n");
    }
}

void
Gicv3Its::moveAllPendingState(
    Gicv3Redistributor *rd1, Gicv3Redistributor *rd2)
{
    const uint64_t largest_lpi_id = 1ULL << (rd1->lpiIDBits + 1);
    const uint64_t table_size = largest_lpi_id / 8;
    auto lpi_pending_table = std::make_unique<uint8_t[]>(table_size);

    // Copying the pending table from redistributor 1 to redistributor 2
    rd1->memProxy->readBlob(
        rd1->lpiPendingTablePtr, lpi_pending_table.get(),
        table_size);

    rd2->memProxy->writeBlob(
        rd2->lpiPendingTablePtr, lpi_pending_table.get(),
        table_size);

    // Clearing pending table in redistributor 2
    rd1->memProxy->memsetBlob(
        rd1->lpiPendingTablePtr, 0,
        table_size);

    rd2->updateDistributor();
}

} // namespace gem5
