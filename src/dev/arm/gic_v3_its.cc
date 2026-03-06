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

namespace
{

inline void
packITTE(const ItsProcess::ITTE &itte, uint64_t *raw)
{
    raw[0] = itte.intNum;
    raw[1] = itte.intNumHyp;
    raw[2] = itte.vpeid;
    raw[3] = itte.icid;
    replaceBits(raw[3], 32, 32, itte.intType);
    replaceBits(raw[3], 33, 33, itte.valid ? 1 : 0);
}

inline ItsProcess::ITTE
unpackITTE(const uint64_t *raw)
{
    ItsProcess::ITTE itte;
    itte.intNum = raw[0];
    itte.intNumHyp = raw[1];
    itte.vpeid = raw[2];
    itte.icid = bits(raw[3], 31, 0);
    itte.intType = bits(raw[3], 32, 32);
    itte.valid = bits(raw[3], 33, 33);
    return itte;
}

inline void
packVPETE(const ItsProcess::VPETE &vpete, uint64_t *raw)
{
    raw[0] = vpete.rdBase;
    raw[1] = vpete.vptAddr;
    raw[2] = vpete.valid ? 1 : 0;
}

inline ItsProcess::VPETE
unpackVPETE(const uint64_t *raw)
{
    ItsProcess::VPETE vpete;
    vpete.rdBase = raw[0];
    vpete.vptAddr = raw[1];
    vpete.valid = raw[2] & 0x1;
    return vpete;
}

} // anonymous namespace

const AddrRange Gicv3Its::GITS_BASER(0x0100, 0x0140);

const uint32_t Gicv3Its::CTLR_QUIESCENT = 0x80000000;

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
    uint64_t raw[4];
    packITTE(itte, raw);

    const Addr address = itt_base + (event_id * sizeof(raw));

    doWrite(yield, address, raw, sizeof(raw));

    DPRINTF(ITS,
            "Writing ITTE at address %#x: intNum=%#x intNumHyp=%#x vpeid=%#x "
            "icid=%#x type=%u valid=%u\n",
            address, itte.intNum, itte.intNumHyp, itte.vpeid, itte.icid,
            itte.intType, itte.valid);
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

ItsProcess::ITTE
ItsProcess::readIrqTranslationTable(
    Yield &yield, const Addr itt_base, uint32_t event_id)
{
    uint64_t raw[4] = {0, 0, 0, 0};
    const Addr address = itt_base + (event_id * sizeof(raw));

    doRead(yield, address, raw, sizeof(raw));

    ITTE itte = unpackITTE(raw);
    DPRINTF(ITS,
            "Reading ITTE at address %#x: intNum=%#x intNumHyp=%#x vpeid=%#x "
            "icid=%#x type=%u valid=%u\n",
            address, itte.intNum, itte.intNumHyp, itte.vpeid, itte.icid,
            itte.intType, itte.valid);
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

void
ItsProcess::writeVpeTable(Yield &yield, uint32_t vpe_id, VPETE vpete)
{
    uint64_t raw[3];
    packVPETE(vpete, raw);

    const Addr base = its.pageAddress(Gicv3Its::VPE_TABLE);
    const Addr address = base + (vpe_id * sizeof(raw));

    doWrite(yield, address, raw, sizeof(raw));

    DPRINTF(ITS,
            "Writing VPETE at address %#x: rdBase=%#x vptAddr=%#x valid=%u\n",
            address, vpete.rdBase, vpete.vptAddr, vpete.valid);
}

ItsProcess::VPETE
ItsProcess::readVpeTable(Yield &yield, uint32_t vpe_id)
{
    uint64_t raw[3] = {0, 0, 0};
    const Addr base = its.pageAddress(Gicv3Its::VPE_TABLE);
    const Addr address = base + (vpe_id * sizeof(raw));

    doRead(yield, address, raw, sizeof(raw));

    VPETE vpete = unpackVPETE(raw);
    DPRINTF(ITS,
            "Reading VPETE at address %#x: rdBase=%#x vptAddr=%#x valid=%u\n",
            address, vpete.rdBase, vpete.vptAddr, vpete.valid);
    return vpete;
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

    uint32_t device_id = 0;
    if (pkt->req->hasStreamId()) {
        device_id = pkt->req->streamId();
    } else {
        // Some software-driven tests write GITS_TRANSLATER from the CPU side,
        // which does not carry a PCI/MSI stream ID in the Request object.
        // Fall back to device_id 0 instead of asserting.
        warn_once("ITS translation request without StreamID; using device_id=0");
    }

    const uint32_t event_id = pkt->getLE<uint32_t>();

    auto result = translateLPI(yield, device_id, event_id);

    const ITTE &itte = result.itte;
    Gicv3Redistributor *redist = result.redist;

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        if (its.directVlpi) {
            // Direct injection path: write to the guest vPE pending table.
            DPRINTF(ITS, "vLPI direct inject vINTID=%u\n", itte.intNum);
            redist->setClrVLPI(itte.intNum, result.vpeid, result.vptAddr, true);
        } else {
            // Trap-based path: inject pINTID to EL2, so SW can reflect to EL1.
            DPRINTF(ITS, "vLPI trap inject pINTID=%u for vINTID=%u\n",
                    itte.intNumHyp, itte.intNum);
            redist->setClrLPI(itte.intNumHyp, true);
        }
    } else {
        // Set the physical LPI in the redistributor.
        redist->setClrLPI(itte.intNum, true);
    }

    // Update the value in GITS_TRANSLATER only once we know
    // there was no error in the tranlation process (before
    // terminating the translation
    its.gitsTranslater = event_id;

    terminate(yield);
}

ItsTranslation::TranslationResult
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
    const auto collection_id = itte.icid;

    if (!itte.valid) {
        terminate(yield);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        VPETE vpete = readVpeTable(yield, itte.vpeid);

        if (!vpete.valid) {
            terminate(yield);
        }

        return TranslationResult{itte, its.getRedistributor(vpete.rdBase), vpete.vptAddr, itte.vpeid};
    }

    if (its.collectionOutOfRange(collection_id)) {
        terminate(yield);
    }

    CTE cte = readIrqCollectionTable(yield, collection_id);

    if (!cte.valid) {
        terminate(yield);
    }

    // Return the translated ITTE and target Redistributor.
    return TranslationResult{itte, its.getRedistributor(cte), 0, 0};
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

    ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        VPETE vpete = readVpeTable(yield, itte.vpeid);
        if (!vpete.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        auto *rd = its.getRedistributor(vpete.rdBase);
        if (its.directVlpi) {
            rd->setClrVLPI(itte.intNum, itte.vpeid, vpete.vptAddr, false);
        } else {
            rd->setClrLPI(itte.intNumHyp, false);
        }
    } else {
        const auto collection_id = itte.icid;
        CTE cte = readIrqCollectionTable(yield, collection_id);

        if (!cte.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        // Clear the LPI in the redistributor
        its.getRedistributor(cte)->setClrLPI(itte.intNum, false);
    }
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

    ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        VPETE vpete = readVpeTable(yield, itte.vpeid);
        if (!vpete.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        auto *rd = its.getRedistributor(vpete.rdBase);
        if (its.directVlpi) {
            rd->setClrVLPI(itte.intNum, itte.vpeid, vpete.vptAddr, false);
        } else {
            rd->setClrLPI(itte.intNumHyp, false);
        }
    } else {
        const auto collection_id = itte.icid;
        CTE cte = readIrqCollectionTable(yield, collection_id);

        if (!cte.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        its.getRedistributor(cte)->setClrLPI(itte.intNum, false);
    }

    // Then removes the mapping from the ITT (invalidating)
    itte.valid = 0;
    writeIrqTranslationTable(
        yield, dte.ittAddress, command.eventId, itte);
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

    ITTE itte = readIrqTranslationTable(
        yield, dte.ittAddress, command.eventId);

    if (!itte.valid) {
        its.incrementReadPointer();
        terminate(yield);
    }

    Gicv3Redistributor *redist = nullptr;
    VPETE vpete;

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        vpete = readVpeTable(yield, itte.vpeid);

        if (!vpete.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        redist = its.getRedistributor(vpete.rdBase);
    } else {
        const auto collection_id = itte.icid;
        CTE cte = readIrqCollectionTable(yield, collection_id);

        if (!cte.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }

        redist = its.getRedistributor(cte);
    }

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        if (its.directVlpi) {
            redist->setClrVLPI(itte.intNum, itte.vpeid, vpete.vptAddr, true);
        } else {
            redist->setClrLPI(itte.intNumHyp, true);
        }
    } else {
        redist->setClrLPI(itte.intNum, true);
    }
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

    if (itte.intType == Gicv3Its::VIRTUAL_INTERRUPT) {
        its.markVpeDirty(itte.vpeid & 0xFFFF);
    } else {
        const auto collection_id = itte.icid;
        CTE cte = readIrqCollectionTable(yield, collection_id);

        if (!cte.valid) {
            its.incrementReadPointer();
            terminate(yield);
        }
        // Do nothing since caching is currently not supported in
        // Redistributor
    }
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
    warn("ITS %s command unimplemented", __func__);
}

void
ItsCommand::vinvall(Yield &yield, CommandEntry &command)
{
    (void)yield;
    const uint32_t vpe_id = bits(command.raw[1], 47, 32);
    its.markVpeDirty(vpe_id & 0xFFFF);
}

void
ItsCommand::vmapi(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);

    if (!dte.valid || idOutOfRange(command, dte) ||
        its.lpiOutOfRange(command.eventId)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);

    itte.valid = 1;
    itte.intType = Gicv3Its::VIRTUAL_INTERRUPT;
    itte.intNum = command.eventId;
    itte.intNumHyp = bits(command.raw[2], 63, 32);
    itte.vpeid = bits(command.raw[1], 47, 32);

    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);
}

void
ItsCommand::vmapp(Yield &yield, CommandEntry &command)
{
    // 中文说明：按 IHI0069 重新解析 VMAPP 位域。
    const uint32_t vpe_id = bits(command.raw[1], 47, 32);
    const uint64_t rd_base = bits(command.raw[2], 51, 16);
    const uint64_t vpt_addr = bits(command.raw[3], 51, 16);
    const bool valid = bits(command.raw[2], 63);

    VPETE vpete;
    vpete.valid = valid;
    vpete.rdBase = rd_base;
    vpete.vptAddr = vpt_addr;

    writeVpeTable(yield, vpe_id, vpete);
}

void
ItsCommand::vmapti(Yield &yield, CommandEntry &command)
{
    if (deviceOutOfRange(command)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    DTE dte = readDeviceTable(yield, command.deviceId);

    const uint32_t pintid = bits(command.raw[2], 63, 32);
    const uint32_t vintid = bits(command.raw[2], 31, 0);
    const uint32_t vpeid = bits(command.raw[1], 47, 32);

    if (!dte.valid || idOutOfRange(command, dte) ||
        its.lpiOutOfRange(vintid) || its.lpiOutOfRange(pintid)) {
        its.incrementReadPointer();
        terminate(yield);
    }

    ITTE itte = readIrqTranslationTable(yield, dte.ittAddress, command.eventId);

    itte.valid = 1;
    itte.intType = Gicv3Its::VIRTUAL_INTERRUPT;
    itte.intNum = vintid;
    itte.intNumHyp = pintid;
    itte.vpeid = vpeid;

    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);
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

    const auto new_vpe = bits(command.raw[1], 47, 32);
    itte.vpeid = new_vpe;

    writeIrqTranslationTable(yield, dte.ittAddress, command.eventId, itte);
}

void
ItsCommand::vmovp(Yield &yield, CommandEntry &command)
{
    const uint32_t vpe_id = bits(command.raw[1], 47, 32);
    const uint64_t rd_base = bits(command.raw[2], 51, 16);

    VPETE vpete = readVpeTable(yield, vpe_id);
    if (!vpete.valid) {
        return;
    }

    vpete.rdBase = rd_base;
    writeVpeTable(yield, vpe_id, vpete);
}

void
ItsCommand::vsync(Yield &yield, CommandEntry &command)
{
    (void)yield;
    (void)command;
    // No op while no virtual caching model is present.
}


Gicv3Its::Gicv3Its(const Gicv3ItsParams &params)
 : BasicPioDevice(params, params.pio_size),
   dmaPort(name() + ".dma", *this),
   gitsControl(CTLR_QUIESCENT),
   gitsTyper(params.gits_typer),
   gitsCbaser(0), gitsCreadr(0),
   gitsCwriter(0), gitsIidr(0),
   tableBases(NUM_BASER_REGS, 0),
   requestorId(params.system->getRequestorId(this)),
   gic(nullptr),
   commandEvent([this] { checkCommandQueue(); }, name()),
   pendingCommands(false),
   pendingTranslations(0),
   directVlpi(params.direct_vlpi)
{
    BASER device_baser = 0;
    device_baser.type = DEVICE_TABLE;
    device_baser.entrySize = sizeof(uint64_t) - 1;
    tableBases[0] = device_baser;

    BASER vpe_baser = 0;
    vpe_baser.type = VPE_TABLE;
    vpe_baser.entrySize = (sizeof(uint64_t) * 3) - 1;
    tableBases[1] = vpe_baser;

    BASER icollect_baser = 0;
    icollect_baser.type = COLLECTION_TABLE;
    icollect_baser.entrySize = sizeof(uint64_t) - 1;
    tableBases[2] = icollect_baser;
}


void
Gicv3Its::setGIC(Gicv3 *_gic)
{
    assert(!gic);
    gic = _gic;

    // Advertise v4-capable virtualization features on GIC-700 style systems.
    if (gic->gicv4Enabled()) {
        gitsTyper.vmovp = 1;
        gitsTyper._virtual = 1;
        gitsTyper.physical = 1;
        gitsTyper.seis = 1;
        gitsTyper.cct = 1;
        gitsTyper.vsgi = 1;
    }
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
    SERIALIZE_SCALAR(directVlpi);

    SERIALIZE_CONTAINER(tableBases);
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
    UNSERIALIZE_SCALAR(directVlpi);

    UNSERIALIZE_CONTAINER(tableBases);
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

bool
Gicv3Its::readVpe(uint32_t vpe_id, VPETE &vpete)
{
    auto base_it = std::find_if(
        tableBases.begin(), tableBases.end(),
        [] (const BASER &b) { return b.type == VPE_TABLE; }
    );

    if (base_it == tableBases.end()) {
        return false;
    }

    const Addr base = pageAddress(VPE_TABLE);
    const Addr address = base + (vpe_id * (sizeof(uint64_t) * 3));
    uint64_t raw[3] = {0, 0, 0};
    gic->getSystem()->physProxy.readBlob(address, raw, sizeof(raw));

    vpete.rdBase = raw[0];
    vpete.vptAddr = raw[1];
    vpete.valid = raw[2] & 0x1;
    return true;
}

void
Gicv3Its::markVpeDirty(uint16_t vpe_id)
{
    if (!gic) {
        return;
    }

    for (int i = 0; i < gic->getSystem()->threads.size(); i++) {
        auto *rd = gic->getRedistributor(i);
        if (rd->vpeResident && ((rd->residentVpeId & 0xFFFF) == vpe_id)) {
            rd->markDirectVlpiDirty();
            rd->updateDistributor();
            break;
        }
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
