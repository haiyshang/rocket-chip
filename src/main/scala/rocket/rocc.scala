// See LICENSE for license details.

package rocket

import Chisel._
import uncore.tilelink._
import uncore.constants._
import uncore.agents.CacheName
import util._
import Chisel.ImplicitConversions._
import cde.{Parameters, Field}

case object RoccMaxTaggedMemXacts extends Field[Int]
case object RoccNMemChannels extends Field[Int]
case object RoccNPTWPorts extends Field[Int]

class RoCCInstruction extends Bundle
{
  val funct = Bits(width = 7)
  val rs2 = Bits(width = 5)
  val rs1 = Bits(width = 5)
  val xd = Bool()
  val xs1 = Bool()
  val xs2 = Bool()
  val rd = Bits(width = 5)
  val opcode = Bits(width = 7)
}

class RoCCCommand(implicit p: Parameters) extends CoreBundle()(p) {
  val inst = new RoCCInstruction
  val rs1 = Bits(width = xLen)
  val rs2 = Bits(width = xLen)
  val status = new MStatus
}

class RoCCResponse(implicit p: Parameters) extends CoreBundle()(p) {
  val rd = Bits(width = 5)
  val data = Bits(width = xLen)
}

class RoCCInterface(implicit p: Parameters) extends CoreBundle()(p) {
  val cmd = Decoupled(new RoCCCommand).flip
  val resp = Decoupled(new RoCCResponse)
  val mem = new HellaCacheIO()(p.alterPartial({ case CacheName => "L1D" }))
  val busy = Bool(OUTPUT)
  val interrupt = Bool(OUTPUT)

  // These should be handled differently, eventually
  val autl = new ClientUncachedTileLinkIO
  val utl = Vec(p(RoccNMemChannels), new ClientUncachedTileLinkIO)
  val ptw = Vec(p(RoccNPTWPorts), new TLBPTWIO)
  val fpu_req = Decoupled(new FPInput)
  val fpu_resp = Decoupled(new FPResult).flip
  val exception = Bool(INPUT)

  override def cloneType = new RoCCInterface().asInstanceOf[this.type]
}

abstract class RoCC(implicit p: Parameters) extends CoreModule()(p) {
  val io = new RoCCInterface
  io.mem.req.bits.phys := Bool(true) // don't perform address translation
  io.mem.invalidate_lr := Bool(false) // don't mess with LR/SC
}

class AccumulatorExample(n: Int = 4)(implicit p: Parameters) extends RoCC()(p) {
  val regfile = Mem(n, UInt(width = xLen))
  val busy = Reg(init = Vec.fill(n){Bool(false)})

  val cmd = Queue(io.cmd)
  val funct = cmd.bits.inst.funct
  val addr = cmd.bits.rs2(log2Up(n)-1,0)
  val doWrite = funct === UInt(0)
  val doRead = funct === UInt(1)
  val doLoad = funct === UInt(2)
  val doAccum = funct === UInt(3)
  val memRespTag = io.mem.resp.bits.tag(log2Up(n)-1,0)

  // datapath
  val addend = cmd.bits.rs1
  val accum = regfile(addr)
  val wdata = Mux(doWrite, addend, accum + addend)

  when (cmd.fire() && (doWrite || doAccum)) {
    regfile(addr) := wdata
  }

  when (io.mem.resp.valid) {
    regfile(memRespTag) := io.mem.resp.bits.data
    busy(memRespTag) := Bool(false)
  }

  // control
  when (io.mem.req.fire()) {
    busy(addr) := Bool(true)
  }

  val doResp = cmd.bits.inst.xd
  val stallReg = busy(addr)
  val stallLoad = doLoad && !io.mem.req.ready
  val stallResp = doResp && !io.resp.ready

  cmd.ready := !stallReg && !stallLoad && !stallResp
    // command resolved if no stalls AND not issuing a load that will need a request

  // PROC RESPONSE INTERFACE
  io.resp.valid := cmd.valid && doResp && !stallReg && !stallLoad
    // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := cmd.bits.inst.rd
    // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := accum
    // Semantics is to always send out prior accumulator register value

  io.busy := cmd.valid || busy.reduce(_||_)
    // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
    // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := cmd.valid && doLoad && !stallReg && !stallResp
  io.mem.req.bits.addr := addend
  io.mem.req.bits.tag := addr
  io.mem.req.bits.cmd := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ := MT_D // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...

  io.autl.acquire.valid := false
  io.autl.grant.ready := false
}

class TranslatorExample(implicit p: Parameters) extends RoCC()(p) {
  val req_addr = Reg(UInt(width = coreMaxAddrBits))
  val req_rd = Reg(io.resp.bits.rd)
  val req_offset = req_addr(pgIdxBits - 1, 0)
  val req_vpn = req_addr(coreMaxAddrBits - 1, pgIdxBits)
  val pte = Reg(new PTE)

  val s_idle :: s_ptw_req :: s_ptw_resp :: s_resp :: Nil = Enum(Bits(), 4)
  val state = Reg(init = s_idle)

  io.cmd.ready := (state === s_idle)

  when (io.cmd.fire()) {
    req_rd := io.cmd.bits.inst.rd
    req_addr := io.cmd.bits.rs1
    state := s_ptw_req
  }

  private val ptw = io.ptw(0)

  when (ptw.req.fire()) { state := s_ptw_resp }

  when (state === s_ptw_resp && ptw.resp.valid) {
    pte := ptw.resp.bits.pte
    state := s_resp
  }

  when (io.resp.fire()) { state := s_idle }

  ptw.req.valid := (state === s_ptw_req)
  ptw.req.bits.addr := req_vpn
  ptw.req.bits.store := Bool(false)
  ptw.req.bits.fetch := Bool(false)

  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := req_rd
  io.resp.bits.data := Mux(pte.leaf(), Cat(pte.ppn, req_offset), SInt(-1, xLen).asUInt)

  io.busy := (state =/= s_idle)
  io.interrupt := Bool(false)
  io.mem.req.valid := Bool(false)
  io.autl.acquire.valid := Bool(false)
  io.autl.grant.ready := Bool(false)
}

class CharacterCountExample(implicit p: Parameters) extends RoCC()(p)
    with HasTileLinkParameters {

  private val blockOffset = tlBeatAddrBits + tlByteAddrBits

  val needle = Reg(UInt(width = 8))
  val addr = Reg(UInt(width = coreMaxAddrBits))
  val count = Reg(UInt(width = xLen))
  val resp_rd = Reg(io.resp.bits.rd)

  val addr_block = addr(coreMaxAddrBits - 1, blockOffset)
  val offset = addr(blockOffset - 1, 0)
  val next_addr = (addr_block + UInt(1)) << UInt(blockOffset)

  val s_idle :: s_acq :: s_gnt :: s_check :: s_resp :: Nil = Enum(Bits(), 5)
  val state = Reg(init = s_idle)

  val gnt = io.autl.grant.bits
  val recv_data = Reg(UInt(width = tlDataBits))
  val recv_beat = Reg(UInt(width = tlBeatAddrBits))

  val data_bytes = Vec.tabulate(tlDataBytes) { i => recv_data(8 * (i + 1) - 1, 8 * i) }
  val zero_match = data_bytes.map(_ === UInt(0))
  val needle_match = data_bytes.map(_ === needle)
  val first_zero = PriorityEncoder(zero_match)

  val chars_found = PopCount(needle_match.zipWithIndex.map {
    case (matches, i) =>
      val idx = Cat(recv_beat, UInt(i, tlByteAddrBits))
      matches && idx >= offset && UInt(i) <= first_zero
  })
  val zero_found = zero_match.reduce(_ || _)
  val finished = Reg(Bool())

  io.cmd.ready := (state === s_idle)
  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := resp_rd
  io.resp.bits.data := count
  io.autl.acquire.valid := (state === s_acq)
  io.autl.acquire.bits := GetBlock(addr_block = addr_block)
  io.autl.grant.ready := (state === s_gnt)

  when (io.cmd.fire()) {
    addr := io.cmd.bits.rs1
    needle := io.cmd.bits.rs2
    resp_rd := io.cmd.bits.inst.rd
    count := UInt(0)
    finished := Bool(false)
    state := s_acq
  }

  when (io.autl.acquire.fire()) { state := s_gnt }

  when (io.autl.grant.fire()) {
    recv_beat := gnt.addr_beat
    recv_data := gnt.data
    state := s_check
  }

  when (state === s_check) {
    when (!finished) {
      count := count + chars_found
    }
    when (zero_found) { finished := Bool(true) }
    when (recv_beat === UInt(tlDataBeats - 1)) {
      addr := next_addr
      state := Mux(zero_found || finished, s_resp, s_acq)
    } .otherwise {
      state := s_gnt
    }
  }

  when (io.resp.fire()) { state := s_idle }

  io.busy := (state =/= s_idle)
  io.interrupt := Bool(false)
  io.mem.req.valid := Bool(false)
}


class MatrixMul(n: Int = 4)(implicit p: Parameters) extends RoCC()(p) {

  val busy = Reg(init = {Bool(false)})

  val r_cmd_count  = Reg(UInt(width = xLen))
  val r_recv_count = Reg(UInt(width = xLen))

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr   = Reg(UInt(width = xLen))
  val r_v_addr = Reg(UInt(width = xLen))
  val r_h_addr = Reg(UInt(width = xLen))
  // datapath
  val r_total = Reg(UInt(width = xLen))
  val r_tag = Reg(UInt(width = n))

  val s_idle :: s_mem_fetch_h :: s_mem_fetch_v :: s_recv_finish :: s_mem_recv_v :: s_mem_recv_h :: Nil = Enum(Bits(), 6)
  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  val r_vfile = Mem(16, UInt(width = xLen))

  when (io.cmd.valid) {
    printf("MatrixMul: On Going. %x, %x\n", r_cmd_state, r_recv_state)
  }


  val match_last_vaddr = (r_v_addr === io.cmd.bits.rs2)
  when (io.cmd.fire()) {
    printf("MatrixMul: Command Received. %x, %x\n", io.cmd.bits.rs1, io.cmd.bits.rs2)

    r_total      := UInt(0)

    r_addr       := Mux (match_last_vaddr, io.cmd.bits.rs1, io.cmd.bits.rs2)
    r_v_addr     := io.cmd.bits.rs2
    r_h_addr     := io.cmd.bits.rs1

    r_recv_count := UInt(0)
    r_cmd_count  := UInt(0)
    r_tag        := UInt(0)

    r_resp_rd := io.cmd.bits.inst.rd

    r_cmd_state  := Mux (match_last_vaddr, s_mem_fetch_h, s_mem_fetch_v)
    r_recv_state := Mux (match_last_vaddr, s_mem_recv_h,  s_mem_recv_v)

  }

  io.cmd.ready := (r_cmd_state === s_idle)
  // command resolved if no stalls AND not issuing a load that will need a request

  val cmd_v_finished = r_cmd_count === UInt(15) // 16-1
  val cmd_h_finished = r_cmd_count === UInt(15) // 16-1

  when ((r_cmd_state === s_mem_fetch_v) && io.mem.req.fire()) {
    printf("MatrixMul: <<s_mem_fetch_v>> IO.MEM Command Fire %x\n", io.mem.resp.bits.data)

    r_cmd_count  := Mux(cmd_v_finished, UInt(0), r_cmd_count + UInt(1))
    r_addr       := Mux(cmd_v_finished, r_h_addr, r_addr + UInt(128))  // 16x8
    r_cmd_state  := Mux(cmd_v_finished, s_mem_fetch_h, s_mem_fetch_v)
  }

  when ((r_cmd_state === s_mem_fetch_h) && io.mem.req.fire()) {
    printf("MatrixMul: <<s_mem_fetch_h>> IO.MEM Command Fire %x\n", io.mem.resp.bits.data)

    r_cmd_count  := Mux(cmd_h_finished, UInt(0), r_cmd_count + UInt(1))
    r_addr       := Mux(cmd_h_finished, r_addr, r_addr + UInt(8))
    r_cmd_state  := Mux(cmd_h_finished, s_idle, s_mem_fetch_h)
  }

  when (io.mem.req.fire()) {
    r_tag        := r_tag + UInt(1)
  }

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := (r_cmd_state === s_mem_fetch_v) || (r_cmd_state === s_mem_fetch_h)
  io.mem.req.bits.addr := r_addr
  io.mem.req.bits.tag  := r_tag
  io.mem.req.bits.cmd  := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ  := MT_D  // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  val recv_v_finished = (r_recv_count === UInt(15))
  when (r_recv_state === s_mem_recv_v && io.mem.resp.fire()) {
    printf("MatrixMul: <<s_mem_recv_v>> IO.MEM Received %x\n", io.mem.resp.bits.data)

    r_recv_count := Mux(recv_v_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_state := Mux(recv_v_finished, s_mem_recv_h, s_mem_recv_v)

    r_vfile(r_recv_count) := io.mem.resp.bits.data
  }

  val recv_h_finished = (r_recv_count === UInt(15))
  when (r_recv_state === s_mem_recv_h && io.mem.resp.fire()) {
    printf("MatrixMul: <<s_mem_recv_h>> IO.MEM Received %x\n", io.mem.resp.bits.data)

    r_recv_count := Mux(recv_h_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_state := Mux(recv_h_finished, s_recv_finish, s_mem_recv_h)

    r_total := r_total + r_vfile(r_recv_count) * io.mem.resp.bits.data
  }


  // control
  when (io.mem.req.fire()) {
    busy := Bool(true)
  }

  when ((r_recv_state === s_recv_finish) && io.resp.fire()) {
    r_recv_state := s_idle
    printf("MatrixMul: Finished. Answer = %x\n", r_total)
  }

  // PROC RESPONSE INTERFACE
  io.resp.valid := (r_recv_state === s_recv_finish)
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := r_total
  // Semantics is to always send out prior accumulator register value

  io.busy      := Bool(false)
  // io.busy := io.cmd.valid
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)
}


class OpcodeSet(val opcodes: Seq[UInt]) {
  def |(set: OpcodeSet) =
    new OpcodeSet(this.opcodes ++ set.opcodes)

  def matches(oc: UInt) = opcodes.map(_ === oc).reduce(_ || _)
}

object OpcodeSet {
  val custom0 = new OpcodeSet(Seq(Bits("b0001011")))
  val custom1 = new OpcodeSet(Seq(Bits("b0101011")))
  val custom2 = new OpcodeSet(Seq(Bits("b1011011")))
  val custom3 = new OpcodeSet(Seq(Bits("b1111011")))
  val all = custom0 | custom1 | custom2 | custom3
}

class RoccCommandRouter(opcodes: Seq[OpcodeSet])(implicit p: Parameters)
    extends CoreModule()(p) {
  val io = new Bundle {
    val in = Decoupled(new RoCCCommand).flip
    val out = Vec(opcodes.size, Decoupled(new RoCCCommand))
    val busy = Bool(OUTPUT)
  }

  val cmd = Queue(io.in)
  val cmdReadys = io.out.zip(opcodes).map { case (out, opcode) =>
    val me = opcode.matches(cmd.bits.inst.opcode)
    out.valid := cmd.valid && me
    out.bits := cmd.bits
    out.ready && me
  }
  cmd.ready := cmdReadys.reduce(_ || _)
  io.busy := cmd.valid

  assert(PopCount(cmdReadys) <= UInt(1),
    "Custom opcode matched for more than one accelerator")
}


class MatrixMulTwoRequester(n: Int = 4)(implicit p: Parameters) extends RoCC()(p) {

  val busy = Reg(init = {Bool(false)})

  val setLength = funct === UInt(0)
  val doCalc    = funct === UInt(1)

  val r_cmd_count  = Reg(UInt(width = xLen))
  val r_recv_count = Reg(UInt(width = xLen))

  val r_matrix_max = Reg(UInt(width = xLen))

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr   = Reg(UInt(width = xLen))
  val r_v_addr = Reg(UInt(width = xLen))
  val r_h_addr = Reg(UInt(width = xLen))

  // datapath
  val r_total = Reg(UInt(width = xLen))
  val r_tag = Reg(UInt(width = n))

  val s_idle :: s_mem_fetch :: s_recv_finish :: s_mem_recv :: Nil = Enum(Bits(), 4)
  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  when (io.cmd.valid) {
    printf("MatrixMulTwoRequester: On Going. %x, %x\n", r_cmd_state, r_recv_state)
  }


  when (io.cmd.fire() && setLength) {
    r_matrix_max = io.cmd.bits.rs1
  }

  when (io.cmd.fire() && doCalc) {
    printf("MatrixMulTwoRequester: DoCalc Received. %x, %x\n", io.cmd.bits.rs1, io.cmd.bits.rs2)

    r_total      := UInt(0)

    r_v_addr     := io.cmd.bits.rs2
    r_h_addr     := io.cmd.bits.rs1

    r_recv_count := UInt(0)
    r_cmd_count  := UInt(0)
    r_tag        := UInt(0)

    r_resp_rd := io.cmd.bits.inst.rd

    r_cmd_state  := s_mem_fetch
    r_recv_state := s_mem_recv
  }

  val w_addr = Mux (r_cmd_count(0) == UInt(0), r_h_addr, r_v_addr)

  io.cmd.ready := (r_cmd_state === s_idle)
  // command resolved if no stalls AND not issuing a load that will need a request

  val cmd_finished = (r_cmd_count === ((r_matrix_max - UInt(1)) << UInt(1))
  when ((r_cmd_state === s_mem_fetch) && io.mem.req.fire()) {
    printf("MatrixMulTwoRequester: <<s_mem_fetch_v>> IO.MEM Command Fire %x\n", w_addr)

    r_cmd_count  := Mux(cmd_v_finished, UInt(0), r_cmd_count + UInt(1))

    r_h_addr     := Mux(r_cmd_count(0), r_h_addr + UInt(8), r_h_addr)
    r_v_addr     := Mux(r_cmd_count(0), r_v_addr, r_v_addr + (r_matrix_max << UInt(3)))
    r_cmd_state  := Mux(cmd_finished, s_idle, s_mem_fetch_v)
  }

  when (io.mem.req.fire()) {
    r_tag        := r_tag + UInt(1)
  }

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := (r_cmd_state === s_mem_fetch) || (r_cmd_state === s_mem_fetch_h)
  io.mem.req.bits.addr := w_addr
  io.mem.req.bits.tag  := r_tag
  io.mem.req.bits.cmd  := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ  := MT_D  // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  val recv_v_finished = (r_recv_count === ((r_matrix_max - UInt(1)) << UInt(1))
  when (r_recv_state === s_mem_recv && io.mem.resp.fire()) {
    printf("MatrixMulTwoRequester: <<s_mem_recv_v>> IO.MEM Received %x\n", io.mem.resp.bits.data)

    r_recv_count := Mux(recv_v_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_state := Mux(recv_v_finished, s_recv_finish, s_mem_recv)

    r_h_val      := Mux(r_recv_count(0), io.mem.resp.bits.data, r_h_val)

    r_total      := r_total + r_h_val * io.mem.resp.bits.data
  }

  val recv_h_finished = (r_recv_count === UInt(15))
  when (r_recv_state === s_mem_recv_h && io.mem.resp.fire()) {
    printf("MatrixMulTwoRequester: <<s_mem_recv_h>> IO.MEM Received %x\n", io.mem.resp.bits.data)

    r_recv_count := Mux(recv_h_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_state := Mux(recv_h_finished, s_mem_recv_h)
  }

  // control
  when (io.mem.req.fire()) {
    busy := Bool(true)
  }

  when ((r_recv_state === s_recv_finish) && io.resp.fire()) {
    r_recv_state := s_idle
    printf("MatrixMulTwoRequester: Finished. Answer = %x\n", r_total)
  }

  // PROC RESPONSE INTERFACE
  io.resp.valid := (io.cmd.fire() && setLength) ||   // setLength
                   (r_recv_state === s_recv_finish)  // doCalc
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := r_total
  // Semantics is to always send out prior accumulator register value

  io.busy      := Bool(false)
  // io.busy := io.cmd.valid
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)
}
