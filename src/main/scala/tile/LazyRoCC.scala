// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.tile

import Chisel._

import freechips.rocketchip.config._
import freechips.rocketchip.coreplex._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.InOrderArbiter

case object RoccNPTWPorts extends Field[Int]
case object BuildRoCC extends Field[Seq[RoCCParams]]

case class RoCCParams(
  opcodes: OpcodeSet,
  generator: Parameters => LazyRoCC,
  nPTWPorts : Int = 0,
  useFPU: Boolean = false)

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

class RoCCCoreIO(implicit p: Parameters) extends CoreBundle()(p) {
  val cmd = Decoupled(new RoCCCommand).flip
  val resp = Decoupled(new RoCCResponse)
  val mem = new HellaCacheIO
  val busy = Bool(OUTPUT)
  val interrupt = Bool(OUTPUT)
  val exception = Bool(INPUT)

  override def cloneType = new RoCCCoreIO()(p).asInstanceOf[this.type]
}

/** Base classes for Diplomatic TL2 RoCC units **/
abstract class LazyRoCC(implicit p: Parameters) extends LazyModule {
  val module: LazyRoCCModule

  val atlNode: TLMixedNode = TLOutputNode()
  val tlNode: TLMixedNode = TLOutputNode()
}

class RoCCIO(outer: LazyRoCC)(implicit p: Parameters) extends RoCCCoreIO()(p) {
  val atl = outer.atlNode.bundleOut
  val tl = outer.tlNode.bundleOut
  // Should be handled differently, eventually
  val ptw = Vec(p(RoccNPTWPorts), new TLBPTWIO)
  val fpu_req = Decoupled(new FPInput)
  val fpu_resp = Decoupled(new FPResult).flip
}

class LazyRoCCModule(outer: LazyRoCC) extends LazyModuleImp(outer) {
  val io = new RoCCIO(outer)
}

/** Mixins for including RoCC **/

trait HasLazyRoCC extends CanHaveSharedFPU with CanHavePTW with HasTileLinkMasterPort {
  implicit val p: Parameters
  val module: HasLazyRoCCModule

  val roccs = p(BuildRoCC).zipWithIndex.map { case (accelParams, i) =>
    accelParams.generator(p.alterPartial({
      case RoccNPTWPorts => accelParams.nPTWPorts
  }))}

  roccs.map(_.atlNode).foreach { atl => tileBus.node :=* atl }
  roccs.map(_.tlNode).foreach { tl => masterNode :=* tl }

  nPTWPorts += p(BuildRoCC).map(_.nPTWPorts).foldLeft(0)(_ + _)
  nDCachePorts += roccs.size
}

trait HasLazyRoCCModule extends CanHaveSharedFPUModule
  with CanHavePTWModule
  with HasCoreParameters
  with HasTileLinkMasterPortModule {
  val outer: HasLazyRoCC
  val roccCore = Wire(new RoCCCoreIO()(outer.p))

  val buildRocc = outer.p(BuildRoCC)
  val usingRocc = !buildRocc.isEmpty
  val nRocc = buildRocc.size
  val nFPUPorts = buildRocc.filter(_.useFPU).size
  val roccOpcodes = buildRocc.map(_.opcodes)

  if(usingRocc) {
    val respArb = Module(new RRArbiter(new RoCCResponse()(outer.p), nRocc))
    roccCore.resp <> respArb.io.out
    val cmdRouter = Module(new RoccCommandRouter(roccOpcodes)(outer.p))
    cmdRouter.io.in <> roccCore.cmd

    outer.roccs.zipWithIndex.foreach { case (rocc, i) =>
      ptwPorts ++= rocc.module.io.ptw
      rocc.module.io.cmd <> cmdRouter.io.out(i)
      rocc.module.io.exception := roccCore.exception
      val dcIF = Module(new SimpleHellaCacheIF()(outer.p))
      dcIF.io.requestor <> rocc.module.io.mem
      dcachePorts += dcIF.io.cache
      respArb.io.in(i) <> Queue(rocc.module.io.resp)
    }
    roccCore.busy := cmdRouter.io.busy || outer.roccs.map(_.module.io.busy).reduce(_ || _)
    roccCore.interrupt := outer.roccs.map(_.module.io.interrupt).reduce(_ || _)

    fpuOpt foreach { fpu =>
      if (usingFPU && nFPUPorts > 0) {
        val fpArb = Module(new InOrderArbiter(new FPInput()(outer.p), new FPResult()(outer.p), nFPUPorts))
        val fp_rocc_ios = outer.roccs.zip(buildRocc)
          .filter { case (_, params) => params.useFPU }
          .map { case (rocc, _) => rocc.module.io }
        fpArb.io.in_req <> fp_rocc_ios.map(_.fpu_req)
        fp_rocc_ios.zip(fpArb.io.in_resp).foreach {
          case (rocc, arb) => rocc.fpu_resp <> arb
        }
        fpu.io.cp_req <> fpArb.io.out_req
        fpArb.io.out_resp <> fpu.io.cp_resp
      } else {
        fpu.io.cp_req.valid := Bool(false)
        fpu.io.cp_resp.ready := Bool(false)
      }
    }
  }
}

class  AccumulatorExample(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new AccumulatorExampleModule(this)
}

class AccumulatorExampleModule(outer: AccumulatorExample, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
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
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)
}

class  TranslatorExample(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new TranslatorExampleModule(this)
}

class TranslatorExampleModule(outer: TranslatorExample)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
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

  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := req_rd
  io.resp.bits.data := Mux(pte.leaf(), Cat(pte.ppn, req_offset), SInt(-1, xLen).asUInt)

  io.busy := (state =/= s_idle)
  io.interrupt := Bool(false)
  io.mem.req.valid := Bool(false)
}

class  CharacterCountExample(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new CharacterCountExampleModule(this)
  override val atlNode = TLClientNode(TLClientParameters("CharacterCountRoCC"))
}

class CharacterCountExampleModule(outer: CharacterCountExample)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters
  with HasL1CacheParameters {
  val cacheParams = tileParams.icache.get

  private val blockOffset = blockOffBits
  private val beatOffset = log2Up(cacheDataBits/8)

  val needle = Reg(UInt(width = 8))
  val addr = Reg(UInt(width = coreMaxAddrBits))
  val count = Reg(UInt(width = xLen))
  val resp_rd = Reg(io.resp.bits.rd)

  val addr_block = addr(coreMaxAddrBits - 1, blockOffset)
  val offset = addr(blockOffset - 1, 0)
  val next_addr = (addr_block + UInt(1)) << UInt(blockOffset)

  val s_idle :: s_acq :: s_gnt :: s_check :: s_resp :: Nil = Enum(Bits(), 5)
  val state = Reg(init = s_idle)

  val tl_out = io.atl.head
  val gnt = tl_out.d.bits
  val recv_data = Reg(UInt(width = cacheDataBits))
  val recv_beat = Reg(UInt(width = log2Up(cacheDataBeats+1)), init = UInt(0))

  val data_bytes = Vec.tabulate(cacheDataBits/8) { i => recv_data(8 * (i + 1) - 1, 8 * i) }
  val zero_match = data_bytes.map(_ === UInt(0))
  val needle_match = data_bytes.map(_ === needle)
  val first_zero = PriorityEncoder(zero_match)

  val chars_found = PopCount(needle_match.zipWithIndex.map {
    case (matches, i) =>
      val idx = Cat(recv_beat - UInt(1), UInt(i, beatOffset))
      matches && idx >= offset && UInt(i) <= first_zero
  })
  val zero_found = zero_match.reduce(_ || _)
  val finished = Reg(Bool())

  io.cmd.ready := (state === s_idle)
  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := resp_rd
  io.resp.bits.data := count
  tl_out.a.valid := (state === s_acq)
  tl_out.a.bits := outer.atlNode.edgesOut(0).Get(
                       fromSource = UInt(0),
                       toAddress = addr_block << blockOffset,
                       lgSize = UInt(lgCacheBlockBytes))._2
  tl_out.d.ready := (state === s_gnt)

  when (io.cmd.fire()) {
    addr := io.cmd.bits.rs1
    needle := io.cmd.bits.rs2
    resp_rd := io.cmd.bits.inst.rd
    count := UInt(0)
    finished := Bool(false)
    state := s_acq
  }

  when (tl_out.a.fire()) { state := s_gnt }

  when (tl_out.d.fire()) {
    recv_beat := recv_beat + UInt(1)
    recv_data := gnt.data
    state := s_check
  }

  when (state === s_check) {
    when (!finished) {
      count := count + chars_found
    }
    when (zero_found) { finished := Bool(true) }
    when (recv_beat === UInt(cacheDataBeats)) {
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
  // Tie off unused channels
  tl_out.b.ready := Bool(true)
  tl_out.c.valid := Bool(false)
  tl_out.e.valid := Bool(false)
}

class OpcodeSet(val opcodes: Seq[UInt]) {
  def |(set: OpcodeSet) =
    new OpcodeSet(this.opcodes ++ set.opcodes)

  def matches(oc: UInt) = opcodes.map(_ === oc).reduce(_ || _)
}

object OpcodeSet {
  def custom0 = new OpcodeSet(Seq(Bits("b0001011")))
  def custom1 = new OpcodeSet(Seq(Bits("b0101011")))
  def custom2 = new OpcodeSet(Seq(Bits("b1011011")))
  def custom3 = new OpcodeSet(Seq(Bits("b1111011")))
  def all = custom0 | custom1 | custom2 | custom3
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


class MemTotalExample(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new MemTotalExampleModule(this)
}

class MemTotalExampleModule(outer: MemTotalExample, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
  val busy = Reg(init = {Bool(false)})

  val r_recv_max   = Reg(UInt(width = xLen));
  val r_cmd_count  = Reg(UInt(width = xLen));
  val r_recv_count = Reg(UInt(width = xLen));

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr = Reg(UInt(width = xLen))
  // datapath
  val r_total = Reg(UInt(width = xLen));
  val r_tag = Reg(UInt(width = n))

  val s_idle :: s_mem_acc :: s_finish :: Nil = Enum(Bits(), 3)
  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  when (io.cmd.valid) {
    printf("MemTotalExample: On Going. %x, %x\n", r_cmd_state, r_recv_state)
  }


  when (io.cmd.fire()) {
    printf("MemTotalExample: Command Received. %x, %x\n", io.cmd.bits.rs1, io.cmd.bits.rs2)

    r_total      := UInt(0)
    r_addr       := io.cmd.bits.rs1
    r_recv_max   := io.cmd.bits.rs2
    r_recv_count := UInt(0)
    r_cmd_count  := UInt(0)
    r_tag        := UInt(0)

    r_resp_rd := io.cmd.bits.inst.rd

    r_cmd_state  := s_mem_acc
    r_recv_state := s_mem_acc

  }

  io.cmd.ready := (r_cmd_state === s_idle)
  // command resolved if no stalls AND not issuing a load that will need a request

  val cmd_finished = r_cmd_count === r_recv_max
  when ((r_cmd_state === s_mem_acc) && io.mem.req.fire()) {
    printf("MemTotalExample: IO.MEM Command Received %x %x\n", io.mem.resp.bits.data, r_cmd_state)

    r_cmd_count  := r_cmd_count + UInt(1)
    r_tag        := r_tag + UInt(1)
    r_addr       := r_addr + UInt(8)
    r_cmd_state  := Mux(cmd_finished, s_idle, s_mem_acc)
  }

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := (r_cmd_state === s_mem_acc)
  io.mem.req.bits.addr := r_addr
  io.mem.req.bits.tag := r_tag
  io.mem.req.bits.cmd := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ := MT_D // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  val recv_finished = (r_recv_count === r_recv_max)
  when (r_recv_state === s_mem_acc && io.mem.resp.valid) {
    printf("MemTotalExample: IO.MEM Received %x %x\n", io.mem.resp.bits.data, r_recv_state)

    r_total      := r_total + io.mem.resp.bits.data
    r_recv_count := r_recv_count + UInt(1)
    r_recv_state := Mux(recv_finished, s_finish, s_mem_acc)
  }

  // control
  when (io.mem.req.fire()) {
    busy := Bool(true)
  }

  when ((r_recv_state === s_finish) && io.resp.fire()) {
    r_recv_state := s_idle
    printf("MemTotalExample: Finished. Answer = %x\n", r_total)
  }

  // PROC RESPONSE INTERFACE
  io.resp.valid := (r_recv_state === s_finish)
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := r_total
  // Semantics is to always send out prior accumulator register value

  io.busy := io.cmd.valid
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)
}


class MatrixMul(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new MatrixMulModule(this)
}

class MatrixMulModule(outer: MatrixMul, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
  val busy = Reg(init = {Bool(false)})

  val r_cmd_count  = Reg(UInt(width = xLen))
  val r_recv_count = Reg(UInt(width = xLen))

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr    = Reg(UInt(width = xLen))
  val r_v_addr  = Reg(UInt(width = xLen))
  val r_h_addr  = Reg(UInt(width = xLen))
  // datapath
  val r_total = Reg(UInt(width = xLen))
  val r_tag   = Reg(UInt(width = n))

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


class MatrixMulTwoRequest(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new MatrixMulTwoRequestModule(this)
}

class MatrixMulTwoRequestModule(outer: MatrixMulTwoRequest, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
  val busy = Reg(init = {Bool(false)})

  val funct  = io.cmd.bits.inst.funct
  val setM   = (funct === UInt(0))
  val setK   = (funct === UInt(1))
  val doCalc = (funct === UInt(2))

  val r_cmd_count  = Reg(UInt(width = xLen))
  val r_recv_count = Reg(UInt(width = xLen))

  val r_matrix_max = Reg(UInt(width = xLen))
  val r_matrix_K   = Reg(UInt(width = xLen))

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr   = Reg(UInt(width = xLen))
  val r_v_addr = Reg(UInt(width = xLen))
  val r_h_addr = Reg(UInt(width = xLen))

  // datapath
  val r_total = Reg(UInt(width = xLen))
  val r_h_val = Reg(UInt(width = xLen))
  val r_tag   = Reg(UInt(width = n))

  val s_idle :: s_mem_fetch :: s_recv_finish :: s_mem_recv :: Nil = Enum(Bits(), 4)
  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  when (io.cmd.valid) {
    printf("MatrixMulTwoRequester: Funct Request. %x\n", funct)
  }

  when (io.cmd.fire() && setM) {
    printf("MatrixMulTwoRequester: SetLengthM Request. %x\n", io.cmd.bits.rs1)
    r_matrix_max := io.cmd.bits.rs1
    r_recv_state := s_recv_finish
  }

  when (io.cmd.fire() && setK) {
    printf("MatrixMulTwoRequester: SetLengthK Request. %x\n", io.cmd.bits.rs1)
    r_matrix_K   := io.cmd.bits.rs1
    r_recv_state := s_recv_finish
  }

  when (io.cmd.fire()) {
    r_total      := UInt(0)
    r_resp_rd := io.cmd.bits.inst.rd
  }

  when (io.cmd.fire() && doCalc) {
    printf("MatrixMulTwoRequester: DoCalc Received. %x, %x\n", io.cmd.bits.rs1, io.cmd.bits.rs2)

    r_v_addr     := io.cmd.bits.rs2
    r_h_addr     := io.cmd.bits.rs1

    r_recv_count := UInt(0)
    r_cmd_count  := UInt(0)
    r_tag        := UInt(0)

    r_cmd_state  := s_mem_fetch
    r_recv_state := s_mem_recv
  }

  val w_addr = Mux (r_cmd_count(0) === UInt(0), r_h_addr, r_v_addr)

  io.cmd.ready := (r_cmd_state === s_idle) && (r_recv_state === s_idle)
  // command resolved if no stalls AND not issuing a load that will need a request

  val cmd_request_max = (r_matrix_max << UInt(1)) - UInt(1)

  val cmd_finished = (r_cmd_count === cmd_request_max)
  when ((r_cmd_state === s_mem_fetch) && io.mem.req.fire()) {
    printf("MatrixMulTwoRequester: <<s_mem_fetch_v>> IO.MEM Command Fire %x\n", w_addr)

    r_cmd_count  := Mux(cmd_finished, UInt(0), r_cmd_count + UInt(1))

    r_h_addr     := Mux(r_cmd_count(0), r_h_addr, r_h_addr + UInt(8))
    r_v_addr     := Mux(r_cmd_count(0), r_v_addr + (r_matrix_K << UInt(3)), r_v_addr)
    r_cmd_state  := Mux(cmd_finished, s_idle, s_mem_fetch)
  }

  when (io.mem.req.fire()) {
    r_tag        := r_tag + UInt(1)
  }

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := (r_cmd_state === s_mem_fetch)
  io.mem.req.bits.addr := w_addr
  io.mem.req.bits.tag  := r_tag
  io.mem.req.bits.cmd  := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ  := MT_D  // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  val recv_finished = (r_recv_count === cmd_request_max)
  when (r_recv_state === s_mem_recv && io.mem.resp.fire()) {
    printf("MatrixMulTwoRequester: <<s_mem_recv_v>> IO.MEM Received %x (r_count=%d)\n", io.mem.resp.bits.data, r_recv_count)

    r_recv_count := Mux(recv_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_state := Mux(recv_finished, s_recv_finish, s_mem_recv)

    r_h_val      := Mux(r_recv_count(0), r_h_val, io.mem.resp.bits.data)

    r_total      := Mux(r_recv_count(0), r_total + r_h_val * io.mem.resp.bits.data, r_total)
    when (r_recv_count(0)) {
      printf("MatrixMulTwoRequester: <<s_mem_recv_v>> r_total update %x\n", r_total)
    }
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
  io.resp.valid := (r_recv_state === s_recv_finish)
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := r_total
  // Semantics is to always send out prior accumulator register value

  io.busy := Bool(false)
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)
}


class MatrixMul32(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new MatrixMul32Module(this)
}

class MatrixMul32Module(outer: MatrixMul32, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
  val busy = Reg(init = {Bool(false)})

  val funct  = io.cmd.bits.inst.funct
  val setM   = (funct === UInt(0))
  val setK   = (funct === UInt(1))
  val doCalc = (funct === UInt(2))

  val r_cmd_count  = Reg(UInt(width = xLen))
  val r_recv_count = Reg(UInt(width = xLen))

  val r_cmd_count_3  = Reg(UInt(width = 2))
  val r_recv_count_3 = Reg(UInt(width = 2))

  val r_matrix_max = Reg(UInt(width = xLen))
  val r_matrix_K   = Reg(UInt(width = xLen))

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr   = Reg(UInt(width = xLen))
  val r_v_addr = Reg(UInt(width = xLen))
  val r_h_addr = Reg(UInt(width = xLen))

  // datapath
  val r_total_0 = Reg(SInt(width = 32))
  val r_total_1 = Reg(SInt(width = 32))
  val r_h_val_0 = Reg(SInt(width = 32))
  val r_h_val_1 = Reg(SInt(width = 32))

  val r_tag   = Reg(UInt(width = n))

  val s_idle :: s_mem_fetch :: s_recv_finish :: s_mem_recv :: Nil = Enum(Bits(), 4)
  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  when (io.cmd.valid) {
    printf("MatrixMul32: Funct Request. %x\n", funct)
  }

  when (io.cmd.fire() && setM) {
    printf("MatrixMul32: SetLengthM Request. %x\n", io.cmd.bits.rs1)
    r_matrix_max := io.cmd.bits.rs1
    r_recv_state := s_recv_finish
  }

  when (io.cmd.fire() && setK) {
    printf("MatrixMul32: SetLengthK Request. %x\n", io.cmd.bits.rs1)
    r_matrix_K   := io.cmd.bits.rs1
    r_recv_state := s_recv_finish
  }

  when (io.cmd.fire()) {
    r_total_0 := SInt(0)
    r_total_1 := SInt(0)
    r_resp_rd := io.cmd.bits.inst.rd
  }

  when (io.cmd.fire() && doCalc) {
    printf("MatrixMul32: DoCalc Received. %x, %x\n", io.cmd.bits.rs1, io.cmd.bits.rs2)

    r_h_addr := Cat(io.cmd.bits.rs1(63, 3), UInt(0, width=3))
    r_v_addr := Cat(io.cmd.bits.rs2(63, 3), UInt(0, width=3))

    r_recv_count   := UInt(0)
    r_recv_count_3 := UInt(0)
    r_cmd_count    := UInt(0)
    r_cmd_count_3  := UInt(0)
    r_tag          := UInt(0)

    r_cmd_state  := s_mem_fetch
    r_recv_state := s_mem_recv
  }

  val w_addr = Mux (r_cmd_count_3 === UInt(0), r_h_addr, r_v_addr)

  io.cmd.ready := (r_cmd_state === s_idle) && (r_recv_state === s_idle)
  // command resolved if no stalls AND not issuing a load that will need a request

  val cmd_request_max = (r_matrix_max(xLen-1,1)) + r_matrix_max  // x3

  val cmd_finished = (r_cmd_count === cmd_request_max)
  when ((r_cmd_state === s_mem_fetch) && io.mem.req.fire()) {
    printf("MatrixMul32: <<s_mem_fetch_v>> IO.MEM Command Fire %x\n", w_addr)

    r_cmd_count  := Mux(cmd_finished,              UInt(0), r_cmd_count + UInt(1))
    r_cmd_count_3:= Mux(r_cmd_count_3 === UInt(2), UInt(0), r_cmd_count_3 + UInt(1))

    r_h_addr     := Mux(r_cmd_count_3 === UInt(0), r_h_addr + UInt(8), r_h_addr)
    r_v_addr     := Mux(r_cmd_count_3 === UInt(1) || r_cmd_count_3 === UInt(2),
                        r_v_addr + (r_matrix_K << UInt(2)),
                        r_v_addr)
    r_cmd_state  := Mux(cmd_finished, s_idle, s_mem_fetch)
  }

  when (io.mem.req.fire()) {
    r_tag        := r_tag + UInt(1)
  }

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := (r_cmd_state === s_mem_fetch)
  io.mem.req.bits.addr := w_addr
  io.mem.req.bits.tag  := r_tag
  io.mem.req.bits.cmd  := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ  := MT_D  // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  val recv_finished = (r_recv_count === cmd_request_max)

  def SignExtend32(x: UInt) = {
    (Cat(Fill(32,x(31)), x)).asSInt
  }

  val w_resp_data_0 = Wire (SInt(width = 32))
  val w_resp_data_1 = Wire (SInt(width = 32))
  w_resp_data_0 := SignExtend32 (io.mem.resp.bits.data(31, 0))
  w_resp_data_1 := SignExtend32 (io.mem.resp.bits.data(63,32))

  val w_total_0 = Wire(SInt(width = 32))
  val w_total_1 = Wire(SInt(width = 32))
  w_total_0 := r_total_0 + Mux(r_recv_count_3 === UInt(1), r_h_val_0.asSInt * w_resp_data_0.asSInt,
                           Mux(r_recv_count_3 === UInt(2), r_h_val_1.asSInt * w_resp_data_0.asSInt,
                           r_total_0))
  w_total_1 := r_total_1 + Mux(r_recv_count_3 === UInt(1), r_h_val_0.asSInt * w_resp_data_1.asSInt,
                           Mux(r_recv_count_3 === UInt(2), r_h_val_1.asSInt * w_resp_data_1.asSInt,
                           r_total_1))

  when (r_recv_state === s_mem_recv && io.mem.resp.fire() && (r_recv_count_3 === UInt(1))) {
    printf("MatrixMul32: <<s_mem_recv_v>> r_total update %x + %x * %x = %x\n",
      r_total_0, r_h_val_0, w_resp_data_0, w_total_0);
  }
  when (r_recv_state === s_mem_recv && io.mem.resp.fire() && (r_recv_count_3 === UInt(2))) {
    printf("MatrixMul32: <<s_mem_recv_v>> r_total update %x + %x * %x = %x\n",
      r_total_0, r_h_val_1, w_resp_data_0, w_total_0);
  }

  when (r_recv_state === s_mem_recv && io.mem.resp.fire()) {
    printf("MatrixMul32: <<s_mem_recv_v>> IO.MEM Received %x,%x (r_recv_count_3=%d)\n",
      SignExtend32 (io.mem.resp.bits.data(31, 0)), SignExtend32 (io.mem.resp.bits.data(63,32)),
      r_recv_count_3)

    r_recv_count   := Mux(recv_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_count_3 := Mux(r_recv_count_3 === UInt(2), UInt(0), r_recv_count_3 + UInt(1))

    r_recv_state := Mux(recv_finished, s_recv_finish, s_mem_recv)

    r_h_val_0    := Mux(r_recv_count_3 === UInt(0), SignExtend32 (io.mem.resp.bits.data(31, 0)), r_h_val_0)
    r_h_val_1    := Mux(r_recv_count_3 === UInt(0), SignExtend32 (io.mem.resp.bits.data(63,32)), r_h_val_1)

    r_total_0 := Mux(r_recv_count_3 === UInt(1) || r_recv_count_3 === UInt(2), w_total_0, r_total_0)
    r_total_1 := Mux(r_recv_count_3 === UInt(1) || r_recv_count_3 === UInt(2), w_total_1, r_total_1)
  }

  // control
  when (io.mem.req.fire()) {
    busy := Bool(true)
  }

  when ((r_recv_state === s_recv_finish) && io.resp.fire()) {
    r_recv_state := s_idle
    printf("MatrixMul32: Finished. Answer = %d, %d\n", r_total_1, r_total_0)
  }

  // PROC RESPONSE INTERFACE
  io.resp.valid := (r_recv_state === s_recv_finish)
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := Cat (r_total_1, r_total_0)
  // Semantics is to always send out prior accumulator register value

  io.busy := Bool(false)
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)
}
