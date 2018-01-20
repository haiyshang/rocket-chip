package freechips.rocketchip.tile

import Chisel._

import freechips.rocketchip.config._
import freechips.rocketchip.coreplex._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.InOrderArbiter

class DotProductF16(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new DotProductF16Module(this)
}

class DotProductF16Module(outer: DotProductF16, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
  val busy = Reg(init = {Bool(false)})

  val funct  = io.cmd.bits.inst.funct
  val setM   = (funct === UInt(0))
  val setK   = (funct === UInt(1))
  val doCalc = (funct === UInt(2))

  val r_cmd_count  = Reg(UInt(width = xLen))
  val r_recv_count = Reg(UInt(width = xLen))

  val r_matrix_max = Reg(UInt(width = xLen))
  val r_v_step     = Reg(UInt(width = xLen))

  val r_resp_rd = Reg(io.resp.bits.rd)
  val r_addr   = Reg(UInt(width = xLen))
  val r_v_addr = Reg(UInt(width = xLen))
  val r_h_addr = Reg(UInt(width = xLen))

  // datapath
  val r_total = Reg(SInt(width = xLen))
  val r_a_val = Reg(UInt(width = xLen))
  val r_tag   = Reg(UInt(width = n))

  val w_result = Wire(SInt(width=32))

  val s_idle :: s_mem_fetch :: s_recv_finish :: s_mem_recv :: Nil = Enum(Bits(), 4)
  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  val w_ah_bh       = Wire(SInt(width=32))
  val w_ah_bl_al_bh = Wire(SInt(width=32))
  val w_al_bl       = Wire(UInt(width=32))

  when (io.cmd.fire() && setM) {
    printf("DotProductF16: SetLengthM Request. %x\n", io.cmd.bits.rs1)
    r_matrix_max := io.cmd.bits.rs1
    r_recv_state := s_recv_finish
  }

  when (io.cmd.fire() && setK) {
    printf("DotProductF16: SetLengthK Request. %x\n", io.cmd.bits.rs1)
    r_v_step     := io.cmd.bits.rs1
    r_recv_state := s_recv_finish
  }

  when (io.cmd.fire()) {
    r_total   := SInt(0)
    r_resp_rd := io.cmd.bits.inst.rd
  }

  when (io.cmd.fire() && doCalc) {
    printf("DotProductF16: DoCalc Received. %x, %x\n", io.cmd.bits.rs1, io.cmd.bits.rs2)

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
    printf("DotProductF16: <<s_mem_fetch_v>> IO.MEM Command Fire %x\n", w_addr)

    r_cmd_count  := Mux(cmd_finished, UInt(0), r_cmd_count + UInt(1))

    r_h_addr     := Mux(r_cmd_count(0), r_h_addr, r_h_addr + UInt(4))
    r_v_addr     := Mux(r_cmd_count(0), r_v_addr + (r_v_step << UInt(2)), r_v_addr)
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
  io.mem.req.bits.typ  := MT_W  // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
  io.mem.invalidate_lr := Bool(false)

  val recv_finished = (r_recv_count === cmd_request_max)
  when (r_recv_state === s_mem_recv && io.mem.resp.fire()) {
    printf("DotProductF16: <<s_mem_recv_v>> IO.MEM Received %x (r_count=%d)\n", io.mem.resp.bits.data, r_recv_count)
    r_recv_count := Mux(recv_finished, UInt(0), r_recv_count + UInt(1))
    r_recv_state := Mux(recv_finished, s_recv_finish, s_mem_recv)

    r_a_val      := Mux(r_recv_count(0), r_a_val, io.mem.resp.bits.data)

    r_total      := Mux(r_recv_count(0), r_total + w_result, r_total)

    when (r_recv_count(0)) {
      printf("DotProductF16: <<s_mem_recv_v>> w_result update %x\n", w_result)
      printf("DotProductF16: <<calc >> %x,%x,%x\n", w_ah_bh, w_ah_bl_al_bh, w_al_bl)
    }
  }

  val w_a_val = Wire(UInt(width = 32))
  val w_b_val = Wire(UInt(width = 32))
  w_a_val := r_a_val
  w_b_val := io.mem.resp.bits.data;

  val w_a_hi = Wire(SInt(width=32))
  val w_b_hi = Wire(SInt(width=32))
  val w_a_lo = Wire(UInt(width=32))
  val w_b_lo = Wire(UInt(width=32))

  // int32_t  A = (a_val >> 16),    C = (b_val >> 16);
  // uint32_t B = (a_val & 0xFFFF), D = (b_val & 0xFFFF);
  w_a_hi := w_a_val(31,16).asSInt()
  w_b_hi := w_b_val(31,16).asSInt()
  w_a_lo := Cat(UInt(0, 16), w_a_val(15, 0))
  w_b_lo := Cat(UInt(0, 16), w_b_val(15, 0))

  // int32_t  AC    = A*C;
  // int32_t  AD_CB = A*D + C*B;
  // uint32_t BD    = B*D;
  w_ah_bh       := w_a_hi * w_b_hi
  w_ah_bl_al_bh := w_a_hi * w_b_lo + w_a_lo * w_b_hi
  w_al_bl       := w_a_lo * w_b_lo

  val product_hi = Wire(SInt(width=32))
  product_hi := w_ah_bh + w_ah_bl_al_bh(31,16).asSInt()

  val product_lo = Wire(UInt(width=32))
  product_lo := w_al_bl + Cat(w_ah_bl_al_bh, UInt(0,width=16))

  val product_hi2 = Wire(SInt(width=32))
  when (product_lo < w_al_bl) {
    product_hi2 := product_hi + SInt(1)
  } .otherwise {
    product_hi2 := product_hi
  }

  val product_hi3 = Wire(SInt(width=32))
  val product_lo2 = Wire(UInt(width=32))
  product_lo2 := product_lo - UInt(0x8000) - product_hi(31)
  when (product_lo2 > product_lo) {
    product_hi3 := product_hi2 - SInt(1)
  } .otherwise {
    product_hi3 := product_hi2
  }

  w_result := Cat(product_hi3(15, 0), product_lo2(31,16)).asSInt() + SInt(1)

  // control
  when (io.mem.req.fire()) {
    busy := Bool(true)
  }

  when ((r_recv_state === s_recv_finish) && io.resp.fire()) {
    r_recv_state := s_idle
    printf("DotProductF16: Finished. Answer = %x\n", r_total)
  }

  // PROC RESPONSE INTERFACE
  io.resp.valid := (r_recv_state === s_recv_finish)
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := r_total.asUInt()
  // Semantics is to always send out prior accumulator register value

  io.busy := Bool(false)
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)
}
