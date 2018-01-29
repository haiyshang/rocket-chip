package freechips.rocketchip.tile

import Chisel._

import chisel3.core.{Input, Output}

import freechips.rocketchip.config._
// import freechips.rocketchip.coreplex._
// import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
// import freechips.rocketchip.tilelink._
// import freechips.rocketchip.util.InOrderArbiter

class Fix16Mul extends Module {
  val io = IO(new Bundle {
    val A_In   = Input(UInt(width = 32))
    val B_In   = Input(UInt(width = 32))
    val En_In  = Input(Bool())
    val C_Out  = Output(SInt(width = 32))
    val En_Out = Output(Bool())
  })

  val w_a_hi = Wire(SInt(width=32))
  val w_b_hi = Wire(SInt(width=32))
  val w_a_lo = Wire(UInt(width=32))
  val w_b_lo = Wire(UInt(width=32))

  // int32_t  A = (a_val >> 16),    C = (b_val >> 16);
  // uint32_t B = (a_val & 0xFFFF), D = (b_val & 0xFFFF);
  w_a_hi := io.A_In(31,16).asSInt()
  w_b_hi := io.B_In(31,16).asSInt()
  w_a_lo := Cat(UInt(0, 16), io.A_In(15, 0))
  w_b_lo := Cat(UInt(0, 16), io.B_In(15, 0))

  // int32_t  AC    = A*C;
  // int32_t  AD_CB = A*D + C*B;
  // uint32_t BD    = B*D;
  val w_ah_bh       = Reg(SInt(width=32))
  val w_ah_bl_al_bh = Reg(SInt(width=32))
  val w_al_bl       = Reg(UInt(width=32))

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

  io.C_Out := Cat(product_hi3(15, 0), product_lo2(31,16)).asSInt() + SInt(1)

  val r_en_out = Reg(Bool())
  r_en_out := io.En_In
  io.En_Out := r_en_out
}


class LoggerRAM (DataWidth: Int, Length: Int) extends Module {
  val io = IO(new Bundle {
    val WData_In = Input(UInt(width = DataWidth))
    val WAddr_In = Input(UInt(width = log2Ceil(Length)))
    val We_In    = Input(Bool())

    val RAddr_In = Input(UInt(width = log2Ceil(Length)))
    val Data_Out = Output(UInt(width = DataWidth))
  })

  val memory = Mem(1024, UInt(width = DataWidth))

  when (io.We_In) {
    memory(io.WAddr_In) := io.WData_In
  }

  val w_data_out = Reg(UInt(width = DataWidth))
  w_data_out := memory(io.RAddr_In)

  io.Data_Out := w_data_out
}


class DotProductF16(implicit p: Parameters) extends LazyRoCC {
  override lazy val module = new DotProductF16Module(this)
}

class DotProductF16Module(outer: DotProductF16, n: Int = 4)(implicit p: Parameters) extends LazyRoCCModule(outer)
  with HasCoreParameters {
  val busy = Reg(init = {Bool(false)})

  val funct   = io.cmd.bits.inst.funct
  val setM         = (funct === UInt(0))
  val setK         = (funct === UInt(1))
  val doCalc       = (funct === UInt(2))
  val getResultMem = (funct === UInt(3))
  val getInputMem  = (funct === UInt(4))
  val getWeightMem = (funct === UInt(5))

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
  val r_tag   = Reg(UInt(width = n))

  val w_result = Wire(SInt(width=32))

  val s_idle :: s_mem_fetch :: s_recv_finish :: s_mem_recv :: s_recv_resultLog_finish :: s_recv_inputLog_finish :: s_recv_weightLog_finish :: Nil = Enum(Bits(), 7)

  val r_cmd_state  = Reg(UInt(width = 3), init = s_idle)
  val r_recv_state = Reg(UInt(width = 3), init = s_idle)

  val w_calc_done  = Wire(init = {Bool(false)})

  var logmem_word_len = 1024

  val w_result_MemOutput  = Wire(UInt(width=32))
  val w_input_MemOutput   = Wire(UInt(width=32))
  val w_weight_MemOutput  = Wire(UInt(width=32))

  val r_result_log_count = Reg(UInt(width=32))
  val r_input_log_count  = Reg(UInt(width=32))
  val r_weight_log_count = Reg(UInt(width=32))

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

  when (io.cmd.fire() && getResultMem) {
    printf("DotProductF16: ResultMem[%d]\n", io.cmd.bits.rs1)
    r_recv_state := s_recv_resultLog_finish
  }

  when (io.cmd.fire() && getInputMem) {
    printf("DotProductF16: InputMem[%d].\n", io.cmd.bits.rs1)
    r_recv_state := s_recv_inputLog_finish
  }

  when (io.cmd.fire() && getWeightMem) {
    printf("DotProductF16: WeightMem[%d].\n", io.cmd.bits.rs1)
    r_recv_state := s_recv_weightLog_finish
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

    r_result_log_count := UInt(0)
    r_input_log_count  := UInt(0)
    r_weight_log_count := UInt(0)

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
  }

  when (w_calc_done) {
    r_total := r_total + w_result
    printf("DotProductF16: <<s_mem_recv_v>> w_result update %x\n", w_result)
  }

  // control
  when (io.mem.req.fire()) {
    busy := Bool(true)
  }

  when (io.resp.fire()) {
    when (r_recv_state === s_recv_finish) {
      r_recv_state := s_idle
      printf ("DotProductF16: Finished. Answer = %x\n", r_total)
    } .elsewhen (r_recv_state === s_recv_resultLog_finish) {
      r_recv_state := s_idle
      printf ("DotProductF16: MemResult Finished = %x\n", w_result_MemOutput)
    } .elsewhen (r_recv_state === s_recv_inputLog_finish) {
      r_recv_state := s_idle
      printf ("DotProductF16: InputResult Finished = %x\n", w_input_MemOutput)
    } .elsewhen (r_recv_state === s_recv_weightLog_finish) {
      r_recv_state := s_idle
      printf ("DotProductF16: WeightResult Finished = %x\n", w_weight_MemOutput)
    }
  }

  // PROC RESPONSE INTERFACE
  io.resp.valid := (r_recv_state === s_recv_finish) || (r_recv_state === s_recv_resultLog_finish) || (r_recv_state === s_recv_inputLog_finish) || (r_recv_state === s_recv_weightLog_finish)
  // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := r_resp_rd
  // Must respond with the appropriate tag or undefined behavior
  when (r_recv_state === s_recv_finish) {
    io.resp.bits.data := r_total.asUInt()
  } .elsewhen (r_recv_state === s_recv_resultLog_finish) {
    io.resp.bits.data := w_result_MemOutput
  } .elsewhen (r_recv_state === s_recv_inputLog_finish) {
    io.resp.bits.data := w_input_MemOutput
  } .elsewhen (r_recv_state === s_recv_weightLog_finish) {
    io.resp.bits.data := w_weight_MemOutput
  } .otherwise {
    io.resp.bits.data := UInt(0)
  }

  //=================================================
  // Reorder Receiver
  //=================================================
  val data_tag_counter = Reg(UInt(width=n))

  val a_data_queue = Module (new ReorderQueue(UInt(width=32), n-1))
  val b_data_queue = Module (new ReorderQueue(UInt(width=32), n-1))

  a_data_queue.io.enq.valid     := io.mem.resp.fire() && !io.mem.resp.bits.tag(0)
  a_data_queue.io.enq.bits.data := io.mem.resp.bits.data
  a_data_queue.io.enq.bits.tag  := io.mem.resp.bits.tag

  a_data_queue.io.deq.tag  := Cat(data_tag_counter, UInt(0, width=1))

  b_data_queue.io.enq.valid     := io.mem.resp.fire() &&  io.mem.resp.bits.tag(0)
  b_data_queue.io.enq.bits.data := io.mem.resp.bits.data
  b_data_queue.io.enq.bits.tag  := io.mem.resp.bits.tag

  b_data_queue.io.deq.tag  := Cat(data_tag_counter, UInt(1, width=1))

  when (a_data_queue.io.deq.matches && b_data_queue.io.deq.matches) {
    a_data_queue.io.deq.valid := Bool(true)
    b_data_queue.io.deq.valid := Bool(true)

    data_tag_counter := data_tag_counter + UInt(1)
  } .otherwise {
    a_data_queue.io.deq.valid := Bool(false)
    b_data_queue.io.deq.valid := Bool(false)
  }

  when (io.cmd.fire() && doCalc) {
    data_tag_counter := UInt(0)
  }

  // Semantics is to always send out prior accumulator register value
  io.busy := Bool(false)
  // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
  // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)

  val r_log_overflow = Reg(init = {Bool(false)})

  //=================================================
  // Fix16_Mul
  //=================================================
  val fix16_mul = Module (new Fix16Mul())
  fix16_mul.io.A_In  := a_data_queue.io.deq.data
  fix16_mul.io.B_In  := b_data_queue.io.deq.data
  fix16_mul.io.En_In := a_data_queue.io.deq.matches && b_data_queue.io.deq.matches
  w_result           := fix16_mul.io.C_Out
  w_calc_done        := fix16_mul.io.En_Out

  //=================================================
  // Logger
  //=================================================

  val ResultRAM = Module (new LoggerRAM(32, 1024))
  val InputRAM  = Module (new LoggerRAM(32, 1024))
  val WeightRAM = Module (new LoggerRAM(32, 1024))

  ResultRAM.io.WData_In := (r_total + w_result).asUInt()
  ResultRAM.io.We_In    := w_calc_done && !r_log_overflow
  ResultRAM.io.WAddr_In := r_result_log_count
  ResultRAM.io.RAddr_In := io.cmd.bits.rs1
  w_result_MemOutput    := ResultRAM.io.Data_Out

  InputRAM.io.WData_In  := a_data_queue.io.deq.data
  InputRAM.io.We_In     := a_data_queue.io.deq.valid && !r_log_overflow
  InputRAM.io.WAddr_In  := r_input_log_count
  InputRAM.io.RAddr_In  := io.cmd.bits.rs1
  w_input_MemOutput     := InputRAM.io.Data_Out

  WeightRAM.io.WData_In := b_data_queue.io.deq.data
  WeightRAM.io.We_In    := b_data_queue.io.deq.valid && !r_log_overflow
  WeightRAM.io.WAddr_In := r_weight_log_count
  WeightRAM.io.RAddr_In := io.cmd.bits.rs1
  w_weight_MemOutput    := WeightRAM.io.Data_Out

  when (w_calc_done && r_log_overflow === Bool(false)) {
    when (r_result_log_count === UInt(logmem_word_len-1)) {
      r_log_overflow := Bool(true)
    }
  }

  when (ResultRAM.io.We_In) {
    r_result_log_count := r_result_log_count + UInt(1)
  }
  when (InputRAM.io.We_In) {
    r_input_log_count := r_input_log_count + UInt(1)
  }
  when (WeightRAM.io.We_In) {
    r_weight_log_count := r_weight_log_count + UInt(1)
  }

}
