// See LICENSE.SiFive for license details.


package freechips.rocketchip.devices.tilelink

import Chisel._
import freechips.rocketchip.coreplex.{HasPeripheryBus}
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._

case class SimpleRAMParams(address: BigInt, name: String, depth: Int = 2048, width: Int = 32)

case object PeripherySimpleRAMKey extends Field[Seq[SimpleRAMParams]]

trait HasPeripherySimpleRAMSlave extends HasPeripheryBus {
  val simpleRAMParams = p(PeripherySimpleRAMKey)
  val simpleRAMs = simpleRAMParams map { params =>
    val simpleRAM = LazyModule(new TLSimpleRAM(params))
    simpleRAM.node := pbus.toFixedWidthSingleBeatSlave(simpleRAM.beatBytes)
    simpleRAM
  }
}

class TLSimpleRAM(c: SimpleRAMParams)(implicit p: Parameters) extends LazyModule {
  val beatBytes = c.width/8
  val node = TLManagerNode(Seq(TLManagerPortParameters(
    Seq(TLManagerParameters(
      address            = AddressSet.misaligned(c.address, c.depth*beatBytes),
      resources          = new SimpleDevice("ram", Seq("msyksphinz,simpleRAM")).reg("mem"),
      regionType         = RegionType.UNCACHEABLE,
      // executable         = true,
      supportsGet        = TransferSizes(1, beatBytes),
      supportsPutPartial = TransferSizes(1, beatBytes),
      supportsPutFull    = TransferSizes(1, beatBytes),
      fifoId             = Some(0))), // requests are handled in order
    beatBytes = beatBytes)))

  lazy val module = new LazyModuleImp(this) {
    val memory = Mem(1024, UInt(width = c.width))
    val a_legal = Wire(Bool(true))

    val (in, edge) = node.in(0)
    val mem_address = edge.addr_hi(in.a.bits.address - UInt(c.address))(log2Ceil(c.depth)-1, 0)

    val d_full = RegInit(Bool(false))
    val d_read = Reg(Bool())
    val d_size = Reg(UInt())
    val d_source = Reg(UInt())
    val d_data = memory(mem_address) holdUnless RegNext(in.a.fire())
    val d_legal = Reg(Bool())

    // Flow control
    when (in.d.fire()) { d_full := Bool(false) }
    when (in.a.fire()) { d_full := Bool(true)  }
    in.d.valid := d_full
    in.a.ready := in.d.ready || !d_full

    in.d.bits := edge.AccessAck(d_source, d_size, !d_legal)
    // avoid data-bus Mux
    in.d.bits.data := d_data
    in.d.bits.opcode := Mux(d_read, TLMessages.AccessAckData, TLMessages.AccessAck)

    val read = in.a.bits.opcode === TLMessages.Get
    val rdata = Wire(Vec(beatBytes, Bits(width = 8)))
    val wdata = Vec.tabulate(beatBytes) { i => in.a.bits.data(8*(i+1)-1, 8*i) }

    when (in.a.fire()) {
      d_read   := read
      d_size   := in.a.bits.size
      d_source := in.a.bits.source
      d_legal  := a_legal
    }


    // Tie off unused channels
    in.b.valid := Bool(false)
    in.c.ready := Bool(true)
    in.e.ready := Bool(true)


    when (in.a.fire() && !read && a_legal) {
      memory(mem_address) := in.a.bits.data
      // memory(mem_address) := 0.U
    }
  }
}
