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
      regionType         = RegionType.UNCACHED,
      executable         = true,
      supportsGet        = TransferSizes(1, beatBytes),
      fifoId             = Some(0))), // requests are handled in order
    beatBytes = beatBytes)))

  lazy val module = new LazyModuleImp(this) {
    val memory = Mem(1024, UInt(width = c.width))

    val (in, edge) = node.in(0)
    val mem_address = edge.addr_hi(in.a.bits.address - UInt(c.address))(log2Ceil(c.depth)-1, 0)

    val read = (in.a.bits.opcode === TLMessages.Get)
    when (in.a.fire() && !read) {
      memory(mem_address) := in.a.bits.data
      // memory(mem_address) := 0.U
    }

    val d_full = RegInit(Bool(false))
    val d_size = Reg(UInt())
    val d_source = Reg(UInt())
    val d_data = memory(mem_address) holdUnless RegNext(in.a.fire())

    // Flow control
    when (in.d.fire())         { d_full := Bool(false) }
    when (in.a.fire() && read) { d_full := Bool(true)  }
    in.d.valid := d_full
    in.a.ready := in.d.ready || !d_full

    when (in.a.fire()) {
      d_size   := in.a.bits.size
      d_source := in.a.bits.source
    }

    in.d.bits := edge.AccessAck(d_source, d_size, d_data)

    // Tie off unused channels
    in.b.valid := Bool(false)
    in.c.ready := Bool(true)
    in.e.ready := Bool(true)
  }
}

