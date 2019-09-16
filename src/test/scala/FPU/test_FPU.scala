package freechips.rocketchip.tile

import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import scala.io.Source
import java.io._
import util.control.Breaks._

import freechips.rocketchip.system._

class test_FPU (c: FPU) extends PeekPokeTester(c)
{
  private val FPU_tb = c

  poke(FPU_tb.io.valid, 1)
  poke(FPU_tb.io.valid, 0)

  step(1)
  step(1)
  step(1)
  step(1)

  poke(FPU_tb.io.valid, 0)

}



class main extends ChiselFlatSpec {
  val config = new DefaultConfig
  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new FPU(new FPUParams)(config)) {
      c => new test_FPU(c)
    } should be (true)
  }
}
