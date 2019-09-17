package tent_fpu

import chisel3._

import chisel3.iotesters
import chisel3.iotesters.{ChiselFlatSpec, Driver, PeekPokeTester}
import scala.io.Source
import java.io._
// import util.control.Breaks._

import freechips.rocketchip.system._
import freechips.rocketchip.tile._


class Inout extends Module {
  val io = IO(new Bundle {
    val in = Input(UInt(8.W))
    val out = Output(UInt(8.W))
  })

  io.out := io.in
}


class test_Inout (c: Inout) extends PeekPokeTester(c)
{
  private val tb = c

  poke(tb.io.in, 1)

  step(1)

  expect(tb.io.out, 1)
}


class InOutTester extends ChiselFlatSpec {
  println("IoTester is called\n")
  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new Inout()) {
      c => new test_Inout(c)
    } should be (true)
  }
}


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



class Tester extends ChiselFlatSpec {
  println("test_FPU is called\n")
  val config = new DefaultConfig
  "Basic test using Driver.execute" should "be used as an alternative way to run specification" in {
    iotesters.Driver.execute(Array(), () => new FPU(new FPUParams)(config)) {
      c => new test_FPU(c)
    } should be (true)
  }
}
