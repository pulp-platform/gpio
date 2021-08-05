//-----------------------------------------------------------------------------
// Title         : GPIO Testbench
//-----------------------------------------------------------------------------
// File          : tb_gpio.sv
// Author        : Manuel Eggimann  <meggimann@iis.ee.ethz.ch>
// Created       : 07.05.2021
//-----------------------------------------------------------------------------
// Description :
// Test the functionality of the GPIO Peripheral
//-----------------------------------------------------------------------------
// Copyright (C) 2013-2021 ETH Zurich, University of Bologna
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//-----------------------------------------------------------------------------


`define SV_RAND_CHECK(r) \
do begin \
  if (!(r)) begin \
    $display("%s:%0d: Randomization failed \"%s\"", `__FILE__, `__LINE__, `"r`"); \
    $stop;\
  end\
  end while (0)


module tb_gpio;
  localparam ClkPeriod        = 5ns;
  localparam RstCycles        = 6;
  localparam SimTimeoutCycles = 5000; // Timeout the simulation after 5000 cycles
  localparam ApplTime         = 1ns;
  localparam TestTime         = 4ns;
  localparam DataWidth        = 32;
  localparam AddrWidth        = 32;
  localparam NrGPIOs          = gpio_reg_pkg::GPIOCount;

  // Testbench control signals
  logic clk;
  logic rst_n;
  logic end_of_sim;

  // Generate clock and reset
  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriod ),
    .RstClkCycles ( RstCycles )
  ) i_clk_rst_gen (
    .clk_o ( clk   ),
    .rst_no( rst_n )
  );

  // Test Timeout Module
  sim_timeout #(
    .Cycles(SimTimeoutCycles)
  ) i_sim_timeout(
    .clk_i  ( clk   ),
    .rst_ni ( rst_n )
  );

  // End of test procedure
  initial begin : proc_end_of_test
    wait (end_of_sim);
    repeat (100) @(posedge clk);
    $info("Simulation ended.");
    $stop();
  end

  // Interface Signals
  REG_BUS #(
    .ADDR_WIDTH (AddrWidth),
    .DATA_WIDTH (DataWidth)) s_reg_bus (.clk_i(clk));

  logic [NrGPIOs-1:0]   gpio_in;
  logic [NrGPIOs-1:0]   gpio_in_sync;
  logic [NrGPIOs-1:0]   gpio_out;
  logic [NrGPIOs-1:0]   gpio_tx_en;
  logic                 interrupt;


  // Instantiate DUT
  gpio_intf #(
    .ADDR_WIDTH ( AddrWidth ),
    .DATA_WIDTH ( DataWidth ),
    .NrGPIOs    ( NrGPIOs   )
  ) i_dut (
    .reg_bus        ( s_reg_bus.in                ),
    // Outputs
    .gpio_out       ( gpio_out[NrGPIOs-1:0]       ),
    .gpio_tx_en_o   ( gpio_tx_en[NrGPIOs-1:0]     ),
    .gpio_in_sync_o ( gpio_in_sync[NrGPIOs-1:0]   ),
    .interrupt_o    ( interrupt                   ),
    // Inputs
    .clk_i          ( clk                         ),
    .rst_ni         ( rst_n                       ),
    .gpio_in        ( gpio_in[NrGPIOs-1:0]        )
  );

  // Connect test programm
  test #(
    .NrGPIOs   ( NrGPIOs   ),
    .DataWidth ( DataWidth ),
    .AddrWidth ( AddrWidth ),
    .ApplTime  ( ApplTime  ),
    .TestTime  ( TestTime  )
   ) i_test (
    .end_of_sim_o   ( end_of_sim   ),
    .clk_i          ( clk          ),
    .rst_ni         ( rst_n        ),
    .gpio_in_o      ( gpio_in      ),
    .gpio_out_i     ( gpio_out     ),
    .gpio_tx_en_i   ( gpio_tx_en   ),
    .gpio_in_sync_i ( gpio_in_sync ),
    .interrupt_i    ( interrupt    ),
    .reg_bus        ( s_reg_bus    )
  );
endmodule


program automatic test #(
  parameter NrGPIOs   = 64,
  parameter DataWidth = 32,
  parameter AddrWidth = 32,
  parameter ApplTime,
  parameter TestTime
) (
  output logic               end_of_sim_o,
  input logic                clk_i,
  input logic                rst_ni,
  output logic [NrGPIOs-1:0] gpio_in_o,
  input logic [NrGPIOs-1:0]  gpio_out_i,
  input logic [NrGPIOs-1:0]  gpio_tx_en_i, // 0 -> input, 1 -> output
  input logic [NrGPIOs-1:0]  gpio_in_sync_i, // sampled and synchronized GPIO
  // input.
  input logic                interrupt_i,
  REG_BUS.out                reg_bus
);
  import reg_test::reg_driver;
  import gpio_reg_pkg::*;

  reg_driver #(.AW(AddrWidth), .DW(DataWidth), .TA(ApplTime), .TT(TestTime)) gpio_reg_driver;

  // Debug Signals
  logic [NrGPIOs-1:0][1:0] gpio_modes;
  logic [NrGPIOs-1:0]      gpio_values;


  initial begin: test
    automatic logic [DataWidth-1:0] data;
    automatic logic [AddrWidth-1:0] addr;
    automatic logic [DataWidth/8-1:0] strb = '1;
    automatic logic                   error = 0;
    // Instantiate driver
    gpio_reg_driver = new(reg_bus);
    end_of_sim_o    = 0;

    // Wait until reset
    gpio_reg_driver.reset_master();
    @(posedge rst_ni);

    // Debug
    $info("Check reg  interface behavior");
    addr = GPIO_DUMMY_0_OFFSET;
    data = 32'h190;
    gpio_reg_driver.send_write(addr, data, strb, error);

    // Configure GPIOs with random modes and randomly drive them
    begin : check_output
      // automatic logic [NrGPIOs-1:0][1:0] gpio_modes;
      // automatic logic [NrGPIOs-1:0]      gpio_values;

      `SV_RAND_CHECK(randomize(gpio_modes));
      for (int i = 0; i < NrGPIOs/DataWidth*2; i++) begin : cfg_gpio_modes
        addr = GPIO_GPIO_MODE_0_OFFSET + i*4;
        data = gpio_modes[i*DataWidth/2+:DataWidth/2];
        gpio_reg_driver.send_write(addr, data, strb, error);
        assert(error == 0) else
          $error("Interface write error while writing GPIO mode.");
      end

      // Now drive them and check the behavior of the tx_en and the gpio_out ports
      for (int repetition_idx = 0; repetition_idx < 10; repetition_idx++) begin : for_repetition
        // Set random gpio out values
        `SV_RAND_CHECK(randomize(gpio_values));
        for (int i = 0; i < NrGPIOs/DataWidth; i++) begin
          addr = GPIO_GPIO_OUT_0_OFFSET + i*4;
          data = gpio_values[i*DataWidth+:DataWidth];
          gpio_reg_driver.send_write(addr, data, strb, error);
          assert(error == 0) else
            $error("Interface write error while writing GPIO out values.");
        end

        // Check if gpio_out and gpio_tx_en is correct for every gpio
        for (int gpio_idx = 0; gpio_idx < NrGPIOs; gpio_idx++) begin : for_gpio
          case (gpio_modes[gpio_idx])
            2'b00: begin // Input Only
              assert(gpio_tx_en_i[gpio_idx] == 1'b0) else
                $error("GPIO %d TX  is enabled even though GPIO is supposed to be in Input mode.", gpio_idx);
            end

            2'b01: begin // Push/Pull mode
              assert(gpio_tx_en_i[gpio_idx] == 1'b1) else
                $error("GPIO %d TX is not enabled even though GPIO is suppposed to be in Push/Pull mode.", gpio_idx);
              assert(gpio_out_i[gpio_idx] == gpio_values[gpio_idx]) else
                $error("Got wrong output value on GPIO %d. Was suposed to be %b but was %b.", gpio_idx, gpio_values[gpio_idx], gpio_out_i[gpio_idx]);
            end

            2'b10: begin // Open Drain mode 0, Drive high on value=1
              if (gpio_values[gpio_idx] == 1'b1) begin
                assert(gpio_out_i[gpio_idx] == 1'b1) else
                  $error("Got wrong output value on GPIO %d. Was suposed to be %b but was %b.", gpio_idx, gpio_values[gpio_idx], gpio_out_i[gpio_idx]);
                assert(gpio_tx_en_i[gpio_idx] == 1'b1) else
                  $error("GPIO %d TX enable is not high even thought the GPIO is in Open Drain Mode 0 and the output is driven high.", gpio_idx);
              end else begin
                assert(gpio_tx_en_i[gpio_idx] == 1'b0) else
                  $error("GPIO %d TX enable is asserted even thought the GPIO is in Open Drain Mode 0 and the output is driven low.", gpio_idx);
              end
            end // case: 2'b10

            2'b10: begin // Open Drain mode 1, Drive high on value=1
              if (gpio_values[gpio_idx] == 1'b0) begin
                assert(gpio_out_i[gpio_idx] == 1'b0) else
                  $error("Got wrong output value on GPIO %d. Was suposed to be %b but was %b.", gpio_idx, gpio_values[gpio_idx], gpio_out_i[gpio_idx]);
                assert(gpio_tx_en_i[gpio_idx] == 1'b1) else
                  $error("GPIO %d TX enable is not high even thought the GPIO is in Open Drain Mode 1 and the output is driven low.", gpio_idx);
              end else begin
                assert(gpio_tx_en_i[gpio_idx] == 1'b0) else
                  $error("GPIO %d TX enable is asserted even thought the GPIO is in Open Drain Mode 1 and the output is driven high.", gpio_idx);
              end
            end
          endcase // case (gpio_modes[reg_idx])
        end // for (int gpio_idx = 0; gpio_idx < NrGPIOs; gpio_idx++)
      end // for (int repetion_idx = 0; repetition_idx < 10; repetition_idx++)
    end // block: check_output

    begin :check_interrupts

    end
    end_of_sim_o = 1;
  end // block: logic

endprogram
