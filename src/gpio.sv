//-----------------------------------------------------------------------------
// Title         : GPIO Peripheral
//-----------------------------------------------------------------------------
// File          : gpio.sv
// Author        : Manuel Eggimann  <meggimann@iis.ee.ethz.ch>
// Created       : 06.05.2021
//-----------------------------------------------------------------------------
// Description :
// This Module contains a very simple but clean implementation of a GPIO
// peripheral. The is controlled through a lightweight reg_bus interface. At the
// bottom of this file there is a SV interface wrapper for the module.
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


`include "register_interface/typedef.svh"
`include "register_interface/assign.svh"

module gpio #(
  /// Data width of the reg_bus
  parameter int unsigned DATA_WIDTH = 32,
  /// Regbus request struct type.
  parameter type reg_req_t          = logic,
  /// Regbus response struct type.
  parameter type reg_rsp_t          = logic,
  /// The number of GPIOs in this module. This parameter can only be changed if
  /// the corresponding register file is regenerated with the same number of
  /// GPIOs. In general, only multiples of the DATA_WIDTH are supported. The
  /// module will error out during elaboration if the given parameter does not
  /// match the number of defined GPIOs in the register file.
  parameter int unsigned NrGPIOs    = 64
) (
  /// Primary input clock. The control interface is suposed to be synchronous to
  /// this clock.
  input logic                clk_i,
  /// Asynchronous active-low reset
  input logic                rst_ni,
  /// GPIO input signals from IO Pads (Pad -> SoC) signal.
  input logic [NrGPIOs-1:0]  gpio_in,
  /// GPIO output signals to IO Pads (SoC -> Pad) signal.
  output logic [NrGPIOs-1:0] gpio_out,
  /// GPIO direction signals. This signal is supposed to control the direction of
  /// the corresponding IO Pad. 0 -> RX (inpu), 1 -> TX (output).
  output logic [NrGPIOs-1:0] gpio_dir_out, // 0 -> input, 1 -> output
  /// Synchronized GPIO input signals. This port provides the `gpio_in` signal
  /// synchronized to `clk_i`.
  output logic [NrGPIOs-1:0] gpio_in_sync,
  /// Global interrupt line. The interrupt line is asserted for one `clk_i`
  /// whenever an unmasked interrupt on one of the GPIOs arrives.
  output logic               interrupt,
  /// Control interface request side using register_interface protocol.
  input                      reg_req_t reg_req_i,
  /// Control interface request side using register_interface protocol.
  output                     reg_rsp_t reg_rsp_o
);
  import gpio_reg_pkg::*;

  // Elaboration time SystemTasks.
  if (DATA_WIDTH != $bits(gpio_reg_pkg::gpio_reg2hw_gpio_dir_mreg_t))
    $error("The data width of the register interface does not match the data width of the generated register file.");

  if (NrGPIOs != DATA_WIDTH * gpio_reg_pkg::GPIORegCount)
    $error("Wrong parametrization of the generated regfile. The number of GPIOs in the register (%d * %d) file does not match the parametrization. Did you forget to regenrate the register file for your parametrization?", gpio_reg_pkg::GPIORegCount, DATA_WIDTH);

  // Internal Signals
  gpio_reg2hw_t s_reg2hw;
  gpio_hw2reg_t s_hw2reg;

  logic [NrGPIOs-1:0] s_gpio_rise_edge;
  logic [NrGPIOs-1:0] s_gpio_rise_intrpt_mask;
  logic [NrGPIOs-1:0] s_gpio_fall_edge;
  logic [NrGPIOs-1:0] s_gpio_fall_intrpt_mask;

  logic [NrGPIOs/DATA_WIDTH-1:0][DATA_WIDTH-1:0] interrupts;


  // Instantiate auto-generated register file
  gpio_reg_top #(
    .reg_req_t(reg_req_t),
    .reg_rsp_t(reg_rsp_t)
  ) i_reg_file (
    .clk_i,
    .rst_ni,
    .reg_req_i,
    .reg_rsp_o,
    .reg2hw(s_reg2hw),
    .hw2reg(s_hw2reg),
    .devmode_i(1'b1)
  );

  // Instantiate logic for individual gpios in blocks of DATA_WIDTH
  for (genvar reg_idx = 0; reg_idx < NrGPIOs/DATA_WIDTH; reg_idx++) begin : gen_block_gpios
    for (genvar i = 0; i < DATA_WIDTH; i++) begin : gen_gpio
      // Instantiate synchronizer to synchronize input to sampling clock
      sync_wedge #(
        .STAGES(2)
      ) i_sync_gpio_input(
        .clk_i,
        .rst_ni,
        .en_i(s_reg2hw.gpio_en[reg_idx].q[i] && ~s_reg2hw.gpio_dir),
        .serial_i(gpio_in[reg_idx*DATA_WIDTH+i]),
        .r_edge_o(s_gpio_rise_edge[reg_idx*DATA_WIDTH+i]),
        .f_edge_o(s_gpio_fall_edge[reg_idx*DATA_WIDTH+i]),
        .serial_o(s_hw2reg.gpio_in[reg_idx].d[i])
      );

      // Output assignments
      assign gpio_out[reg_idx*DATA_WIDTH+i] = s_reg2hw.gpio_out[reg_idx].q[i];
      assign gpio_dir_out[reg_idx*DATA_WIDTH+i] = s_reg2hw.gpio_dir[reg_idx].q[i];

      // Wire individual interrupts
      assign s_gpio_rise_intrpt_mask[reg_idx*DATA_WIDTH+i] = s_reg2hw.intrpt_rise_en[reg_idx].q[i];
      assign s_gpio_fall_intrpt_mask[reg_idx*DATA_WIDTH+i] = s_reg2hw.intrpt_fall_en[reg_idx].q[i];
      assign interrupts[reg_idx][i]                = (s_gpio_rise_edge & s_gpio_rise_intrpt_mask) | (s_gpio_fall_edge & s_gpio_fall_intrpt_mask);

    end // block: gen_gpio

    // GPIO set, clear and toggle logic
    always_comb begin
      unique if (s_reg2hw.gpio_set[reg_idx].qe) begin
        s_hw2reg.gpio_out[reg_idx].d = s_reg2hw.gpio_out[reg_idx].q | s_reg2hw.gpio_set[reg_idx].q;
        s_hw2reg.gpio_out[reg_idx].de = 1'b1;
      end else if (s_reg2hw.gpio_clear[reg_idx].qe) begin
        s_hw2reg.gpio_out[reg_idx].d = s_reg2hw.gpio_out[reg_idx].q & ~s_reg2hw.gpio_clear[reg_idx].q;
        s_hw2reg.gpio_out[reg_idx].de = 1'b1;
      end else if (s_reg2hw.gpio_toggle[reg_idx].qe) begin
        s_hw2reg.gpio_out[reg_idx].d = s_reg2hw.gpio_out[reg_idx].q ^ s_reg2hw.gpio_toggle[reg_idx].q;
        s_hw2reg.gpio_out[reg_idx].de = 1'b1;
      end else begin
        s_hw2reg.gpio_out[reg_idx].d = s_reg2hw.gpio_out[reg_idx].q;
        s_hw2reg.gpio_out[reg_idx].de = 1'b0;
      end
    end

    //Wire interrupt registers
    assign s_hw2reg.intrpt_status[reg_idx].d = interrupts[reg_idx] | s_reg2hw.intrpt_status[reg_idx].q;
    // Only update if there are any pending interrupts in this register
    assign s_hw2reg.intrpt_status[reg_idx].de = |interrupts[reg_idx];
  end


endmodule : gpio

module gpio_intf #(
  /// ADDR_WIDTH of the reg_bus interface
  parameter int unsigned  ADDR_WIDTH = 32,
  /// DATA_WIDTH of the reg_bus interface
  parameter int unsigned  DATA_WIDTH = 32,
  /// The number of GPIOs in this module. This parameter can only be changed if
  /// the corresponding register file is regenerated with the same number of
  /// GPIOs. In general, only multiples of the DATA_WIDTH are supported. The
  /// module will error out during elaboration if the given parameter does not
  /// match the number of defined GPIOs in the register file.
  parameter int unsigned  NrGPIOs    = 64,
  localparam int unsigned STRB_WIDTH = DATA_WIDTH/8
) (
  input logic                clk_i,
  input logic                rst_ni,
  input logic [NrGPIOs-1:0]  gpio_in,
  output logic [NrGPIOs-1:0] gpio_out,
  output logic [NrGPIOs-1:0] gpio_dir_out, // 0 -> input, 1 -> output
  output logic [NrGPIOs-1:0] gpio_in_sync, // sampled and synchronized GPIO
  // input.
  output logic               interrupt,
  REG_BUS.in                 reg_bus
);

  // Define structs for reg_bus
  typedef logic [ADDR_WIDTH-1:0] addr_t;
  typedef logic [DATA_WIDTH-1:0] data_t;
  typedef logic [STRB_WIDTH-1:0] strb_t;
  `REG_BUS_TYPEDEF_ALL(reg_bus, addr_t, data_t, strb_t)

  reg_bus_req_t s_reg_req;
  reg_bus_rsp_t s_reg_rsp;

  // Assign SV interface to structs
  `REG_BUS_ASSIGN_TO_REQ(s_reg_req, reg_bus);
  `REG_BUS_ASSIGN_FROM_RSP(reg_bus, s_reg_rsp);

  gpio #(
    .reg_req_t(reg_bus_req_t),
    .reg_rsp_t(reg_bus_rsp_t)
  ) i_gpio (
     .clk_i,
     .rst_ni,
     .gpio_in,
     .gpio_out,
     .gpio_dir_out, // 0 -> input, 1 -> output
     .gpio_in_sync, // sampled and synchronized GPIO

     .interrupt,
     .reg_req_i(s_reg_req),
     .reg_rsp_o(s_reg_rsp)
  );

endmodule : gpio_intf
