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


`include "include/typedef.svh"

module gpio #(
  // Regbus request struct type.
  parameter type reg_req_t = logic,
  /// Regbus response struct type.
  parameter type reg_rsp_t = logic,
  localparam int unsigned NrGPIOs = 64 // In order to reparametrize this module,
                                       // alter the hjson register file
                                       // description accordingly and regenerate
                                       // the register file using reggen.
) (
  input logic                clk_i,
  input logic                rst_ni,
  input logic [NrGPIOs-1:0]  gpio_in,
  output logic [NrGPIOs-1:0] gpio_out,
  output logic [NrGPIOs-1:0] gpio_dir_out, // 0 -> input, 1 -> output
  output logic [NrGPIOs-1:0] gpio_in_sync, // sampled and synchronized GPIO
                                           // input.
  output logic               interrupt,
  input                      reg_req_t reg_req_i,
  output                     reg_rsp_t reg_rsp_o
);
  if (NrGPIOs % 32 != 0)
    $error("Only multiples of 32 GPIOs are supported.");

  import gpio_reg_pkg::*;

  // Internal Signals
  gpio_reg2hw_t s_reg2hw;
  gpio_hw2reg_t s_hw2reg;

  logic [NrGPIOs-1:0] s_gpio_rise_edge;
  logic [NrGPIOs-1:0] s_gpio_rise_intrpt_mask;
  logic [NrGPIOs-1:0] s_gpio_fall_edge;
  logic [NrGPIOs-1:0] s_gpio_fall_intrpt_mask;

  logic [NrGPIOs/32-1:0][31:0] interrupts;


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

  // Instantiate logic for individual gpios in blocks of 32
  for (genvar reg_idx = 0; reg_idx < NrGPIOs/32; reg_idx++) begin : gen_32_gpios
    for (genvar i = 0; i < 32; i++) begin : gen_gpio
      // Instantiate synchronizer to synchronize input to sampling clock
      sync_wedge #(
        .STAGES(2)
      ) i_sync_gpio_input(
        .clk_i,
        .rst_ni,
        .en_i(s_reg2hw.gpio_en[reg_idx].q[i] && ~s_reg2hw.gpio_dir),
        .serial_i(gpio_in[reg_idx*32+i]),
        .r_edge_o(s_gpio_rise_edge[reg_idx*32+i]),
        .f_edge_o(s_gpio_fall_edge[reg_idx*32+i]),
        .serial_o(s_hw2reg.gpio_in[reg_idx].d[i])
      );

      // Output assignments
      assign gpio_out[reg_idx*32+i] = s_reg2hw.gpio_out[reg_idx].q[i];
      assign gpio_dir_out[reg_idx*32+i] = s_reg2hw.gpio_dir[reg_idx].q[i];

      // Wire individual interrupts
      assign s_gpio_rise_intrpt_mask[reg_idx*32+i] = s_reg2hw.intrpt_rise_en[reg_idx].q[i];
      assign s_gpio_fall_intrpt_mask[reg_idx*32+i] = s_reg2hw.intrpt_fall_en[reg_idx].q[i];
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
    // Only update if there are any pending interrupts in this 32-bit register
    assign s_hw2reg.intrpt_status[reg_idx].de = |interrupts[reg_idx];
  end


endmodule : gpio

module gpio_intf (
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

  typedef logic [31:0]         addr_t;
  typedef logic [31:0]         data_t;
  typedef logic [3:0]          strb_t;
  `REG_BUS_TYPEDEFF_ALL(reg_bus, addr_t, data_t, strb_t)

  reg_bus_req_t s_reg_req;
  reg_bus_rsp_t s_reg_rsp;

  `REG_BUS_ASSIGN_TO_REQ(s_reg_req, reg_bus);
  `REG_BUS_ASSIGN_FROM_RSP(s_reg_req, reg_bus);

  gpio #(
    .reg_req_t(reg_bus_req_t),
    .reg_rsp_t(reg_bus_rsp_t)
  ) (
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
