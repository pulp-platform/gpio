`include "include/typedef.svh"

module gpio_axi_lite_wrap # (
  /// The width of the address.
  localparam int ADDR_WIDTH = 32,
  /// The width of the data.
  localparam int DATA_WIDTH = 32,
  /// Whether the AXI-Lite W channel should be decoupled with a register. This
  /// can help break long paths at the expense of registers.
  parameter bit DECOUPLE_W = 1,
  /// AXI-Lite request struct type.
  parameter type axi_lite_req_t = logic,
  /// AXI-Lite response struct type.
  parameter type axi_lite_rsp_t = logic,
)(
  input logic                clk_i,
  input logic                rst_ni,
  input logic [NrGPIOs-1:0]  gpio_in,
  output logic [NrGPIOs-1:0] gpio_out,
  output logic [NrGPIOs-1:0] gpio_dir_out, // 0 -> input, 1 -> output
  output logic [NrGPIOs-1:0] gpio_in_sync, // sampled and synchronized GPIO
  // input.
  output logic               interrupt,
  input axi_lite_req_t       axi_lite_req_i,
  output axi_lite_rsp_t      axi_lite_rsp_o
);

  typedef logic [31:0] addr_t;
  typedef logic [31:0] data_t;
  typedef logic [3:0]  strb_t;
  `REG_BUS_TYPEDEFF_ALL(reg_bus, addr_t, data_t, strb_t)

  reg_bus_req_t s_reg_req;
  reg_bus_rsp_t s_reg_rsp;

  axi_lite_to_reg #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .BUFFER_DEPTH(1),
    .DECOUPLE_W(0),
    .axi_lite_req_t(axi_lite_req_t),
    .axi_lite_rsp_t(axi_lite_rsp_t),
    .reg_req_t(reg_bus_req_t),
    .reg_rsp_t(reg_bus_rsp_t)
  ) i_axi_lite_to_reg (
    .clk_i,
    .rst_ni,
    .axi_lite_req_i,
    .axi_lite_rsp_o,
    .reg_req_o(s_reg_req),
    .reg_rsp_i(s_reg_rsp)
  );


  gpio #(
    .reg_req_t(reg_req_t)
  ) i_gpio (
    .clk_i,
    .rst_ni,
    .gpio_in,
    .gpio_out,
    .gpio_dir_out,
    .gpio_in_sync,
    .interrupt,
    .reg_req_i(s_reg_req),
    .reg_rsp_o(s_reg_rsp)
  );

endmodule

module gpio_axi_lite_wrap_intf # (
                             /// The width of the address.
                             localparam int ADDR_WIDTH = 32,
                             /// The width of the data.
                             localparam int DATA_WIDTH = 32,
                             /// Whether the AXI-Lite W channel should be decoupled with a register. This
                             /// can help break long paths at the expense of registers.
                             parameter bit  DECOUPLE_W = 1,
                             )(
                               input logic                clk_i,
                               input logic                rst_ni,
                               input logic [NrGPIOs-1:0]  gpio_in,
                               output logic [NrGPIOs-1:0] gpio_out,
                               output logic [NrGPIOs-1:0] gpio_dir_out, // 0 -> input, 1 -> output
                               output logic [NrGPIOs-1:0] gpio_in_sync, // sampled and synchronized GPIO
                               // input.
                               output logic               interrupt,
                               AXI_LITE.Slave             axi_i
                               );

  REG_BUS #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) s_reg_bus();

  axi_lite_to_reg_inf #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .BUFFER_DEPTH(1),
    .DECOUPLE_W(0)
  ) i_axi_lite_to_reg (
    .clk_i,
    .rst_ni,
    .axi_i,
    .reg_o(s_reg_bus)
  );

  gpio_intf i_gpio_intf (
    .clk_i,
    .rst_ni,
    .gpio_in,
    .gpio_out,
    .gpio_dir_out,
    .gpio_in_sync,
    .interrupt,
    .reg_bus(s_reg_bus)
  );

endmodule
