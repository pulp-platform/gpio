module gpio_input_stage #(
  parameter NrSyncStages=2
) (
   input logic  clk_i,
   input logic  rst_ni,
   input logic  en_i,
   input logic  serial_i,
   output logic r_edge_o,
   output logic f_edge_o,
   output logic serial_o
);

  logic         clk;
  logic         serial, serial_q;

  assign serial_o =  serial_q;
  assign f_edge_o = (~serial) & serial_q;
  assign r_edge_o = serial & (~serial_q);

  pulp_clock_gating i_pulp_clock_gating (
    .clk_i,
    .en_i,
    .test_en_i ( 1'b0 ),
    .clk_o     ( clk  )
  );

  sync #(
    .STAGES (NrSyncStages)
  ) i_sync (
    .clk_i(clk),
    .rst_ni,
    .serial_i,
    .serial_o ( serial )
  );

  always_ff @(posedge clk, negedge rst_ni) begin
    if (!rst_ni) begin
      serial_q <= 1'b0;
    end else begin
      serial_q <= serial;
    end
  end


endmodule : gpio_input_stage
