# GPIO Peripheral

This repository provides an simple GPIO peripheral with integrated register file
to control 64 GPIOs (by default). The peripheral supports rising, falling or
either edge interrupts and performs two stage synchronization of the inputs to
resolve potential metastability.

The interface to the peripheral is the lightweight [register_interface protocol](https://github.com/pulp-platform/register_interface ).
However, the repository contains convience wrappers to attach AXI-lite or APB
buses for control. Each module in the repository contains an additional wrapper
at the bottom of the respective source files for the users that prefer
SystemVerilog interfaces over hierarchical structs.

# Ports
| **Signal Name** | **Direction** | **Description**                                                                                                                         |
| --------------- | ------------- | ---------------                                                                                                                         |
| `clk_i`         | *input*       | Primary input clock. The control interface is suposed to be synchronous to this clock.                                                  |
| `rst_ni`        | *input*       | Asynchronous active-low reset                                                                                                           |
| `gpio_in`       | *input*       | GPIO input signals from IO Pads (Pad -> SoC) signal.                                                                                    |
| `gpio_out`      | *output*      | GPIO output signals to IO Pads (SoC -> Pad) signal.                                                                                     |
| `gpio_dir_out`  | *output*      | GPIO direction signals. This signal is supposed to control the direction of the corresponding IO Pad. 0 -> RX (inpu), 1 -> TX (output). |
| `gpio_in_sync`  | *input*       | Synchronized GPIO input signals. This port provides the `gpio_in` signal synchronized to `clk_i`.                                       |
| `interrupt`     | *output*      | Global interrupt line. The interrupt line is asserted for one `clk_i` whenever an unmasked interrupt on one of the GPIOs arrives.       |
| `reg_req_i`     | *input*       | Control interface request side using register interface protocol.                                                                       |
| `reg_rsp_o`     | *output*      | Control interface request side using register_interface protocol.                                                                       |

# Register Map
The registers of this module are all defined in the `gpio_regs.hjson` file which
is used to auto-generate the actual SV register file using [lowRISCs reggen tool](https://docs.opentitan.org/doc/rm/register_tool/ ).

Here is a summary of the registers
