// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Package auto-generated by `reggen` containing data structure

package gpio_reg_pkg;

  // Param list
  parameter int GPIOCount = 56;

  // Address widths within the block
  parameter int BlockAw = 11;

  ////////////////////////////
  // Typedefs for registers //
  ////////////////////////////

  typedef struct packed {
    struct packed {
      logic        q;
    } intrpt_mode;
    struct packed {
      logic        q;
    } reserved;
  } gpio_reg2hw_cfg_reg_t;

  typedef struct packed {
    logic [1:0]  q;
  } gpio_reg2hw_gpio_mode_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_gpio_en_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_gpio_out_mreg_t;

  typedef struct packed {
    logic        q;
    logic        qe;
  } gpio_reg2hw_gpio_set_mreg_t;

  typedef struct packed {
    logic        q;
    logic        qe;
  } gpio_reg2hw_gpio_clear_mreg_t;

  typedef struct packed {
    logic        q;
    logic        qe;
  } gpio_reg2hw_gpio_toggle_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_rise_en_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_fall_en_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_lvl_high_en_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_lvl_low_en_mreg_t;

  typedef struct packed {
    logic        q;
    logic        qe;
  } gpio_reg2hw_intrpt_status_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_rise_status_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_fall_status_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_lvl_high_status_mreg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_intrpt_lvl_low_status_mreg_t;

  typedef struct packed {
    struct packed {
      logic [9:0] d;
    } gpio_cnt;
    struct packed {
      logic [9:0] d;
    } version;
  } gpio_hw2reg_info_reg_t;

  typedef struct packed {
    logic        d;
  } gpio_hw2reg_gpio_in_mreg_t;

  typedef struct packed {
    logic        d;
    logic        de;
  } gpio_hw2reg_gpio_out_mreg_t;

  typedef struct packed {
    logic        d;
  } gpio_hw2reg_intrpt_status_mreg_t;

  typedef struct packed {
    logic        d;
    logic        de;
  } gpio_hw2reg_intrpt_rise_status_mreg_t;

  typedef struct packed {
    logic        d;
    logic        de;
  } gpio_hw2reg_intrpt_fall_status_mreg_t;

  typedef struct packed {
    logic        d;
    logic        de;
  } gpio_hw2reg_intrpt_lvl_high_status_mreg_t;

  typedef struct packed {
    logic        d;
    logic        de;
  } gpio_hw2reg_intrpt_lvl_low_status_mreg_t;

  // Register -> HW type
  typedef struct packed {
    gpio_reg2hw_cfg_reg_t cfg; // [1121:1120]
    gpio_reg2hw_gpio_mode_mreg_t [55:0] gpio_mode; // [1119:1008]
    gpio_reg2hw_gpio_en_mreg_t [55:0] gpio_en; // [1007:952]
    gpio_reg2hw_gpio_out_mreg_t [55:0] gpio_out; // [951:896]
    gpio_reg2hw_gpio_set_mreg_t [55:0] gpio_set; // [895:784]
    gpio_reg2hw_gpio_clear_mreg_t [55:0] gpio_clear; // [783:672]
    gpio_reg2hw_gpio_toggle_mreg_t [55:0] gpio_toggle; // [671:560]
    gpio_reg2hw_intrpt_rise_en_mreg_t [55:0] intrpt_rise_en; // [559:504]
    gpio_reg2hw_intrpt_fall_en_mreg_t [55:0] intrpt_fall_en; // [503:448]
    gpio_reg2hw_intrpt_lvl_high_en_mreg_t [55:0] intrpt_lvl_high_en; // [447:392]
    gpio_reg2hw_intrpt_lvl_low_en_mreg_t [55:0] intrpt_lvl_low_en; // [391:336]
    gpio_reg2hw_intrpt_status_mreg_t [55:0] intrpt_status; // [335:224]
    gpio_reg2hw_intrpt_rise_status_mreg_t [55:0] intrpt_rise_status; // [223:168]
    gpio_reg2hw_intrpt_fall_status_mreg_t [55:0] intrpt_fall_status; // [167:112]
    gpio_reg2hw_intrpt_lvl_high_status_mreg_t [55:0] intrpt_lvl_high_status; // [111:56]
    gpio_reg2hw_intrpt_lvl_low_status_mreg_t [55:0] intrpt_lvl_low_status; // [55:0]
  } gpio_reg2hw_t;

  // HW -> register type
  typedef struct packed {
    gpio_hw2reg_info_reg_t info; // [691:672]
    gpio_hw2reg_gpio_in_mreg_t [55:0] gpio_in; // [671:616]
    gpio_hw2reg_gpio_out_mreg_t [55:0] gpio_out; // [615:504]
    gpio_hw2reg_intrpt_status_mreg_t [55:0] intrpt_status; // [503:448]
    gpio_hw2reg_intrpt_rise_status_mreg_t [55:0] intrpt_rise_status; // [447:336]
    gpio_hw2reg_intrpt_fall_status_mreg_t [55:0] intrpt_fall_status; // [335:224]
    gpio_hw2reg_intrpt_lvl_high_status_mreg_t [55:0] intrpt_lvl_high_status; // [223:112]
    gpio_hw2reg_intrpt_lvl_low_status_mreg_t [55:0] intrpt_lvl_low_status; // [111:0]
  } gpio_hw2reg_t;

  // Register offsets
  parameter logic [BlockAw-1:0] GPIO_INFO_OFFSET = 11'h 0;
  parameter logic [BlockAw-1:0] GPIO_CFG_OFFSET = 11'h 4;
  parameter logic [BlockAw-1:0] GPIO_GPIO_MODE_0_OFFSET = 11'h 8;
  parameter logic [BlockAw-1:0] GPIO_GPIO_MODE_1_OFFSET = 11'h c;
  parameter logic [BlockAw-1:0] GPIO_GPIO_MODE_2_OFFSET = 11'h 10;
  parameter logic [BlockAw-1:0] GPIO_GPIO_MODE_3_OFFSET = 11'h 14;
  parameter logic [BlockAw-1:0] GPIO_GPIO_EN_0_OFFSET = 11'h 80;
  parameter logic [BlockAw-1:0] GPIO_GPIO_EN_1_OFFSET = 11'h 84;
  parameter logic [BlockAw-1:0] GPIO_GPIO_IN_0_OFFSET = 11'h 100;
  parameter logic [BlockAw-1:0] GPIO_GPIO_IN_1_OFFSET = 11'h 104;
  parameter logic [BlockAw-1:0] GPIO_GPIO_OUT_0_OFFSET = 11'h 180;
  parameter logic [BlockAw-1:0] GPIO_GPIO_OUT_1_OFFSET = 11'h 184;
  parameter logic [BlockAw-1:0] GPIO_GPIO_SET_0_OFFSET = 11'h 200;
  parameter logic [BlockAw-1:0] GPIO_GPIO_SET_1_OFFSET = 11'h 204;
  parameter logic [BlockAw-1:0] GPIO_GPIO_CLEAR_0_OFFSET = 11'h 280;
  parameter logic [BlockAw-1:0] GPIO_GPIO_CLEAR_1_OFFSET = 11'h 284;
  parameter logic [BlockAw-1:0] GPIO_GPIO_TOGGLE_0_OFFSET = 11'h 300;
  parameter logic [BlockAw-1:0] GPIO_GPIO_TOGGLE_1_OFFSET = 11'h 304;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_RISE_EN_0_OFFSET = 11'h 380;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_RISE_EN_1_OFFSET = 11'h 384;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_FALL_EN_0_OFFSET = 11'h 400;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_FALL_EN_1_OFFSET = 11'h 404;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_HIGH_EN_0_OFFSET = 11'h 480;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_HIGH_EN_1_OFFSET = 11'h 484;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_LOW_EN_0_OFFSET = 11'h 500;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_LOW_EN_1_OFFSET = 11'h 504;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_STATUS_0_OFFSET = 11'h 580;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_STATUS_1_OFFSET = 11'h 584;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_RISE_STATUS_0_OFFSET = 11'h 600;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_RISE_STATUS_1_OFFSET = 11'h 604;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_FALL_STATUS_0_OFFSET = 11'h 680;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_FALL_STATUS_1_OFFSET = 11'h 684;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_HIGH_STATUS_0_OFFSET = 11'h 700;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_HIGH_STATUS_1_OFFSET = 11'h 704;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_LOW_STATUS_0_OFFSET = 11'h 780;
  parameter logic [BlockAw-1:0] GPIO_INTRPT_LVL_LOW_STATUS_1_OFFSET = 11'h 784;

  // Reset values for hwext registers and their fields
  parameter logic [19:0] GPIO_INFO_RESVAL = 20'h 400;
  parameter logic [9:0] GPIO_INFO_VERSION_RESVAL = 10'h 1;
  parameter logic [31:0] GPIO_GPIO_IN_0_RESVAL = 32'h 0;
  parameter logic [23:0] GPIO_GPIO_IN_1_RESVAL = 24'h 0;
  parameter logic [31:0] GPIO_GPIO_SET_0_RESVAL = 32'h 0;
  parameter logic [23:0] GPIO_GPIO_SET_1_RESVAL = 24'h 0;
  parameter logic [31:0] GPIO_GPIO_CLEAR_0_RESVAL = 32'h 0;
  parameter logic [23:0] GPIO_GPIO_CLEAR_1_RESVAL = 24'h 0;
  parameter logic [31:0] GPIO_GPIO_TOGGLE_0_RESVAL = 32'h 0;
  parameter logic [23:0] GPIO_GPIO_TOGGLE_1_RESVAL = 24'h 0;
  parameter logic [31:0] GPIO_INTRPT_STATUS_0_RESVAL = 32'h 0;
  parameter logic [23:0] GPIO_INTRPT_STATUS_1_RESVAL = 24'h 0;

  // Register index
  typedef enum int {
    GPIO_INFO,
    GPIO_CFG,
    GPIO_GPIO_MODE_0,
    GPIO_GPIO_MODE_1,
    GPIO_GPIO_MODE_2,
    GPIO_GPIO_MODE_3,
    GPIO_GPIO_EN_0,
    GPIO_GPIO_EN_1,
    GPIO_GPIO_IN_0,
    GPIO_GPIO_IN_1,
    GPIO_GPIO_OUT_0,
    GPIO_GPIO_OUT_1,
    GPIO_GPIO_SET_0,
    GPIO_GPIO_SET_1,
    GPIO_GPIO_CLEAR_0,
    GPIO_GPIO_CLEAR_1,
    GPIO_GPIO_TOGGLE_0,
    GPIO_GPIO_TOGGLE_1,
    GPIO_INTRPT_RISE_EN_0,
    GPIO_INTRPT_RISE_EN_1,
    GPIO_INTRPT_FALL_EN_0,
    GPIO_INTRPT_FALL_EN_1,
    GPIO_INTRPT_LVL_HIGH_EN_0,
    GPIO_INTRPT_LVL_HIGH_EN_1,
    GPIO_INTRPT_LVL_LOW_EN_0,
    GPIO_INTRPT_LVL_LOW_EN_1,
    GPIO_INTRPT_STATUS_0,
    GPIO_INTRPT_STATUS_1,
    GPIO_INTRPT_RISE_STATUS_0,
    GPIO_INTRPT_RISE_STATUS_1,
    GPIO_INTRPT_FALL_STATUS_0,
    GPIO_INTRPT_FALL_STATUS_1,
    GPIO_INTRPT_LVL_HIGH_STATUS_0,
    GPIO_INTRPT_LVL_HIGH_STATUS_1,
    GPIO_INTRPT_LVL_LOW_STATUS_0,
    GPIO_INTRPT_LVL_LOW_STATUS_1
  } gpio_id_e;

  // Register width information to check illegal writes
  parameter logic [3:0] GPIO_PERMIT [36] = '{
    4'b 0111, // index[ 0] GPIO_INFO
    4'b 0001, // index[ 1] GPIO_CFG
    4'b 1111, // index[ 2] GPIO_GPIO_MODE_0
    4'b 1111, // index[ 3] GPIO_GPIO_MODE_1
    4'b 1111, // index[ 4] GPIO_GPIO_MODE_2
    4'b 0011, // index[ 5] GPIO_GPIO_MODE_3
    4'b 1111, // index[ 6] GPIO_GPIO_EN_0
    4'b 0111, // index[ 7] GPIO_GPIO_EN_1
    4'b 1111, // index[ 8] GPIO_GPIO_IN_0
    4'b 0111, // index[ 9] GPIO_GPIO_IN_1
    4'b 1111, // index[10] GPIO_GPIO_OUT_0
    4'b 0111, // index[11] GPIO_GPIO_OUT_1
    4'b 1111, // index[12] GPIO_GPIO_SET_0
    4'b 0111, // index[13] GPIO_GPIO_SET_1
    4'b 1111, // index[14] GPIO_GPIO_CLEAR_0
    4'b 0111, // index[15] GPIO_GPIO_CLEAR_1
    4'b 1111, // index[16] GPIO_GPIO_TOGGLE_0
    4'b 0111, // index[17] GPIO_GPIO_TOGGLE_1
    4'b 1111, // index[18] GPIO_INTRPT_RISE_EN_0
    4'b 0111, // index[19] GPIO_INTRPT_RISE_EN_1
    4'b 1111, // index[20] GPIO_INTRPT_FALL_EN_0
    4'b 0111, // index[21] GPIO_INTRPT_FALL_EN_1
    4'b 1111, // index[22] GPIO_INTRPT_LVL_HIGH_EN_0
    4'b 0111, // index[23] GPIO_INTRPT_LVL_HIGH_EN_1
    4'b 1111, // index[24] GPIO_INTRPT_LVL_LOW_EN_0
    4'b 0111, // index[25] GPIO_INTRPT_LVL_LOW_EN_1
    4'b 1111, // index[26] GPIO_INTRPT_STATUS_0
    4'b 0111, // index[27] GPIO_INTRPT_STATUS_1
    4'b 1111, // index[28] GPIO_INTRPT_RISE_STATUS_0
    4'b 0111, // index[29] GPIO_INTRPT_RISE_STATUS_1
    4'b 1111, // index[30] GPIO_INTRPT_FALL_STATUS_0
    4'b 0111, // index[31] GPIO_INTRPT_FALL_STATUS_1
    4'b 1111, // index[32] GPIO_INTRPT_LVL_HIGH_STATUS_0
    4'b 0111, // index[33] GPIO_INTRPT_LVL_HIGH_STATUS_1
    4'b 1111, // index[34] GPIO_INTRPT_LVL_LOW_STATUS_0
    4'b 0111  // index[35] GPIO_INTRPT_LVL_LOW_STATUS_1
  };

endpackage

