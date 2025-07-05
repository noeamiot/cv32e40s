// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
/**
 * Ibex simple system based simple system for cv32e40s
 *
 * This is a basic system consisting of an ibex, a 1 MB sram for instruction/data
 * and a small memory mapped control module for outputting ASCII text and
 * controlling/halting the simulation from the software running on the ibex.
 *
 * It is designed to be used with verilator but should work with other
 * simulators, a small amount of work may be required to support the
 * simulator_ctrl module.
 */

module simple_system (
  input IO_CLK,
  input IO_RST_N
);
  parameter SRAMInitFile = "";

  logic clk_sys = 1'b0, rst_sys_n;

  typedef enum logic {
    CoreD
  } bus_host_e;

  typedef enum logic[0:0] {
    Ram
  } bus_device_e;

  localparam int NrDevices = 1;
  localparam int NrHosts = 1;

  // host and device signals
  logic           host_req    [NrHosts];
  logic           host_gnt    [NrHosts];
  logic [31:0]    host_addr   [NrHosts];
  logic           host_we     [NrHosts];
  logic [ 3:0]    host_be     [NrHosts];
  logic [31:0]    host_wdata  [NrHosts];
  logic           host_rvalid [NrHosts];
  logic [31:0]    host_rdata  [NrHosts];
  logic           host_err    [NrHosts];

  logic [6:0]     data_rdata_intg;
  logic [6:0]     instr_rdata_intg;

  // devices (slaves)
  logic           device_req    [NrDevices];
  logic [31:0]    device_addr   [NrDevices];
  logic           device_we     [NrDevices];
  logic [ 3:0]    device_be     [NrDevices];
  logic [31:0]    device_wdata  [NrDevices];
  logic           device_rvalid [NrDevices];
  logic [31:0]    device_rdata  [NrDevices];
  logic           device_err    [NrDevices];

  // Device address mapping
  logic [31:0] cfg_device_addr_base [NrDevices];
  logic [31:0] cfg_device_addr_mask [NrDevices];
  assign cfg_device_addr_base[Ram] = 32'h100000;
  assign cfg_device_addr_mask[Ram] = ~32'hFFFFF; // 1 MB

  // Instruction fetch signals
  logic instr_req;
  logic instr_gnt;
  logic instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;
  logic instr_err;

  assign instr_gnt = instr_req;
  assign instr_err = '0;

  assign clk_sys = IO_CLK;
  assign rst_sys_n = IO_RST_N;

  // Tie-off unused error signals
  assign device_err[Ram] = 1'b0;

  bus #(
    .NrDevices    ( NrDevices ),
    .NrHosts      ( NrHosts   ),
    .DataWidth    ( 32        ),
    .AddressWidth ( 32        )
  ) u_bus (
    .clk_i               (clk_sys),
    .rst_ni              (rst_sys_n),

    .host_req_i          (host_req     ),
    .host_gnt_o          (host_gnt     ),
    .host_addr_i         (host_addr    ),
    .host_we_i           (host_we      ),
    .host_be_i           (host_be      ),
    .host_wdata_i        (host_wdata   ),
    .host_rvalid_o       (host_rvalid  ),
    .host_rdata_o        (host_rdata   ),
    .host_err_o          (host_err     ),

    .device_req_o        (device_req   ),
    .device_addr_o       (device_addr  ),
    .device_we_o         (device_we    ),
    .device_be_o         (device_be    ),
    .device_wdata_o      (device_wdata ),
    .device_rvalid_i     (device_rvalid),
    .device_rdata_i      (device_rdata ),
    .device_err_i        (device_err   ),

    .cfg_device_addr_base,
    .cfg_device_addr_mask
  );

  assign data_rdata_intg = '0;
  assign instr_rdata_intg = '0;

  // Use default top parameters to ease bottom up synthesis
  // Enable passtrhough test clock gating
  cv32e40s_core u_top (
      .clk_i                  (clk_sys),
      .rst_ni                 (rst_sys_n),

      .dm_halt_addr_i         (32'h1A110800),
      .dm_exception_addr_i    (32'h0),
      .mhartid_i              (32'b0),

      // First instruction executed is at 0x0 + 0x80
      .boot_addr_i            (32'h00100080),
      .mtvec_addr_i           (32'h00100000),   

      .instr_req_o            (instr_req),
      .instr_gnt_i            (instr_gnt),
      .instr_rvalid_i         (instr_rvalid),
      .instr_addr_o           (instr_addr),
      .instr_rdata_i          (instr_rdata),
      // Tie off
      .instr_err_i            (1'b0),
      // Secure
      .instr_gntpar_i         (~instr_gnt),
      .instr_rvalidpar_i      (~instr_rvalid),

      .data_req_o             (host_req[CoreD]),
      .data_gnt_i             (host_gnt[CoreD]),
      .data_rvalid_i          (host_rvalid[CoreD]),
      .data_we_o              (host_we[CoreD]),
      .data_be_o              (host_be[CoreD]),
      .data_addr_o            (host_addr[CoreD]),
      .data_wdata_o           (host_wdata[CoreD]),
      .data_rdata_i           (host_rdata[CoreD]),
      // Secure
      .data_gntpar_i          (~host_gnt[CoreD]),
      .data_rvalidpar_i       (~host_rvalid[CoreD]),

      .irq_i                  (32'b0),
      .debug_req_i            (1'b0),
      .fetch_enable_i         (1'b1),
      .scan_cg_en_i           (1'b1),
    );

  // SRAM block for instruction and data storage
  ram_2p #(
      .Depth(1024*1024/4),
      .MemInitFile(SRAMInitFile)
    ) u_ram (
      .clk_i       (clk_sys),
      .rst_ni      (rst_sys_n),

      .a_req_i     (device_req[Ram]),
      .a_we_i      (device_we[Ram]),
      .a_be_i      (device_be[Ram]),
      .a_addr_i    (device_addr[Ram]),
      .a_wdata_i   (device_wdata[Ram]),
      .a_rvalid_o  (device_rvalid[Ram]),
      .a_rdata_o   (device_rdata[Ram]),

      .b_req_i     (instr_req),
      .b_we_i      (1'b0),
      .b_be_i      (4'b0),
      .b_addr_i    (instr_addr),
      .b_wdata_i   (32'b0),
      .b_rvalid_o  (instr_rvalid),
      .b_rdata_o   (instr_rdata)
    );
endmodule
