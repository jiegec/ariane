// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Description: Xilinx FPGA top-level
// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>

module ariane_xilinx (
  input  logic         cpu_resetn  ,

  output logic [14:0]  DDR_addr   ,
  output logic [ 2:0]  DDR_ba     ,
  output logic         DDR_cas_n  ,
  output logic         DDR_ck_n   ,
  output logic         DDR_ck_p   ,
  output logic         DDR_cke    ,
  output logic         DDR_cs_n   ,
  output logic [ 3:0]  DDR_dm     ,
  inout  logic [31:0]  DDR_dq     ,
  inout  logic [ 3:0]  DDR_dqs_n  ,
  inout  logic [ 3:0]  DDR_dqs_p  ,
  output logic         DDR_odt    ,
  output logic         DDR_ras_n  ,
  output logic         DDR_reset_n,
  output logic         DDR_we_n   ,

  output logic         FIXED_IO_ddr_vrn,
  output logic         FIXED_IO_ddr_vrp,
  output logic [53:0]  FIXED_IO_mio,
  output logic         FIXED_IO_ps_clk,
  output logic         FIXED_IO_ps_porb,
  output logic         FIXED_IO_ps_srstb,


  output wire          eth_rst_n   ,
  input  wire          eth_rxck    ,
  input  wire          eth_rxctl   ,
  input  wire [3:0]    eth_rxd     ,
  output wire          eth_txck    ,
  output wire          eth_txctl   ,
  output wire [3:0]    eth_txd     ,
  inout  wire          eth_mdio    ,
  output logic         eth_mdc     ,
  output logic [ 7:0]  led         ,
  input  logic [ 7:0]  sw          ,
  output logic         fan_pwm     ,

  // SPI
  output logic        spi_mosi    ,
  input  logic        spi_miso    ,
  output logic        spi_ss      ,
  output logic        spi_clk_o   ,

  // common part
  input  logic        tck         ,
  input  logic        tms         ,
  input  logic        trst_n      ,
  input  logic        tdi         ,
  output logic        tdo         ,
  input  logic        rx          ,
  output logic        tx
);

// 24 MByte in 8 byte words
localparam NumWords = (24 * 1024 * 1024) / 8;
localparam NBSlave = 2; // debug, ariane
localparam logic [63:0] CacheStartAddr = 64'h8000_0000;
localparam AxiAddrWidth = 64;
localparam AxiDataWidth = 64;
localparam AxiIdWidthMaster = 4;
localparam AxiIdWidthSlaves = AxiIdWidthMaster + $clog2(NBSlave); // 5
localparam AxiUserWidth = 1;

AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthMaster ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) slave[NBSlave-1:0]();

AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) master[ariane_soc::NB_PERIPHERALS-1:0]();

// disable test-enable
logic test_en;
logic ndmreset;
logic ndmreset_n;
logic debug_req_irq;
logic time_irq;
logic ipi;

logic clk; // 50MHz
logic eth_clk; // 125MHz, 90 degree phase shift
logic phy_tx_clk; // 125MHz
logic sd_clk_sys; // 50MHz

logic ddr_sync_reset;
logic ddr_clock_out; // 200MHz

logic rst_n, rst;
logic rtc;

// we need to switch reset polarity
logic cpu_reset;
assign cpu_reset  = ~cpu_resetn;

logic pll_locked;

// ROM
logic                    rom_req;
logic [AxiAddrWidth-1:0] rom_addr;
logic [AxiDataWidth-1:0] rom_rdata;

// Debug
logic          debug_req_valid;
logic          debug_req_ready;
dm::dmi_req_t  debug_req;
logic          debug_resp_valid;
logic          debug_resp_ready;
dm::dmi_resp_t debug_resp;

logic dmactive;

// IRQ
logic [1:0] irq;
assign test_en    = 1'b0;

logic [NBSlave-1:0] pc_asserted;

rstgen i_rstgen_main (
    .clk_i        ( clk                      ),
    .rst_ni       ( pll_locked & (~ndmreset) ),
    .test_mode_i  ( test_en                  ),
    .rst_no       ( ndmreset_n               ),
    .init_no      (                          ) // keep open
);

assign rst_n = ~ddr_sync_reset;
assign rst = ddr_sync_reset;

// ---------------
// AXI Xbar
// ---------------
axi_node_wrap_with_slices #(
    // three ports from Ariane (instruction, data and bypass)
    .NB_SLAVE           ( NBSlave                    ),
    .NB_MASTER          ( ariane_soc::NB_PERIPHERALS ),
    .NB_REGION          ( ariane_soc::NrRegion       ),
    .AXI_ADDR_WIDTH     ( AxiAddrWidth               ),
    .AXI_DATA_WIDTH     ( AxiDataWidth               ),
    .AXI_USER_WIDTH     ( AxiUserWidth               ),
    .AXI_ID_WIDTH       ( AxiIdWidthMaster           ),
    .MASTER_SLICE_DEPTH ( 2                          ),
    .SLAVE_SLICE_DEPTH  ( 2                          )
) i_axi_xbar (
    .clk          ( clk        ),
    .rst_n        ( ndmreset_n ),
    .test_en_i    ( test_en    ),
    .slave        ( slave      ),
    .master       ( master     ),
    .start_addr_i ({
        ariane_soc::DebugBase,
        ariane_soc::ROMBase,
        ariane_soc::CLINTBase,
        ariane_soc::PLICBase,
        ariane_soc::UARTBase,
        ariane_soc::SPIBase,
        ariane_soc::EthernetBase,
        ariane_soc::GPIOBase,
        ariane_soc::DRAMBase
    }),
    .end_addr_i   ({
        ariane_soc::DebugBase    + ariane_soc::DebugLength - 1,
        ariane_soc::ROMBase      + ariane_soc::ROMLength - 1,
        ariane_soc::CLINTBase    + ariane_soc::CLINTLength - 1,
        ariane_soc::PLICBase     + ariane_soc::PLICLength - 1,
        ariane_soc::UARTBase     + ariane_soc::UARTLength - 1,
        ariane_soc::SPIBase      + ariane_soc::SPILength - 1,
        ariane_soc::EthernetBase + ariane_soc::EthernetLength -1,
        ariane_soc::GPIOBase     + ariane_soc::GPIOLength - 1,
        ariane_soc::DRAMBase     + ariane_soc::DRAMLength - 1
    }),
    .valid_rule_i (ariane_soc::ValidRule)
);

// ---------------
// Debug Module
// ---------------
dmi_jtag i_dmi_jtag (
    .clk_i                ( clk                  ),
    .rst_ni               ( rst_n                ),
    .dmi_rst_no           (                      ), // keep open
    .testmode_i           ( test_en              ),
    .dmi_req_valid_o      ( debug_req_valid      ),
    .dmi_req_ready_i      ( debug_req_ready      ),
    .dmi_req_o            ( debug_req            ),
    .dmi_resp_valid_i     ( debug_resp_valid     ),
    .dmi_resp_ready_o     ( debug_resp_ready     ),
    .dmi_resp_i           ( debug_resp           ),
    .tck_i                ( tck    ),
    .tms_i                ( tms    ),
    .trst_ni              ( trst_n ),
    .td_i                 ( tdi    ),
    .td_o                 ( tdo    ),
    .tdo_oe_o             (        )
);

ariane_axi::req_t    dm_axi_m_req;
ariane_axi::resp_t   dm_axi_m_resp;

logic                dm_slave_req;
logic                dm_slave_we;
logic [64-1:0]       dm_slave_addr;
logic [64/8-1:0]     dm_slave_be;
logic [64-1:0]       dm_slave_wdata;
logic [64-1:0]       dm_slave_rdata;

logic                dm_master_req;
logic [64-1:0]       dm_master_add;
logic                dm_master_we;
logic [64-1:0]       dm_master_wdata;
logic [64/8-1:0]     dm_master_be;
logic                dm_master_gnt;
logic                dm_master_r_valid;
logic [64-1:0]       dm_master_r_rdata;

// debug module
dm_top #(
    .NrHarts          ( 1                 ),
    .BusWidth         ( AxiDataWidth      ),
    .Selectable_Harts ( 1'b1              )
) i_dm_top (
    .clk_i            ( clk               ),
    .rst_ni           ( rst_n             ), // PoR
    .testmode_i       ( test_en           ),
    .ndmreset_o       ( ndmreset          ),
    .dmactive_o       ( dmactive          ), // active debug session
    .debug_req_o      ( debug_req_irq     ),
    .unavailable_i    ( '0                ),
    .slave_req_i      ( dm_slave_req      ),
    .slave_we_i       ( dm_slave_we       ),
    .slave_addr_i     ( dm_slave_addr     ),
    .slave_be_i       ( dm_slave_be       ),
    .slave_wdata_i    ( dm_slave_wdata    ),
    .slave_rdata_o    ( dm_slave_rdata    ),
    .master_req_o     ( dm_master_req     ),
    .master_add_o     ( dm_master_add     ),
    .master_we_o      ( dm_master_we      ),
    .master_wdata_o   ( dm_master_wdata   ),
    .master_be_o      ( dm_master_be      ),
    .master_gnt_i     ( dm_master_gnt     ),
    .master_r_valid_i ( dm_master_r_valid ),
    .master_r_rdata_i ( dm_master_r_rdata ),
    .dmi_rst_ni       ( rst_n             ),
    .dmi_req_valid_i  ( debug_req_valid   ),
    .dmi_req_ready_o  ( debug_req_ready   ),
    .dmi_req_i        ( debug_req         ),
    .dmi_resp_valid_o ( debug_resp_valid  ),
    .dmi_resp_ready_i ( debug_resp_ready  ),
    .dmi_resp_o       ( debug_resp        )
);

axi2mem #(
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves    ),
    .AXI_ADDR_WIDTH ( AxiAddrWidth        ),
    .AXI_DATA_WIDTH ( AxiDataWidth        ),
    .AXI_USER_WIDTH ( AxiUserWidth        )
) i_dm_axi2mem (
    .clk_i      ( clk                       ),
    .rst_ni     ( rst_n                     ),
    .slave      ( master[ariane_soc::Debug] ),
    .req_o      ( dm_slave_req              ),
    .we_o       ( dm_slave_we               ),
    .addr_o     ( dm_slave_addr             ),
    .be_o       ( dm_slave_be               ),
    .data_o     ( dm_slave_wdata            ),
    .data_i     ( dm_slave_rdata            )
);

axi_master_connect i_dm_axi_master_connect (
  .axi_req_i(dm_axi_m_req),
  .axi_resp_o(dm_axi_m_resp),
  .master(slave[1])
);

axi_adapter #(
    .DATA_WIDTH            ( AxiDataWidth              )
) i_dm_axi_master (
    .clk_i                 ( clk                       ),
    .rst_ni                ( rst_n                     ),
    .req_i                 ( dm_master_req             ),
    .type_i                ( ariane_axi::SINGLE_REQ    ),
    .gnt_o                 ( dm_master_gnt             ),
    .gnt_id_o              (                           ),
    .addr_i                ( dm_master_add             ),
    .we_i                  ( dm_master_we              ),
    .wdata_i               ( dm_master_wdata           ),
    .be_i                  ( dm_master_be              ),
    .size_i                ( 2'b11                     ), // always do 64bit here and use byte enables to gate
    .id_i                  ( '0                        ),
    .valid_o               ( dm_master_r_valid         ),
    .rdata_o               ( dm_master_r_rdata         ),
    .id_o                  (                           ),
    .critical_word_o       (                           ),
    .critical_word_valid_o (                           ),
    .axi_req_o             ( dm_axi_m_req              ),
    .axi_resp_i            ( dm_axi_m_resp             )
);

// ---------------
// Core
// ---------------
ariane_axi::req_t    axi_ariane_req;
ariane_axi::resp_t   axi_ariane_resp;

ariane #(
    .CachedAddrBeg ( CacheStartAddr   )
) i_ariane (
    .clk_i        ( clk                 ),
    .rst_ni       ( ndmreset_n          ),
    .boot_addr_i  ( ariane_soc::ROMBase ), // start fetching from ROM
    .hart_id_i    ( '0                  ),
    .irq_i        ( irq                 ),
    .ipi_i        ( ipi                 ),
    .time_irq_i   ( timer_irq           ),
    .debug_req_i  ( debug_req_irq       ),
    .axi_req_o    ( axi_ariane_req      ),
    .axi_resp_i   ( axi_ariane_resp     )
);

axi_master_connect i_axi_master_connect_ariane (.axi_req_i(axi_ariane_req), .axi_resp_o(axi_ariane_resp), .master(slave[0]));

// ---------------
// CLINT
// ---------------
// divide clock by two
always_ff @(posedge clk or negedge ndmreset_n) begin
  if (~ndmreset_n) begin
    rtc <= 0;
  end else begin
    rtc <= rtc ^ 1'b1;
  end
end

ariane_axi::req_t    axi_clint_req;
ariane_axi::resp_t   axi_clint_resp;

clint #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .NR_CORES       ( 1                )
) i_clint (
    .clk_i       ( clk            ),
    .rst_ni      ( ndmreset_n     ),
    .testmode_i  ( test_en        ),
    .axi_req_i   ( axi_clint_req  ),
    .axi_resp_o  ( axi_clint_resp ),
    .rtc_i       ( rtc            ),
    .timer_irq_o ( timer_irq      ),
    .ipi_o       ( ipi            )
);

axi_slave_connect i_axi_slave_connect_clint (.axi_req_o(axi_clint_req), .axi_resp_i(axi_clint_resp), .slave(master[ariane_soc::CLINT]));

// ---------------
// ROM
// ---------------
axi2mem #(
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) i_axi2rom (
    .clk_i  ( clk                     ),
    .rst_ni ( ndmreset_n              ),
    .slave  ( master[ariane_soc::ROM] ),
    .req_o  ( rom_req                 ),
    .we_o   (                         ),
    .addr_o ( rom_addr                ),
    .be_o   (                         ),
    .data_o (                         ),
    .data_i ( rom_rdata               )
);

bootrom i_bootrom (
    .clk_i   ( clk       ),
    .req_i   ( rom_req   ),
    .addr_i  ( rom_addr  ),
    .rdata_o ( rom_rdata )
);

// ---------------
// Peripherals
// ---------------
ariane_peripherals #(
    .AxiAddrWidth ( AxiAddrWidth     ),
    .AxiDataWidth ( AxiDataWidth     ),
    .AxiIdWidth   ( AxiIdWidthSlaves ),
    .AxiUserWidth ( AxiUserWidth     ),
    .InclUART     ( 1'b1             ),
    .InclGPIO     ( 1'b1             ),
    .InclSPI      ( 1'b1         ),
    .InclEthernet ( 1'b1         )
) i_ariane_peripherals (
    .clk_i        ( clk                          ),
    .clk_200MHz_i ( ddr_clock_out                ),
    .rst_ni       ( ndmreset_n                   ),
    .plic         ( master[ariane_soc::PLIC]     ),
    .uart         ( master[ariane_soc::UART]     ),
    .spi          ( master[ariane_soc::SPI]      ),
    .gpio         ( master[ariane_soc::GPIO]     ),
    .eth_clk_i    ( eth_clk                      ),
    .ethernet     ( master[ariane_soc::Ethernet] ),
    .irq_o        ( irq                          ),
    .rx_i         ( rx                           ),
    .tx_o         ( tx                           ),
    .eth_txck,
    .eth_rxck,
    .eth_rxctl,
    .eth_rxd,
    .eth_rst_n,
    .eth_txctl,
    .eth_txd,
    .eth_mdio,
    .eth_mdc,
    .phy_tx_clk_i   ( phy_tx_clk                  ),
    .sd_clk_i       ( sd_clk_sys                  ),
    .spi_clk_o      ( spi_clk_o                   ),
    .spi_mosi       ( spi_mosi                    ),
    .spi_miso       ( spi_miso                    ),
    .spi_ss         ( spi_ss                      ),
    .leds_o         ( led                         ),
    .dip_switches_i ( sw                          )
);


// ---------------------
// Board peripherals
// ---------------------
// ---------------
// DDR
// ---------------
logic [AxiIdWidthSlaves-1:0] s_axi_awid;
logic [AxiAddrWidth-1:0]     s_axi_awaddr;
logic [7:0]                  s_axi_awlen;
logic [2:0]                  s_axi_awsize;
logic [1:0]                  s_axi_awburst;
logic [0:0]                  s_axi_awlock;
logic [3:0]                  s_axi_awcache;
logic [2:0]                  s_axi_awprot;
logic [3:0]                  s_axi_awregion;
logic [3:0]                  s_axi_awqos;
logic                        s_axi_awvalid;
logic                        s_axi_awready;
logic [AxiDataWidth-1:0]     s_axi_wdata;
logic [AxiDataWidth/8-1:0]   s_axi_wstrb;
logic                        s_axi_wlast;
logic                        s_axi_wvalid;
logic                        s_axi_wready;
logic [AxiIdWidthSlaves-1:0] s_axi_bid;
logic [1:0]                  s_axi_bresp;
logic                        s_axi_bvalid;
logic                        s_axi_bready;
logic [AxiIdWidthSlaves-1:0] s_axi_arid;
logic [AxiAddrWidth-1:0]     s_axi_araddr;
logic [7:0]                  s_axi_arlen;
logic [2:0]                  s_axi_arsize;
logic [1:0]                  s_axi_arburst;
logic [0:0]                  s_axi_arlock;
logic [3:0]                  s_axi_arcache;
logic [2:0]                  s_axi_arprot;
logic [3:0]                  s_axi_arregion;
logic [3:0]                  s_axi_arqos;
logic                        s_axi_arvalid;
logic                        s_axi_arready;
logic [AxiIdWidthSlaves-1:0] s_axi_rid;
logic [AxiDataWidth-1:0]     s_axi_rdata;
logic [1:0]                  s_axi_rresp;
logic                        s_axi_rlast;
logic                        s_axi_rvalid;
logic                        s_axi_rready;

logic [6-1:0] inter_axi_awid;
logic [32-1:0]     inter_axi_awaddr;
logic [7:0]                  inter_axi_awlen;
logic [2:0]                  inter_axi_awsize;
logic [1:0]                  inter_axi_awburst;
logic [0:0]                  inter_axi_awlock;
logic [3:0]                  inter_axi_awcache;
logic [2:0]                  inter_axi_awprot;
logic [3:0]                  inter_axi_awregion;
logic [3:0]                  inter_axi_awqos;
logic                        inter_axi_awvalid;
logic                        inter_axi_awready;
logic [64-1:0]     inter_axi_wdata;
logic [64/8-1:0]   inter_axi_wstrb;
logic                        inter_axi_wlast;
logic                        inter_axi_wvalid;
logic                        inter_axi_wready;
logic [6-1:0] inter_axi_bid;
logic [1:0]                  inter_axi_bresp;
logic                        inter_axi_bvalid;
logic                        inter_axi_bready;
logic [6-1:0] inter_axi_arid;
logic [32-1:0]     inter_axi_araddr;
logic [7:0]                  inter_axi_arlen;
logic [2:0]                  inter_axi_arsize;
logic [1:0]                  inter_axi_arburst;
logic [0:0]                  inter_axi_arlock;
logic [3:0]                  inter_axi_arcache;
logic [2:0]                  inter_axi_arprot;
logic [3:0]                  inter_axi_arregion;
logic [3:0]                  inter_axi_arqos;
logic                        inter_axi_arvalid;
logic                        inter_axi_arready;
logic [6-1:0] inter_axi_rid;
logic [64-1:0]     inter_axi_rdata;
logic [1:0]                  inter_axi_rresp;
logic                        inter_axi_rlast;
logic                        inter_axi_rvalid;
logic                        inter_axi_rready;


AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     )
) dram();

axi_riscv_atomics_wrap #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
    .AXI_DATA_WIDTH ( AxiDataWidth     ),
    .AXI_ID_WIDTH   ( AxiIdWidthSlaves ),
    .AXI_USER_WIDTH ( AxiUserWidth     ),
    .AXI_MAX_WRITE_TXNS ( 1  ),
    .RISCV_WORD_WIDTH   ( 64 )
) i_axi_riscv_atomics (
    .clk_i  ( clk                      ),
    .rst_ni ( ndmreset_n               ),
    .slv    ( master[ariane_soc::DRAM] ),
    .mst    ( dram                     )
);

`ifdef PROTOCOL_CHECKER
logic pc_status;
// assign led[0] = pc_status;
// assign led[7:1] = '0;

xlnx_protocol_checker i_xlnx_protocol_checker (
  .pc_status(),
  .pc_asserted(pc_status),
  .aclk(clk),
  .aresetn(ndmreset_n),
  .pc_axi_awid     (dram.aw_id),
  .pc_axi_awaddr   (dram.aw_addr),
  .pc_axi_awlen    (dram.aw_len),
  .pc_axi_awsize   (dram.aw_size),
  .pc_axi_awburst  (dram.aw_burst),
  .pc_axi_awlock   (dram.aw_lock),
  .pc_axi_awcache  (dram.aw_cache),
  .pc_axi_awprot   (dram.aw_prot),
  .pc_axi_awqos    (dram.aw_qos),
  .pc_axi_awregion (dram.aw_region),
  .pc_axi_awuser   (dram.aw_user),
  .pc_axi_awvalid  (dram.aw_valid),
  .pc_axi_awready  (dram.aw_ready),
  .pc_axi_wlast    (dram.w_last),
  .pc_axi_wdata    (dram.w_data),
  .pc_axi_wstrb    (dram.w_strb),
  .pc_axi_wuser    (dram.w_user),
  .pc_axi_wvalid   (dram.w_valid),
  .pc_axi_wready   (dram.w_ready),
  .pc_axi_bid      (dram.b_id),
  .pc_axi_bresp    (dram.b_resp),
  .pc_axi_buser    (dram.b_user),
  .pc_axi_bvalid   (dram.b_valid),
  .pc_axi_bready   (dram.b_ready),
  .pc_axi_arid     (dram.ar_id),
  .pc_axi_araddr   (dram.ar_addr),
  .pc_axi_arlen    (dram.ar_len),
  .pc_axi_arsize   (dram.ar_size),
  .pc_axi_arburst  (dram.ar_burst),
  .pc_axi_arlock   (dram.ar_lock),
  .pc_axi_arcache  (dram.ar_cache),
  .pc_axi_arprot   (dram.ar_prot),
  .pc_axi_arqos    (dram.ar_qos),
  .pc_axi_arregion (dram.ar_region),
  .pc_axi_aruser   (dram.ar_user),
  .pc_axi_arvalid  (dram.ar_valid),
  .pc_axi_arready  (dram.ar_ready),
  .pc_axi_rid      (dram.r_id),
  .pc_axi_rlast    (dram.r_last),
  .pc_axi_rdata    (dram.r_data),
  .pc_axi_rresp    (dram.r_resp),
  .pc_axi_ruser    (dram.r_user),
  .pc_axi_rvalid   (dram.r_valid),
  .pc_axi_rready   (dram.r_ready)
);
`endif

assign dram.r_user = '0;
assign dram.b_user = '0;

xlnx_axi_clock_converter i_xlnx_axi_clock_converter_ddr (
  .s_axi_aclk     ( clk              ),
  .s_axi_aresetn  ( ndmreset_n       ),
  .s_axi_awid     ( dram.aw_id       ),
  .s_axi_awaddr   ( dram.aw_addr     ),
  .s_axi_awlen    ( dram.aw_len      ),
  .s_axi_awsize   ( dram.aw_size     ),
  .s_axi_awburst  ( dram.aw_burst    ),
  .s_axi_awlock   ( dram.aw_lock     ),
  .s_axi_awcache  ( dram.aw_cache    ),
  .s_axi_awprot   ( dram.aw_prot     ),
  .s_axi_awregion ( dram.aw_region   ),
  .s_axi_awqos    ( dram.aw_qos      ),
  .s_axi_awvalid  ( dram.aw_valid    ),
  .s_axi_awready  ( dram.aw_ready    ),
  .s_axi_wdata    ( dram.w_data      ),
  .s_axi_wstrb    ( dram.w_strb      ),
  .s_axi_wlast    ( dram.w_last      ),
  .s_axi_wvalid   ( dram.w_valid     ),
  .s_axi_wready   ( dram.w_ready     ),
  .s_axi_bid      ( dram.b_id        ),
  .s_axi_bresp    ( dram.b_resp      ),
  .s_axi_bvalid   ( dram.b_valid     ),
  .s_axi_bready   ( dram.b_ready     ),
  .s_axi_arid     ( dram.ar_id       ),
  .s_axi_araddr   ( dram.ar_addr     ),
  .s_axi_arlen    ( dram.ar_len      ),
  .s_axi_arsize   ( dram.ar_size     ),
  .s_axi_arburst  ( dram.ar_burst    ),
  .s_axi_arlock   ( dram.ar_lock     ),
  .s_axi_arcache  ( dram.ar_cache    ),
  .s_axi_arprot   ( dram.ar_prot     ),
  .s_axi_arregion ( dram.ar_region   ),
  .s_axi_arqos    ( dram.ar_qos      ),
  .s_axi_arvalid  ( dram.ar_valid    ),
  .s_axi_arready  ( dram.ar_ready    ),
  .s_axi_rid      ( dram.r_id        ),
  .s_axi_rdata    ( dram.r_data      ),
  .s_axi_rresp    ( dram.r_resp      ),
  .s_axi_rlast    ( dram.r_last      ),
  .s_axi_rvalid   ( dram.r_valid     ),
  .s_axi_rready   ( dram.r_ready     ),
  // to size converter
  .m_axi_aclk     ( ddr_clock_out    ),
  .m_axi_aresetn  ( ndmreset_n       ),
  .m_axi_awid     ( s_axi_awid       ),
  .m_axi_awaddr   ( s_axi_awaddr     ),
  .m_axi_awlen    ( s_axi_awlen      ),
  .m_axi_awsize   ( s_axi_awsize     ),
  .m_axi_awburst  ( s_axi_awburst    ),
  .m_axi_awlock   ( s_axi_awlock     ),
  .m_axi_awcache  ( s_axi_awcache    ),
  .m_axi_awprot   ( s_axi_awprot     ),
  .m_axi_awregion ( s_axi_awregion   ),
  .m_axi_awqos    ( s_axi_awqos      ),
  .m_axi_awvalid  ( s_axi_awvalid    ),
  .m_axi_awready  ( s_axi_awready    ),
  .m_axi_wdata    ( s_axi_wdata      ),
  .m_axi_wstrb    ( s_axi_wstrb      ),
  .m_axi_wlast    ( s_axi_wlast      ),
  .m_axi_wvalid   ( s_axi_wvalid     ),
  .m_axi_wready   ( s_axi_wready     ),
  .m_axi_bid      ( s_axi_bid        ),
  .m_axi_bresp    ( s_axi_bresp      ),
  .m_axi_bvalid   ( s_axi_bvalid     ),
  .m_axi_bready   ( s_axi_bready     ),
  .m_axi_arid     ( s_axi_arid       ),
  .m_axi_araddr   ( s_axi_araddr     ),
  .m_axi_arlen    ( s_axi_arlen      ),
  .m_axi_arsize   ( s_axi_arsize     ),
  .m_axi_arburst  ( s_axi_arburst    ),
  .m_axi_arlock   ( s_axi_arlock     ),
  .m_axi_arcache  ( s_axi_arcache    ),
  .m_axi_arprot   ( s_axi_arprot     ),
  .m_axi_arregion ( s_axi_arregion   ),
  .m_axi_arqos    ( s_axi_arqos      ),
  .m_axi_arvalid  ( s_axi_arvalid    ),
  .m_axi_arready  ( s_axi_arready    ),
  .m_axi_rid      ( s_axi_rid        ),
  .m_axi_rdata    ( s_axi_rdata      ),
  .m_axi_rresp    ( s_axi_rresp      ),
  .m_axi_rlast    ( s_axi_rlast      ),
  .m_axi_rvalid   ( s_axi_rvalid     ),
  .m_axi_rready   ( s_axi_rready     )
);

xlnx_clk_gen i_xlnx_clk_gen (
  .clk_out1 ( clk           ), // 50 MHz
  .clk_out2 ( phy_tx_clk    ), // 125 MHz (for RGMII PHY)
  .clk_out3 ( eth_clk       ), // 125 MHz quadrature (90 deg phase shift)
  .clk_out4 ( sd_clk_sys    ), // 50 MHz clock
  .reset    ( cpu_reset     ),
  .locked   ( pll_locked    ),
  .clk_in1  ( ddr_clock_out ) // 200MHz
);

fan_ctrl i_fan_ctrl (
    .clk_i         ( clk        ),
    .rst_ni        ( ndmreset_n ),
    .pwm_setting_i ( '1         ),
    .fan_pwm_o     ( fan_pwm    )
);

xlnx_axi_interconnect smc (
    .INTERCONNECT_ACLK(ddr_clock_out),
    .INTERCONNECT_ARESETN(ddr_sync_reset),
    .S00_AXI_ACLK(ddr_clock_out),
    .S00_AXI_ARESET_OUT_N(ddr_sync_reset),
    .M00_AXI_ACLK(ddr_clock_out),
    .M00_AXI_ARESET_OUT_N(ddr_sync_reset),

    .S00_AXI_AWID(s_axi_awid),
    .S00_AXI_AWADDR(s_axi_awaddr),
    .S00_AXI_AWLEN(s_axi_awlen),
    .S00_AXI_AWSIZE(s_axi_awsize),
    .S00_AXI_AWBURST(s_axi_awburst),
    .S00_AXI_AWLOCK(s_axi_awlock),
    .S00_AXI_AWCACHE(s_axi_awcache),
    .S00_AXI_AWPROT(s_axi_awprot),
    .S00_AXI_AWQOS(s_axi_awqos),
    .S00_AXI_AWVALID(s_axi_awvalid),
    .S00_AXI_AWREADY(s_axi_awready),
    .S00_AXI_WDATA(s_axi_wdata),
    .S00_AXI_WSTRB(s_axi_wstrb),
    .S00_AXI_WLAST(s_axi_wlast),
    .S00_AXI_WVALID(s_axi_wvalid),
    .S00_AXI_WREADY(s_axi_wready),
    .S00_AXI_BREADY(s_axi_bready),
    .S00_AXI_BID(s_axi_bid),
    .S00_AXI_BRESP(s_axi_bresp),
    .S00_AXI_BVALID(s_axi_bvalid),
    .S00_AXI_ARID(s_axi_arid),
    .S00_AXI_ARADDR(s_axi_araddr[29:0]),
    .S00_AXI_ARLEN(s_axi_arlen),
    .S00_AXI_ARSIZE(s_axi_arsize),
    .S00_AXI_ARBURST(s_axi_arburst),
    .S00_AXI_ARLOCK(s_axi_arlock),
    .S00_AXI_ARCACHE(s_axi_arcache),
    .S00_AXI_ARPROT(s_axi_arprot),
    .S00_AXI_ARQOS(s_axi_arqos),
    .S00_AXI_ARVALID(s_axi_arvalid),
    .S00_AXI_ARREADY(s_axi_arready),
    .S00_AXI_RREADY(s_axi_rready),
    .S00_AXI_RID(s_axi_rid),
    .S00_AXI_RDATA(s_axi_rdata),
    .S00_AXI_RRESP(s_axi_rresp),
    .S00_AXI_RLAST(s_axi_rlast),
    .S00_AXI_RVALID(s_axi_rvalid),

    .M00_AXI_AWID(inter_axi_awid),
    .M00_AXI_AWADDR(inter_axi_awaddr),
    .M00_AXI_AWLEN(inter_axi_awlen),
    .M00_AXI_AWSIZE(inter_axi_awsize),
    .M00_AXI_AWBURST(inter_axi_awburst),
    .M00_AXI_AWLOCK(inter_axi_awlock),
    .M00_AXI_AWCACHE(inter_axi_awcache),
    .M00_AXI_AWPROT(inter_axi_awprot),
    .M00_AXI_AWQOS(inter_axi_awqos),
    .M00_AXI_AWVALID(inter_axi_awvalid),
    .M00_AXI_AWREADY(inter_axi_awready),
    .M00_AXI_WDATA(inter_axi_wdata),
    .M00_AXI_WSTRB(inter_axi_wstrb),
    .M00_AXI_WLAST(inter_axi_wlast),
    .M00_AXI_WVALID(inter_axi_wvalid),
    .M00_AXI_WREADY(inter_axi_wready),
    .M00_AXI_BREADY(inter_axi_bready),
    .M00_AXI_BID(inter_axi_bid),
    .M00_AXI_BRESP(inter_axi_bresp),
    .M00_AXI_BVALID(inter_axi_bvalid),
    .M00_AXI_ARID(inter_axi_arid),
    .M00_AXI_ARADDR(inter_axi_araddr[29:0]),
    .M00_AXI_ARLEN(inter_axi_arlen),
    .M00_AXI_ARSIZE(inter_axi_arsize),
    .M00_AXI_ARBURST(inter_axi_arburst),
    .M00_AXI_ARLOCK(inter_axi_arlock),
    .M00_AXI_ARCACHE(inter_axi_arcache),
    .M00_AXI_ARPROT(inter_axi_arprot),
    .M00_AXI_ARQOS(inter_axi_arqos),
    .M00_AXI_ARVALID(inter_axi_arvalid),
    .M00_AXI_ARREADY(inter_axi_arready),
    .M00_AXI_RREADY(inter_axi_rready),
    .M00_AXI_RID(inter_axi_rid),
    .M00_AXI_RDATA(inter_axi_rdata),
    .M00_AXI_RRESP(inter_axi_rresp),
    .M00_AXI_RLAST(inter_axi_rlast),
    .M00_AXI_RVALID(inter_axi_rvalid)
);

xlnx_processing_system7 i_ps7 (
    .DDR_Addr(DDR_addr),
    .DDR_BankAddr(DDR_ba),
    .DDR_CAS_n(DDR_cas_n),
    .DDR_CKE(DDR_cke),
    .DDR_CS_n(DDR_cs_n),
    .DDR_Clk(DDR_ck_p),
    .DDR_Clk_n(DDR_ck_n),
    .DDR_DM(DDR_dm),
    .DDR_DQ(DDR_dq),
    .DDR_DQS(DDR_dqs_p),
    .DDR_DQS_n(DDR_dqs_n),
    .DDR_DRSTB(DDR_reset_n),
    .DDR_ODT(DDR_odt),
    .DDR_RAS_n(DDR_ras_n),
    .DDR_VRN(FIXED_IO_ddr_vrn),
    .DDR_VRP(FIXED_IO_ddr_vrp),
    .DDR_WEB(DDR_we_n),
    .MIO(FIXED_IO_mio),
    .PS_CLK(FIXED_IO_ps_clk),
    .PS_PORB(FIXED_IO_ps_porb),
    .PS_SRSTB(FIXED_IO_ps_srstb),
    .FCLK_CLK0(ddr_clock_out),
    .FCLK_RESET0_N(ddr_sync_reset),

    .S_AXI_HP0_AWID(inter_axi_awid),
    .S_AXI_HP0_AWADDR(inter_axi_awaddr),
    .S_AXI_HP0_AWLEN(inter_axi_awlen),
    .S_AXI_HP0_AWSIZE(inter_axi_awsize),
    .S_AXI_HP0_AWBURST(inter_axi_awburst),
    .S_AXI_HP0_AWLOCK(inter_axi_awlock),
    .S_AXI_HP0_AWCACHE(inter_axi_awcache),
    .S_AXI_HP0_AWPROT(inter_axi_awprot),
    .S_AXI_HP0_AWQOS(inter_axi_awqos),
    .S_AXI_HP0_AWVALID(inter_axi_awvalid),
    .S_AXI_HP0_AWREADY(inter_axi_awready),
    .S_AXI_HP0_WDATA(inter_axi_wdata),
    .S_AXI_HP0_WSTRB(inter_axi_wstrb),
    .S_AXI_HP0_WLAST(inter_axi_wlast),
    .S_AXI_HP0_WVALID(inter_axi_wvalid),
    .S_AXI_HP0_WREADY(inter_axi_wready),
    .S_AXI_HP0_BREADY(inter_axi_bready),
    .S_AXI_HP0_BID(inter_axi_bid),
    .S_AXI_HP0_BRESP(inter_axi_bresp),
    .S_AXI_HP0_BVALID(inter_axi_bvalid),
    .S_AXI_HP0_ARID(inter_axi_arid),
    .S_AXI_HP0_ARADDR(inter_axi_araddr[29:0]),
    .S_AXI_HP0_ARLEN(inter_axi_arlen),
    .S_AXI_HP0_ARSIZE(inter_axi_arsize),
    .S_AXI_HP0_ARBURST(inter_axi_arburst),
    .S_AXI_HP0_ARLOCK(inter_axi_arlock),
    .S_AXI_HP0_ARCACHE(inter_axi_arcache),
    .S_AXI_HP0_ARPROT(inter_axi_arprot),
    .S_AXI_HP0_ARQOS(inter_axi_arqos),
    .S_AXI_HP0_ARVALID(inter_axi_arvalid),
    .S_AXI_HP0_ARREADY(inter_axi_arready),
    .S_AXI_HP0_RREADY(inter_axi_rready),
    .S_AXI_HP0_RID(inter_axi_rid),
    .S_AXI_HP0_RDATA(inter_axi_rdata),
    .S_AXI_HP0_RRESP(inter_axi_rresp),
    .S_AXI_HP0_RLAST(inter_axi_rlast),
    .S_AXI_HP0_RVALID(inter_axi_rvalid)
);

// xlnx_mig_7_ddr3 i_ddr (
//     .sys_clk_p,
//     .sys_clk_n,
//     .ddr3_dq,
//     .ddr3_dqs_n,
//     .ddr3_dqs_p,
//     .ddr3_addr,
//     .ddr3_ba,
//     .ddr3_ras_n,
//     .ddr3_cas_n,
//     .ddr3_we_n,
//     .ddr3_reset_n,
//     .ddr3_ck_p,
//     .ddr3_ck_n,
//     .ddr3_cke,
//     .ddr3_cs_n,
//     .ddr3_dm,
//     .ddr3_odt,
//     .mmcm_locked     (                ), // keep open
//     .app_sr_req      ( '0             ),
//     .app_ref_req     ( '0             ),
//     .app_zq_req      ( '0             ),
//     .app_sr_active   (                ), // keep open
//     .app_ref_ack     (                ), // keep open
//     .app_zq_ack      (                ), // keep open
//     .ui_clk          ( ddr_clock_out  ),
//     .ui_clk_sync_rst ( ddr_sync_reset ),
//     .aresetn         ( ndmreset_n     ),
//     .s_axi_awid,
//     .s_axi_awaddr    ( s_axi_awaddr[29:0] ),
//     .s_axi_awlen,
//     .s_axi_awsize,
//     .s_axi_awburst,
//     .s_axi_awlock,
//     .s_axi_awcache,
//     .s_axi_awprot,
//     .s_axi_awqos,
//     .s_axi_awvalid,
//     .s_axi_awready,
//     .s_axi_wdata,
//     .s_axi_wstrb,
//     .s_axi_wlast,
//     .s_axi_wvalid,
//     .s_axi_wready,
//     .s_axi_bready,
//     .s_axi_bid,
//     .s_axi_bresp,
//     .s_axi_bvalid,
//     .s_axi_arid,
//     .s_axi_araddr     ( s_axi_araddr[29:0] ),
//     .s_axi_arlen,
//     .s_axi_arsize,
//     .s_axi_arburst,
//     .s_axi_arlock,
//     .s_axi_arcache,
//     .s_axi_arprot,
//     .s_axi_arqos,
//     .s_axi_arvalid,
//     .s_axi_arready,
//     .s_axi_rready,
//     .s_axi_rid,
//     .s_axi_rdata,
//     .s_axi_rresp,
//     .s_axi_rlast,
//     .s_axi_rvalid,
//     .init_calib_complete (            ), // keep open
//     .device_temp         (            ), // keep open
//     .sys_rst             ( cpu_resetn )
// );

endmodule
