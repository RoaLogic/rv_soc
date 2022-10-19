/////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.          //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.    //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'    //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.    //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'    //
//                                             `---'               //
//    RISC-V                                                       //
//    Lattice ECP5 Versa Development Kit Testbench Top Level       //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//             Copyright (C) 2021 ROA Logic BV                     //
//             www.roalogic.com                                    //
//                                                                 //
//     Unless specifically agreed in writing, this software is     //
//   licensed under the RoaLogic Non-Commercial License            //
//   version-1.0 (the "License"), a copy of which is included      //
//   with this file or may be found on the RoaLogic website        //
//   http://www.roalogic.com. You may not use the file except      //
//   in compliance with the License.                               //
//                                                                 //
//     THIS SOFTWARE IS PROVIDED "AS IS" AND WITHOUT ANY           //
//   EXPRESS OF IMPLIED WARRANTIES OF ANY KIND.                    //
//   See the License for permissions and limitations under the     //
//   License.                                                      //
//                                                                 //
/////////////////////////////////////////////////////////////////////


import ahb3lite_pkg::*;

`define RV_CORE soc_top.cpu_subsys.riscv_top.core

module rv_soc_tb;
  parameter TECHNOLOGY         = "GENERIC";
  parameter BOOTROM_SIZE       = 4;  //BootROM size in kBytes
  parameter RAM_SIZE           = 4;  //on-chip RAM size in kBytes
  parameter BOOTROM_INIT_FILE  = ""; //in Verilog-Hex format
  parameter SDRAM_INIT_FILE    = ""; //in Intel-Hex format
  parameter JTAG_SERVER_TYPE   = "OPENOCD";  // Server type to connect external sim debugger

  /*
   * CPU options
   */
  parameter XLEN               = 32;
  parameter HAS_USER           = 0;
  parameter HAS_SUPER          = 0;
  parameter HAS_HYPER          = 0;
  parameter HAS_BPU            = 1;
  parameter HAS_FPU            = 0;
  parameter HAS_MMU            = 0;
  parameter HAS_RVM            = 1;
  parameter HAS_RVA            = 0;
  parameter HAS_RVC            = 0;
  parameter HAS_S              = 1;
  parameter HAS_H              = 0;
  parameter HAS_U              = 1;
  
  parameter MULT_LATENCY       = 1;

  parameter BP_GLOBAL_BITS     = 2;
  parameter BP_LOCAL_BITS      = 10;

  parameter ICACHE_SIZE        = 1;  //in kBytes
  parameter ICACHE_BLOCK_SIZE  = 32; //in Bytes
  parameter ICACHE_WAYS        = 2;  //'n'-way set associative
  parameter ICACHE_REPLACE_ALG = 0;

  parameter DCACHE_SIZE        = 1;  //in kBytes
  parameter DCACHE_BLOCK_SIZE  = 32; //in Bytes
  parameter DCACHE_WAYS        = 2;  //'n'-way set associative
  parameter DCACHE_REPLACE_ALG = 0;
  parameter CORES              = 1;


  parameter BAUD_RATE          = 460800; //2457600; //9600;



  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  import ahb3lite_pkg::*;
  
  localparam GPIO_CNT         = 10;
  localparam UART_CNT         = 1;
  localparam USR_SAHB_CNT     = 3;
  localparam USR_APB32_CNT    = 3;
  localparam USR_APB8_CNT     = 1;
  localparam USR_INT_CNT      = 1;
  
  
  localparam PLEN             = 32;
  localparam SDRAM_HADDR_SIZE = 31;
  localparam EXT_HADDR_SIZE   = 24;
  localparam USR_HADDR_SIZE   = 25 - $clog2(USR_SAHB_CNT ) -1;
  localparam USR32_PADDR_SIZE = 20 - $clog2(USR_APB32_CNT) -1;
  localparam USR8_PADDR_SIZE  = 20 - $clog2(USR_APB8_CNT ) -1;


  //SDRAM Model parameters
  localparam SDRAM_ADDR_SIZE = 11;
  localparam SDRAM_DATA_SIZE = XLEN;


  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic                         jtag_trstn;
  logic                         jtag_tck;
  logic                         jtag_tms;
  logic                         jtag_tdi;
  logic                         jtag_tdo;

  logic                         poweron_rst_n,
                                sysrst_n,
                                sdram_init_done;
  logic                         clk_ahb  , clk_apb;
  logic                         rst_ahb_n, rst_apb_n;
  logic                         rst_sys;

  
  //SDRAM AHB Bus
  logic                         sdram_HSEL;
  logic [SDRAM_HADDR_SIZE -1:0] sdram_HADDR;
  logic [XLEN             -1:0] sdram_HWDATA;
  logic [XLEN             -1:0] sdram_HRDATA;
  logic                         sdram_HWRITE;
  logic [HSIZE_SIZE       -1:0] sdram_HSIZE;
  logic [HBURST_SIZE      -1:0] sdram_HBURST;
  logic [HPROT_SIZE       -1:0] sdram_HPROT;
  logic [HTRANS_SIZE      -1:0] sdram_HTRANS;
  logic                         sdram_HMASTLOCK;
  logic                         sdram_HREADY;
  logic                         sdram_HREADYOUT;
  logic                         sdram_HRESP;

  //External AHB Bus
  logic                         ext_HSEL;
  logic [EXT_HADDR_SIZE   -1:0] ext_HADDR;
  logic [XLEN             -1:0] ext_HWDATA;
  logic [XLEN             -1:0] ext_HRDATA;
  logic                         ext_HWRITE;
  logic [HSIZE_SIZE       -1:0] ext_HSIZE;
  logic [HBURST_SIZE      -1:0] ext_HBURST;
  logic [HPROT_SIZE       -1:0] ext_HPROT;
  logic [HTRANS_SIZE      -1:0] ext_HTRANS;
  logic                         ext_HMASTLOCK;
  logic                         ext_HREADY;
  logic                         ext_HREADYOUT;
  logic                         ext_HRESP;
  
  //User AHB Bus
  logic                         usr_HSEL      [USR_SAHB_CNT];
  logic [USR_HADDR_SIZE   -1:0] usr_HADDR     [USR_SAHB_CNT];
  logic [XLEN             -1:0] usr_HWDATA    [USR_SAHB_CNT];
  logic [XLEN             -1:0] usr_HRDATA    [USR_SAHB_CNT];
  logic                         usr_HWRITE    [USR_SAHB_CNT];
  logic [HSIZE_SIZE       -1:0] usr_HSIZE     [USR_SAHB_CNT];
  logic [HBURST_SIZE      -1:0] usr_HBURST    [USR_SAHB_CNT];
  logic [HPROT_SIZE       -1:0] usr_HPROT     [USR_SAHB_CNT];
  logic [HTRANS_SIZE      -1:0] usr_HTRANS    [USR_SAHB_CNT];
  logic                         usr_HMASTLOCK [USR_SAHB_CNT];
  logic                         usr_HREADY    [USR_SAHB_CNT];
  logic                         usr_HREADYOUT [USR_SAHB_CNT];
  logic                         usr_HRESP     [USR_SAHB_CNT];
  
  // User 32b APB
  logic                         usr_PSEL32    [USR_APB32_CNT];
  logic                         usr_PENABLE32 [USR_APB32_CNT];
  logic  [                 2:0] usr_PPROT32   [USR_APB32_CNT];
  logic                         usr_PWRITE32  [USR_APB32_CNT];
  logic  [                 3:0] usr_PSTRB32   [USR_APB32_CNT];
  logic  [USR32_PADDR_SIZE-1:0] usr_PADDR32   [USR_APB32_CNT];
  logic  [                31:0] usr_PWDATA32  [USR_APB32_CNT];
  logic  [                31:0] usr_PRDATA32  [USR_APB32_CNT];
  logic                         usr_PREADY32  [USR_APB32_CNT];
  logic                         usr_PSLVERR32 [USR_APB32_CNT];
  
  // User 8b APB
  logic                         usr_PSEL8     [USR_APB8_CNT ];
  logic                         usr_PENABLE8  [USR_APB8_CNT ];
  logic  [                 2:0] usr_PPROT8    [USR_APB8_CNT ];
  logic                         usr_PWRITE8   [USR_APB8_CNT ];
  logic  [USR8_PADDR_SIZE -1:0] usr_PADDR8    [USR_APB8_CNT ];
  logic  [                 7:0] usr_PWDATA8   [USR_APB8_CNT ];
  logic  [                 7:0] usr_PRDATA8   [USR_APB8_CNT ];
  logic                         usr_PREADY8   [USR_APB8_CNT ];
  logic                         usr_PSLVERR8  [USR_APB8_CNT ];

  //GPIO
  logic [                  7:0] gpioi         [GPIO_CNT     ];
  logic [                  7:0] gpioo         [GPIO_CNT     ];
  logic [                  7:0] gpiooe        [GPIO_CNT     ];

  //UART
  logic                         uart_rxd      [UART_CNT     ];
  logic                         uart_txd      [UART_CNT     ];
  
  //User Interrupts
  logic [USR_INT_CNT      -1:0] usr_int;


  //DevBoard connectors
  logic [ 7:0] ledi    , ledo    , ledoe;
  logic [ 7:0] switchi , switcho , switchoe;
  logic [15:0] displayi, displayo, displayoe;
  logic [47:0] expconi , expcono , expconoe;



  /*
   * Instantiate SoC top level
   */
  rv_soc_top #(
    .TECHNOLOGY         ( TECHNOLOGY         ),
    .BOOTROM_SIZE       ( BOOTROM_SIZE       ),
    .RAM_SIZE           ( RAM_SIZE           ),
    .INIT_FILE          ( BOOTROM_INIT_FILE  ),
  
    .GPIO_CNT           ( GPIO_CNT           ),
    .UART_CNT           ( UART_CNT           ),
    .USR_SAHB_CNT       ( USR_SAHB_CNT       ),
    .USR_APB8_CNT       ( USR_APB8_CNT       ),
    .USR_APB32_CNT      ( USR_APB32_CNT      ),

    .USR_INT_CNT        ( USR_INT_CNT        ),

    .APB_CLK_FREQ       ( 12.5               ),
    .UART_BAUDRATE      ( BAUD_RATE          ),
    
    // CPU options
    .XLEN               ( XLEN               ),
    .PLEN               ( PLEN               ),
    .HAS_USER           ( HAS_USER           ),
    .HAS_SUPER          ( HAS_SUPER          ),
    .HAS_HYPER          ( HAS_HYPER          ),
    .HAS_BPU            ( HAS_BPU            ),
    .HAS_FPU            ( HAS_FPU            ),
    .HAS_MMU            ( HAS_MMU            ),
    .HAS_RVM            ( HAS_RVM            ),
    .HAS_RVA            ( HAS_RVA            ),
    .HAS_RVC            ( HAS_RVC            ),
	 
    .MULT_LATENCY       ( MULT_LATENCY       ),

    .BP_GLOBAL_BITS     ( BP_GLOBAL_BITS     ),
    .BP_LOCAL_BITS      ( BP_LOCAL_BITS      ),

    .ICACHE_SIZE        ( ICACHE_SIZE        ),
    .ICACHE_BLOCK_SIZE  ( ICACHE_BLOCK_SIZE  ),
    .ICACHE_WAYS        ( ICACHE_WAYS        ),
    .ICACHE_REPLACE_ALG ( ICACHE_REPLACE_ALG ),

    .DCACHE_SIZE        ( DCACHE_SIZE        ),
    .DCACHE_BLOCK_SIZE  ( DCACHE_BLOCK_SIZE  ),
    .DCACHE_WAYS        ( DCACHE_WAYS        ),
    .DCACHE_REPLACE_ALG ( DCACHE_REPLACE_ALG ) )
  soc_top (

    .poweron_rst_ni     ( poweron_rst_n      ),

    /*
     * /JTAG interface (only for RoaLogic TAP controller)
     */
    .jtag_trst_ni       ( jtag_trstn         ),
    .jtag_tck_i         ( jtag_tck           ),
    .jtag_tms_i         ( jtag_tms           ),
    .jtag_tdi_i         ( jtag_tdi           ),
    .jtag_tdo_o         ( jtag_tdo           ),
    .jtag_tdoe_o        (                    ),

    /*
     * Clock & Reset
     */
    .rst_ahb_ni         ( rst_ahb_n          ),
    .clk_ahb_i          ( clk_ahb            ),
    .rst_apb_ni         ( rst_apb_n          ),
    .clk_apb_i          ( clk_apb            ),

    .rst_sys_o          ( rst_sys            ),

    /*
     * SDRAM AHB
     */
    .sdram_HSEL         ( sdram_HSEL         ),
    .sdram_HADDR        ( sdram_HADDR        ),
    .sdram_HWDATA       ( sdram_HWDATA       ),
    .sdram_HRDATA       ( sdram_HRDATA       ),
    .sdram_HWRITE       ( sdram_HWRITE       ),
    .sdram_HSIZE        ( sdram_HSIZE        ),
    .sdram_HBURST       ( sdram_HBURST       ),
    .sdram_HPROT        ( sdram_HPROT        ),
    .sdram_HTRANS       ( sdram_HTRANS       ),
    .sdram_HMASTLOCK    ( sdram_HMASTLOCK    ),
    .sdram_HREADYOUT    ( sdram_HREADYOUT    ),
    .sdram_HREADY       ( sdram_HREADY       ),
    .sdram_HRESP        ( sdram_HRESP        ),

    /*
     * External (Flash) AHB
     */
    .ext_HSEL           ( ext_HSEL           ),
    .ext_HADDR          ( ext_HADDR          ),
    .ext_HWDATA         ( ext_HWDATA         ),
    .ext_HRDATA         ( ext_HRDATA         ),
    .ext_HWRITE         ( ext_HWRITE         ),
    .ext_HSIZE          ( ext_HSIZE          ),
    .ext_HBURST         ( ext_HBURST         ),
    .ext_HPROT          ( ext_HPROT          ),
    .ext_HTRANS         ( ext_HTRANS         ),
    .ext_HMASTLOCK      ( ext_HMASTLOCK      ),
    .ext_HREADYOUT      ( ext_HREADYOUT      ),
    .ext_HREADY         ( ext_HREADY         ),
    .ext_HRESP          ( ext_HRESP          ),

    /*
     * User AHB
     */
    .usr_HSEL           ( usr_HSEL           ),
    .usr_HADDR          ( usr_HADDR          ),
    .usr_HWDATA         ( usr_HWDATA         ),
    .usr_HRDATA         ( usr_HRDATA         ),
    .usr_HWRITE         ( usr_HWRITE         ),
    .usr_HSIZE          ( usr_HSIZE          ),
    .usr_HBURST         ( usr_HBURST         ),
    .usr_HPROT          ( usr_HPROT          ),
    .usr_HTRANS         ( usr_HTRANS         ),
    .usr_HMASTLOCK      ( usr_HMASTLOCK      ),
    .usr_HREADYOUT      ( usr_HREADYOUT      ),
    .usr_HREADY         ( usr_HREADY         ),
    .usr_HRESP          ( usr_HRESP          ),

    /*
     * User APB32
     */
    .usr_PSEL32         ( usr_PSEL32         ),
    .usr_PENABLE32      ( usr_PENABLE32      ),
    .usr_PPROT32        ( usr_PPROT32        ),
    .usr_PWRITE32       ( usr_PWRITE32       ),
    .usr_PSTRB32        ( usr_PSTRB32        ),
    .usr_PADDR32        ( usr_PADDR32        ),
    .usr_PWDATA32       ( usr_PWDATA32       ),
    .usr_PRDATA32       ( usr_PRDATA32       ),
    .usr_PREADY32       ( usr_PREADY32       ),
    .usr_PSLVERR32      ( usr_PSLVERR32      ),

    /*
     * User APB8
     */
    .usr_PSEL8          ( usr_PSEL8          ),
    .usr_PENABLE8       ( usr_PENABLE8       ),
    .usr_PPROT8         ( usr_PPROT8         ),
    .usr_PWRITE8        ( usr_PWRITE8        ),
    .usr_PADDR8         ( usr_PADDR8         ),
    .usr_PWDATA8        ( usr_PWDATA8        ),
    .usr_PRDATA8        ( usr_PRDATA8        ),
    .usr_PREADY8        ( usr_PREADY8        ),
    .usr_PSLVERR8       ( usr_PSLVERR8       ),

    /*
     * GPIOs
     */
    .gpio_i             ( gpioi              ),
    .gpio_o             ( gpioo              ),
    .gpio_oe_o          ( gpiooe             ),

    /*
     * UARTs
     */
    .uart_rxd_i         ( uart_rxd           ),
    .uart_txd_o         ( uart_txd           ),

    /*
     * Interrupts
     */
    .int_usr_i          ( usr_int            ) );
  

  //Tie off USR APB32 bus
generate
  for (genvar n=0; n < USR_APB32_CNT; n++)
  begin
      assign usr_PRDATA32 [n] = 32'h0;
      assign usr_PREADY32 [n] = 1'b1;
      assign usr_PSLVERR32[n] = 1'b0;
  end
endgenerate


  //Assign GPIO
  always_comb
  begin
      gpioi[ 0] = ledi;
      ledo      = gpioo [ 0];
      ledoe     = gpiooe[ 0];
      gpioi[ 1] = switchi;
      switcho   = gpioo [1];
      switchoe  = gpiooe[1];

      for (int n=0; n < 1; n++)
      begin
          gpioi[n +2] = displayi;
          displayo    = gpioo [n+2];
          displayoe   = gpiooe[n+2];
      end

      for (int n=0; n < 5; n++)
      begin
          gpioi[n +4] = expconi;
          expcono     = gpioo [n +4];
          expconoe    = gpiooe[n +4];
      end
  end

  //Assign interrupts
//  assign usr_int = {int_gclk_i, int_spi_i, int_eth_switch_i, int_flash_ctrl_i, int_can_i};


generate
  if (JTAG_SERVER_TYPE == "OPENOCD" ||
      JTAG_SERVER_TYPE == "Openocd" ||
      JTAG_SERVER_TYPE == "openocd" )
  begin
      jtag_vpi #(
        .DEBUG_INFO (1) )
      jtag_vpi0
      (
	    .tms        (jtag_tms   ),
	    .tck        (jtag_tck   ),
	    .tdi        (jtag_tdi   ),
	    .tdo        (jtag_tdo   ),

	    .enable     (jtag_trstn ),
	    .init_done  (1'b1       )
      );

    initial jtag_vpi0.main(); 
  end
  else if (JTAG_SERVER_TYPE == "VPI" ||
           JTAG_SERVER_TYPE == "vpi")
  begin 
      dbg_comm_vpi #(
        .JP_PORT (4567) )
      jtag_sim (
        .TRSTN   (jtag_trstn ),
        .TCK     (jtag_tck   ),
        .TMS     (jtag_tms   ),
        .TDI     (jtag_tdi   ),
        .TDO     (jtag_tdo   )
      );
  end
  else
  begin
  end
endgenerate
  

`ifdef USE_SDRAM_CONTROLLER
  wire  [31:0] sdram_dq;
  wire  [10:0] sdram_addr;
  wire  [ 1:0] sdram_ba;
  wire  [ 3:0] sdram_dqm;
  wire         sdram_we_n,
               sdram_cs_n,
               sdram_cas_n,
               sdram_ras_n,
               sdram_cke,
               sdram_clk;


  ahb_sdr_ctl_top_rl #(
    .LATTICE_FAMILY ( "ECP5U" ),
    .LATTICE_DEVICE ( "All"   ))
  sdram_ctrl_inst  (
    // AHB interface
    .i_hresetn   ( rst_ahb_n       ),
    .i_hclk      ( clk_ahb         ),

    .i_haddr     ({ {32-SDRAM_HADDR_SIZE{1'b0}}         , //Tie off MSBs (if any)
                    sdram_HADDR[SDRAM_HADDR_SIZE-3  :10], //Controller maps HADDR[11:10] to BA
                    sdram_HADDR[SDRAM_HADDR_SIZE-1 -: 2], //map the MSBs to BA to support readmemh to sdram_memory.bank0
                    sdram_HADDR[                 9  : 0] }),
    .i_hburst    ( sdram_HBURST    ),
    .i_hprot     ( sdram_HPROT     ),
    .i_hready    ( sdram_HREADY    ),
    .i_hsel      ( sdram_HSEL      ),
    .i_hsize     ( sdram_HSIZE     ),
    .i_htrans    ( sdram_HTRANS    ),
    .i_hwdata    ( sdram_HWDATA    ),
    .i_hwrite    ( sdram_HWRITE    ),
    .i_hmastlock ( sdram_HMASTLOCK ),
    .o_hrdata    ( sdram_HRDATA    ),
    .o_hready    ( sdram_HREADYOUT ),
    .o_hresp     ( sdram_HRESP     ),

    //Sdram interface
    .sdr_DQ      ( sdram_dq         ),
    .sdr_A       ( sdram_addr[10:0] ),
    .sdr_BA      ( sdram_ba         ),
    .sdr_CKE     ( sdram_cke        ),
    .sdr_CSn     ( sdram_cs_n       ),
    .sdr_RASn    ( sdram_ras_n      ),
    .sdr_CASn    ( sdram_cas_n      ),
    .sdr_WEn     ( sdram_we_n       ),
    .sdr_DQM     ( sdram_dqm        ),
    .sdr_CLK     ( sdram_clk        ),

    //Global
    .sys_DLY_100US (),
    .pll_lock      () );


  /* GSR and PUR used by SDRAM controller
   * Naming must be like this, this is !$&@^ hardcoded somewhere
   */
  GSR GSR_INST ( .GSR (poweron_rst_n) );
  PUR PUR_INST ( .PUR (poweron_rst_n) );


  /* SDRAM Model
  */
  IS42VM32200M #(
    .no_of_bank  ( 2               ),
    .no_of_addr  ( SDRAM_ADDR_SIZE ),
    .no_of_data  ( SDRAM_DATA_SIZE ),
    .no_of_col   ( 8               ),
    .no_of_dqm   ( 4               ),
    .mem_sizes   ( 524288          ) )
  sdram_memory     (
    .dq          ( sdram_dq        ),
    .addr        ( sdram_addr      ),
    .ba          ( sdram_ba        ),
    .clk         ( sdram_clk       ),
    .cke         ( sdram_cke       ),
    .csb         ( sdram_cs_n      ),
    .rasb        ( sdram_ras_n     ),
    .casb        ( sdram_cas_n     ),
    .web         ( sdram_we_n      ),
    .dqm         ( sdram_dqm       ) );

`else

  /*
   *  Emulate SDRAM as SRAM
   */
  memory_model_ahb3lite #(
    .DATA_WIDTH ( XLEN                   ),
    .ADDR_WIDTH ( SDRAM_HADDR_SIZE+1     ),
    .BASE       (                      0 ),
    .PORTS      (                      1 ),
    .LATENCY    (                      3 ) )
  sdram_memory (
    .HRESETn    (         rst_ahb_n      ),
    .HCLK       (         clk_ahb        ),
    .HSEL       ('{       sdram_HSEL    }),
    .HTRANS     ('{       sdram_HTRANS  }),
    .HADDR      ('{ {1'b1,sdram_HADDR}  }), //memory located at 0x8000_0000
    .HWRITE     ('{     sdram_HWRITE    }),
    .HSIZE      ('{     sdram_HSIZE     }),
    .HBURST     ('{     sdram_HBURST    }),
    .HWDATA     ('{     sdram_HWDATA    }),
    .HRDATA     ('{     sdram_HRDATA    }),
    .HREADY     ('{     sdram_HREADY    }),
    .HREADYOUT  ('{     sdram_HREADYOUT }),
    .HRESP      ('{     sdram_HRESP     }) );


  assign sdram_init_done = 1'b1;

`endif


  /*
   *  Emulate external (Flash) AHB as memory
   */
  memory_model_ahb3lite #(
    .DATA_WIDTH ( XLEN            ),
    .ADDR_WIDTH ( EXT_HADDR_SIZE  ),
    .BASE       (               0 ),
    .PORTS      (               1 ),
    .LATENCY    (               3 ) )
  ext_ahb (
    .HRESETn    (  rst_ahb_n      ),
    .HCLK       (  clk_ahb        ),
    .HSEL       ('{ext_HSEL      }),
    .HTRANS     ('{ext_HTRANS    }),
    .HADDR      ('{ext_HADDR     }),
    .HWRITE     ('{ext_HWRITE    }),
    .HSIZE      ('{ext_HSIZE     }),
    .HBURST     ('{ext_HBURST    }),
    .HWDATA     ('{ext_HWDATA    }),
    .HRDATA     ('{ext_HRDATA    }),
    .HREADY     ('{ext_HREADY    }),
    .HREADYOUT  ('{ext_HREADYOUT }),
    .HRESP      ('{ext_HRESP     }) );

  /*
   *  Simulate User Slave AHB as memory
   */
  memory_model_ahb3lite #(
    .DATA_WIDTH ( XLEN            ),
    .ADDR_WIDTH ( USR_HADDR_SIZE  ),
    .BASE       (               0 ),
    .PORTS      ( USR_SAHB_CNT    ),
    .LATENCY    (               3 ) )
  usr_ahb (
    .HRESETn    (  rst_ahb_n      ),
    .HCLK       (  clk_ahb        ),
    .HSEL       (  usr_HSEL       ),
    .HTRANS     (  usr_HTRANS     ),
    .HADDR      (  usr_HADDR      ),
    .HWRITE     (  usr_HWRITE     ),
    .HSIZE      (  usr_HSIZE      ),
    .HBURST     (  usr_HBURST     ),
    .HWDATA     (  usr_HWDATA     ),
    .HRDATA     (  usr_HRDATA     ),
    .HREADY     (  usr_HREADY     ),
    .HREADYOUT  (  usr_HREADYOUT  ),
    .HRESP      (  usr_HRESP      ) );


  /*
   * Hookup UART simulator
   */
generate
  for (genvar n=0; n < UART_CNT; n++)
  begin : uart_sim
      uart_sim #(
        .DATARATE  ( BAUD_RATE ),
        .STOPBITS  (         1 ),
        .DATABITS  (         8 ),
        .PARITYBIT (    "NONE" ) )
      uart_sim_inst (
        .rx_i ( uart_txd[n] ),
        .tx_o ( uart_rxd[n] ) );
  end
endgenerate


  /*
   * Hookup Task/Address Monitor (for FreeRTOS)
   */
generate
  localparam unsigned   FREERTOS_TASK_SIZE  = 12*4;
  localparam unsigned   FREERTOS_STACK_SIZE = 'h4000;

  localparam int        FREERTOS_TASKS = 5;
  localparam string     FREERTOS_TASKNAME   [FREERTOS_TASKS] = '{"idle"    , "timer"   , "default" , "job1"    , "job2"    };
  localparam [XLEN-1:0] FREERTOS_TASK_BASE  [FREERTOS_TASKS] = '{'h8003f0b4, 'h8004310c, 'h8004331c, 'h8004b380, 'h8004b3d8};
  localparam [XLEN-1:0] FREERTOS_STACK_BASE [FREERTOS_TASKS] = '{'h8003f10c, 'h80043164, 'h8003b0b4, 'h80043380, 'h80047380};
/*
  //tasks
  for (genvar n=0; n < FREERTOS_TASKS; n++)
  begin: task_mon
      freertos_task_mon #(
        .FILENAME      ( {"rtos_", FREERTOS_TASKNAME [n]} ),
        .XLEN          ( XLEN                      ),
        .ADDRESS_BASE  ( FREERTOS_TASK_BASE[n]     ),
        .ADDRESS_RANGE ( FREERTOS_TASK_SIZE        ))
      task_mon_inst (
        .clk_i        ( clk_ahb                    ),
        .req_i        ( `RV_CORE.dmem_req_o        ),
        .adr_i        ( `RV_CORE.dmem_adr_o        ),
        .d_i          ( `RV_CORE.dmem_d_o          ),
        .q_i          ( `RV_CORE.dmem_q_i          ),
        .we_i         ( `RV_CORE.dmem_we_o         ),
        .size_i       ( `RV_CORE.dmem_size_o       ),
        .ack_i        ( `RV_CORE.dmem_ack_i        ),
        .sp           ( `RV_CORE.int_rf.rf[2]      ),
        .ra           ( `RV_CORE.int_rf.rf[1]      ));
  end

  //task-stacks
  for (genvar n=0; n < FREERTOS_TASKS; n++)
  begin: stack_mon
      freertos_task_mon #(
        .FILENAME      ( {"rtos_", FREERTOS_TASKNAME [n],"_stack"} ),
        .XLEN          ( XLEN                      ),
        .ADDRESS_BASE  ( FREERTOS_STACK_BASE[n]    ),
        .ADDRESS_RANGE ( FREERTOS_STACK_SIZE       ))
      stack_mon_inst (
        .clk_i        ( clk_ahb                    ),
        .req_i        ( `RV_CORE.dmem_req_o        ),
        .adr_i        ( `RV_CORE.dmem_adr_o        ),
        .d_i          ( `RV_CORE.dmem_d_o          ),
        .q_i          ( `RV_CORE.dmem_q_i          ),
        .we_i         ( `RV_CORE.dmem_we_o         ),
        .size_i       ( `RV_CORE.dmem_size_o       ),
        .ack_i        ( `RV_CORE.dmem_ack_i        ),
        .sp           ( `RV_CORE.int_rf.rf[2]      ),
        .ra           ( `RV_CORE.int_rf.rf[1]      ));
  end

  //Current task
  begin: curr_task_mon
      freertos_task_mon #(
        .FILENAME      ( "rtos_current_task"       ),
        .XLEN          ( XLEN                      ),
	.ADDRESS_BASE  ( 'h8003af88                ),
	.ADDRESS_RANGE ( 4                         ))
      curr_task_mon_inst (
        .clk_i        ( clk_ahb                    ),
	.req_i        ( `RV_CORE.dmem_req_o        ),
        .adr_i        ( `RV_CORE.dmem_adr_o        ),
        .d_i          ( `RV_CORE.dmem_d_o          ),
        .q_i          ( `RV_CORE.dmem_q_i          ),
        .we_i         ( `RV_CORE.dmem_we_o         ),
        .size_i       ( `RV_CORE.dmem_size_o       ),
        .ack_i        ( `RV_CORE.dmem_ack_i        ),
        .sp           ( `RV_CORE.int_rf.rf[2]      ),
	.ra           ( `RV_CORE.int_rf.rf[1]      ));
  end
*/

  //Whole SDRAM
  begin: sdram_mon
      freertos_task_mon #(
        .FILENAME      ( "sdram_whole"             ),
        .XLEN          ( XLEN                      ),
        .ADDRESS_BASE  ( 'h80000000                ),
        .ADDRESS_RANGE ( 1024*1024                 ))
      sdram_mon_inst (
        .clk_i        ( clk_ahb                    ),
        .req_i        ( `RV_CORE.dmem_req_o        ),
        .adr_i        ( `RV_CORE.dmem_adr_o        ),
        .d_i          ( `RV_CORE.dmem_d_o          ),
        .q_i          ( `RV_CORE.dmem_q_i          ),
        .we_i         ( `RV_CORE.dmem_we_o         ),
        .size_i       ( `RV_CORE.dmem_size_o       ),
        .ack_i        ( `RV_CORE.dmem_ack_i        ),
        .sp           ( `RV_CORE.int_rf.rf[2]      ),
        .ra           ( `RV_CORE.int_rf.rf[1]      ));
  end



endgenerate


/*
  //Hookup Data Memory Access Validator
    dmav #(
    .XLEN                ( XLEN      ),
    .INIT_FILE           ( BOOTROM_INIT_FILE ),
    .CHECK_CREATE        (1          ),
    .ADDRESS_LOWERBOUND  ('h8000_0000),
    .ADDRESS_UPPERBOUND  ('hffff_ffff)
  //  ,.CHECK_CREATE ( 0         )
  )
  dmav_inst (
    .clk_i        ( clk_ahb                    ),
    .req_i        ( `RV_CORE.dmem_req_o        ),
    .adr_i        ( `RV_CORE.dmem_adr_o        ),
    .d_i          ( `RV_CORE.dmem_d_o          ),
    .q_i          ( `RV_CORE.dmem_q_i          ),
    .we_i         ( `RV_CORE.dmem_we_o         ),
    .size_i       ( `RV_CORE.dmem_size_o       ),
    .lock_i       ( `RV_CORE.dmem_lock_o       ),
    .ack_i        ( `RV_CORE.dmem_ack_i        ),
    .err_i        ( `RV_CORE.dmem_err_i        ),
    .misaligned_i ( `RV_CORE.dmem_misaligned_i ),
    .page_fault_i ( `RV_CORE.dmem_page_fault_i ) );
*/

  /*
   * Board constants
   */
  assign switchi[  0] = 1'b0; //indicate we're in simulation (speed up program)

  assign switchi[6:1] = 6'h0; //tie-off

  initial switchi[7] = 1'b0;
  always #500000 switchi[7] = ~switchi[7];


  /*
   * Generate Clocks
   */
  always #10 clk_ahb = ~clk_ahb; //50MHz AHB clk
  always #40 clk_apb = ~clk_apb; //12.5Hz APB clk

  /*
   * Generate Resets
   */
  initial
  begin
	  #55;
	  poweron_rst_n = 1'b0;
	  #55;
	  poweron_rst_n = 1'b1;
  end


  assign sysrst_n = ~(~poweron_rst_n | rst_sys | ~sdram_init_done);

  always @(posedge clk_ahb, negedge sysrst_n)
    if (!sysrst_n) rst_ahb_n <= 1'b0;
    else           rst_ahb_n <= 1'b1;

  always @(posedge clk_apb, negedge sysrst_n)
    if (!sysrst_n) rst_apb_n <= 1'b0;
    else           rst_apb_n <= 1'b1;



  initial
  begin
      $display("\n\n");
      $display ("------------------------------------------------------------");
      $display (" ,------.                    ,--.                ,--.       ");
      $display (" |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---. ");
      $display (" |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--' ");
      $display (" |  |\\  \\ ' '-' '\\ '-'  |    |  '--.' '-' ' '-' ||  |\\ `--. ");
      $display (" `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---' ");
      $display ("- RISC-V System-on-Chip Testbench -------  `---'  ----------");
      $display ("  XLEN | PRIV | MMU | FPU | AMO | MDU | MULLAT | CORES  ");
      $display ("   %3d | %C%C%C%C | %3d | %3d | %3d | %3d | %6d | %3d   ", 
                 XLEN, "M", HAS_H > 0 ? "H" : " ", HAS_S > 0 ? "S" : " ", HAS_U > 0 ? "U" : " ",
                 HAS_MMU, HAS_FPU, HAS_RVA, HAS_RVM, MULT_LATENCY, CORES);
      $display ("-------------------------------------------------------------");
      $display ("  BootROM    = %s", BOOTROM_INIT_FILE);
      $display ("  SDRAM      = %s", SDRAM_INIT_FILE);
      $display ("  Technology = %s", TECHNOLOGY);
      $display ("  ICache = %0dkB", ICACHE_SIZE);
      $display ("  DCache = %0dkB", DCACHE_SIZE);
      $display ("-------------------------------------------------------------");
      $display ("  Using JTAG server type: %s", JTAG_SERVER_TYPE);
      $display ("\n");

      `ifdef WAVES
          $shm_open("waves");
          $shm_probe("AS",testbench_top,"AS");
          $display("INFO: Signal dump enabled ...\n\n");
      `endif

      //load program into SDRAM
`ifdef USE_SDRAM_CONTROLLER
      //crude way to initialize bank0 of sdram
	$display("INFO: Load SDRAM file\n\n");
      if (SDRAM_INIT_FILE != "") $readmemh(SDRAM_INIT_FILE, sdram_memory.bank0);
`else
      if (SDRAM_INIT_FILE != "") sdram_memory.read_ihex(SDRAM_INIT_FILE);
`endif

      jtag_tck   = 1'b0;
      jtag_tms   = 1'b0;
      jtag_tdi   = 1'b0;
      jtag_tdo   = 1'b0;

      clk_ahb    = 1'b0;
      clk_apb    = 1'b0;

      jtag_trstn = 1'b1;

      repeat (5) @(negedge clk_ahb);
      jtag_trstn = 'b0;

      repeat (5) @(negedge clk_ahb);
      jtag_trstn = 'b1;
  end		

endmodule









/*==============================
  =
  = Data Validation
  =
  ==============================*/
import biu_constants_pkg::*;
/*
 * Data Memory Access Validation
 */
module dmav #(
  parameter XLEN         = 32,
  parameter INIT_FILE    = "inifile",
  parameter CHECK_CREATE = 1,
  parameter ADDRESS_LOWERBOUND = 'h0000_0000,
  parameter ADDRESS_UPPERBOUND = 'hffff_ffff
)
(
  input            clk_i,
  input            req_i,
  input [XLEN-1:0] adr_i,
  input [XLEN-1:0] d_i,
  input [XLEN-1:0] q_i,
  input            we_i,
  input biu_size_t size_i,
  input            lock_i,
  input            ack_i,
                   err_i,
                   misaligned_i,
                   page_fault_i
);

 /////////////////////////////////////////////////////////////////
 //
 // Typedef
 //
 typedef struct {
   logic [XLEN-1:0] adr;
   logic [XLEN-1:0] data;
   logic            we;
   biu_size_t       size;
   logic            lock;
   logic            ack,
                    err,
                    misaligned,
                    page_fault;
   string           comment;
 } data_t;


  /////////////////////////////////////////////////////////////////
  //
  // Functions/Tasks
  //

  //Open golden file, either for reading or writing
  function int golden_open (input string filename, input bit rw);
    golden_open = $fopen(filename, rw ? "r" : "w");

    if (!golden_open) $fatal("Failed to open: %s", filename);
    else              $info ("Opened %s (%0d)", filename, golden_open);
  endfunction: golden_open

  //Close golden file
  //TODO: fd should be argument
  //      call when closing simulator (callback?)
  task golden_close();
    $fclose(fd);
  endtask: golden_close

  //Write datablob to golden file
  //TODO: why doesn't $fwrite work?
  //      why can't a typedef be written at once with %z?
  task golden_write (input int fd, input data_t blob);
//    $display ("fwrite %0t %z", $realtime, blob);
//    $fdisplay (fd, "%z", blob);
    $fdisplay (fd, "%h %h %b %h %b %h %s",
                   blob.adr,
                   blob.data,
                   blob.we,
                   blob.size,
                   blob.lock,
                   {blob.ack, blob.err, blob.misaligned, blob.page_fault},
                   "-" );
  endtask: golden_write

  //Read golden file
  //TODO: ideally would want to read a type_def with %z
  function data_t golden_read(input int fd);
    int err;
    data_t tmp;
    err = $fscanf (fd, "%h %h %b %h %b %h %s",
                       tmp.adr,
                       tmp.data,
                       tmp.we,
                       tmp.size,
                       tmp.lock,
                       {tmp.ack, tmp.err, tmp.misaligned, tmp.page_fault},
                       tmp.comment );

    if (err != 7)
    begin
        $error ("golden_read");
        return data_t'(-1);
    end
    else
    begin
        return tmp;
    end
  endfunction: golden_read


  //Compare results
  //g=golden
  //r=reference
  function int golden_compare(input data_t g, r);
    r.comment = "-";
    if (r !== g && g.comment[0] !== "+")
    begin
        $display ("ERROR  : golden_compare error @%0t %s", $realtime, g.comment);
        $display ("        | golden %s| reference", {XLEN/4-6{" "}} );
        $display ("adr     | %h | %h",   g.adr,  r.adr);
        $display ("data    | %h | %h",   g.data, r.data);
        $display ("size    | %h  %s| %h",   g.size, {XLEN/4-2{" "}}, r.size);
        $display ("we/lock | %b%b %s| %b%b", g.we, g.lock, {XLEN/4-2{" "}}, r.we, r.lock);
        $display ("aemp    | %b%b%b%b %s| %b%b%b%b", g.ack, g.err, g.misaligned, g.page_fault, {XLEN/4-4{" "}},
                                                   r.ack, r.err, r.misaligned, r.page_fault);
        return -1;
    end
    else
        return 0;
  endfunction: golden_compare


  task golden_finish();
    //close file
    golden_close();

    //display notice
    $info ("dmav errors: %0d", golden_errors);
  endtask: golden_finish

  /////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  int fd;
  
  data_t queue[$],
	 queue_d,
         queue_q;

  int golden_errors=0;

  /////////////////////////////////////////////////////////////////
  //
  // Module body
  //

  //open file
  initial fd = golden_open({INIT_FILE, ".golden"}, CHECK_CREATE);


  //store access request
  assign queue_d.adr  = adr_i;
  assign queue_d.we   = we_i;
  assign queue_d.size = size_i;
  assign queue_d.lock = lock_i;
  assign queue_d.data = d_i; //gets overwritten for a read


  //push access request into queue
  always @(posedge clk_i)
    if (req_i) queue.push_front(queue_d);

  //wait for acknowledge and write to file
  always @(posedge clk_i)    
    if (ack_i || err_i || page_fault_i)
    begin
        //pop request from queue
        queue_q = queue.pop_back();

        //store response
        queue_q.ack        = ack_i;
        queue_q.err        = err_i;
        queue_q.misaligned = misaligned_i;
        queue_q.page_fault = page_fault_i;

        if (!queue_q.we) queue_q.data = q_i;

        if (CHECK_CREATE)
        begin
            //read from file and compare
	    if(queue_q.adr > ADDRESS_LOWERBOUND && queue_q.adr < ADDRESS_UPPERBOUND)
               if (golden_compare(golden_read(fd), queue_q) ) golden_errors++;
        end
        else
        begin
            //write to file
	    if(queue_q.adr > ADDRESS_LOWERBOUND && queue_q.adr < ADDRESS_UPPERBOUND)
            	golden_write(fd, queue_q);
        end
    end


endmodule
