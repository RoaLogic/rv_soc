/////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.          //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.    //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'    //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.    //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'    //
//                                             `---'               //
//    RISC-V                                                       //
//    Lattice ECP5 Versa Development Kit Top Level                 //
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


module ecp5versa_top #(
  parameter TECHNOLOGY         = "GENERIC",
//`ifdef SIM
  parameter INIT_FILE          = "../../sw/riscv-blink-danfoss/Debug/riscv-blink.hex.ver", // Verilog compatible format
//`else
//  parameter INIT_FILE          = "../../sw/riscv-blink-int/Debug/riscvlattice.mem",      // Lattice Format: Do not use symbols in file name, single extension only
//`endif
  parameter BOOTROM_SIZE       = 4, //in kBytes (BootROM    )
  parameter RAM_SIZE           = 4, //in kBytes (on-chip RAM)

  // CPU Parameters
  parameter XLEN               = 32,
  parameter HAS_USER           = 0,
  parameter HAS_SUPER          = 0,
  parameter HAS_HYPER          = 0,
  parameter HAS_BPU            = 1,
  parameter HAS_FPU            = 0,
  parameter HAS_MMU            = 0,
  parameter HAS_RVM            = 1,
  parameter HAS_RVA            = 0,
  parameter HAS_RVC            = 1,
  
  parameter MULT_LATENCY       = 3,  //ECP5 too slow

  parameter BP_GLOBAL_BITS     = 2,
  parameter BP_LOCAL_BITS      = 10,

  parameter ICACHE_SIZE        = 4,  //in KBytes
  parameter ICACHE_BLOCK_SIZE  = 32, //in Bytes
  parameter ICACHE_WAYS        = 2,  //'n'-way set associative
  parameter ICACHE_REPLACE_ALG = 0,

  parameter DCACHE_SIZE        = 4,  //in KBytes
  parameter DCACHE_BLOCK_SIZE  = 32, //in Bytes
  parameter DCACHE_WAYS        = 2,  //'n'-way set associative
  parameter DCACHE_REPLACE_ALG = 0,
  parameter CORES              = 1
)	
(
  /** CPU Debug JTAG Ports
   */

  //system ports
  input         rst_ni,              //asynchronous, active low reset
  input         clk_i,               //Main clock

  //JTAG
  input         dbg_trst_n,
                dbg_tck,
                dbg_tms,
                dbg_tdi,
  output        dbg_tdo,


  //Configuration/Data FLASH
  output       flash_cs_no,          //FLASH Chip Select
  input        flash_miso_i,         //Serial data from FLASH
  output       flash_mosi_o,         //Serial data to FLASH
//  output       flash_wpj_o,          //FLASH write protect
`ifdef SIM
               flash_sclk_o,         //FLASH serial clock
`endif


  //SDRAM Controller - SPH
  inout  [31:0] sdram_dq,
  output [11:0] sdram_addr,
  output [ 1:0] sdram_ba,
  output [ 3:0] sdram_dqm,
  output        sdram_we_n,
                sdram_cs_n,
                sdram_cas_n,
                sdram_ras_n,
                sdram_cke,
                sdram_clk,


  //UART
  input        uart_rxd,
  output       uart_txd,


  //GPIO
  // 8LEDs
  //
  // U1pin   Signal   LED
  //  E18     LED0    D21
  //  D18     LED1    D22
  //  D17     LED2    D24
  //  E16     LED3    D25
  //  F17     LED4    D26
  //  F18     LED5    D27
  //  F16     LED6    D29
  //  E17     LED7    D28
  inout [ 7:0] led_io,

  // 8 DIP switches
  //
  // U1pin  Signal  Switch  Port
  //  H2    SWITCH1   1     switch[0]
  //  K3    SWITCH2   2     switch[1]
  //  G3    SWITCH3   3     switch[2]
  //  F2    SWITCH4   4     switch[3]
  //  J18   SWITCH5   5     switch[4]
  //  K18   SWITCH6   6     switch[5]
  //  K19   SWITCH7   7     switch[6]
  //  K20   SWITCH8   8     switch[7]
  inout [ 7:0] switch_io,

  //14 segment LED Display
  //
  // U1pin  Signal  Segment  port
  //  U1    SEG14   DP       display[14]
  //  R17   SEG13   P        display[13]
  //  R16   SEG12   N        display[12]
  //  P16   SEG11   M        display[11]
  //  N17   SEG10   L        display[10]
  //  P17   SEG9    K        display[ 9]
  //  N18   SEG8    J        display[ 8]
  //  M17   SEG7    H        display[ 7]
  //  N16   SEG6    G        display[ 6]
  //  M18   SEG5    F        display[ 5]
  //  L17   SEG4    E        display[ 4]
  //  L16   SEG3    D        display[ 3]
  //  M19   SEG2    C        display[ 2]
  //  L18   SEG1    B        display[ 1]
  //  M20   SEG0    A        display[ 0]
  inout [14:0] display_io,

  //Expansion Connectors
  inout [45:0] expcon_io
);
  
  //////////////////////////////////////////////////////////////////
  //
  // Typedef
  //

  typedef struct packed {
    logic can;
    logic flash_ctrl;
    logic eth_switch;
    logic spi;
    logic gclk;
  } usr_int_t;


  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //

  localparam GPIO_CNT         = 10;
  localparam UART_CNT         = 1;
  localparam USR_SAHB_CNT     = 3;
  localparam USR_APB32_CNT    = 2;
  localparam USR_APB8_CNT     = 0;
  localparam USR_INT_CNT      = $bits(usr_int_t);

  localparam HDATA_SIZE       = 32;
  localparam SDRAM_HADDR_SIZE = 31;
  localparam EXT_HADDR_SIZE   = 24;
  localparam USR_HADDR_SIZE   = 25 - $clog2(USR_SAHB_CNT ) -1;
  localparam USR32_PADDR_SIZE = 20 - $clog2(USR_APB32_CNT) -1;
  localparam USR8_PADDR_SIZE  = 20 - $clog2(USR_APB8_CNT ) -1;




  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //

  //for tristate buffer
  wire                          cpu_jtag_tdo,
                                cpu_jtag_tdoe;


  //clocks and resets
  wire                          clk100,
                                clk50,
                                clk25,
                                clk20;

  wire                          pll_lock,
                                rst_n;

  //System reset from debugger
  wire                          rst_sys;

  //synchronised resets
  wire                          rst100_n,
                                rst50_n,
                                rst25_n,
                                rst20_n;

  reg   [                  1:0] rst100_reg,
                                rst50_reg,
                                rst25_reg,
                                rst20_reg;


  //SDRAM AHB Bus
  wire                          sdram_HSEL;
  wire  [SDRAM_HADDR_SIZE -1:0] sdram_HADDR;
  wire  [HDATA_SIZE       -1:0] sdram_HWDATA;
  wire  [HDATA_SIZE       -1:0] sdram_HRDATA;
  wire                          sdram_HWRITE;
  wire  [HSIZE_SIZE       -1:0] sdram_HSIZE;
  wire  [HBURST_SIZE      -1:0] sdram_HBURST;
  wire  [HPROT_SIZE       -1:0] sdram_HPROT;
  wire  [HTRANS_SIZE      -1:0] sdram_HTRANS;
  wire                          sdram_HMASTLOCK;
  wire                          sdram_HREADY;
  wire                          sdram_HREADYOUT;
  wire                          sdram_HRESP;

  //Ext (Flash) AHB Bus
  wire                          ext_HSEL;
  wire  [EXT_HADDR_SIZE   -1:0] ext_HADDR;
  wire  [HDATA_SIZE       -1:0] ext_HWDATA;
  wire  [HDATA_SIZE       -1:0] ext_HRDATA;
  wire                          ext_HWRITE;
  wire  [HSIZE_SIZE       -1:0] ext_HSIZE;
  wire  [HBURST_SIZE      -1:0] ext_HBURST;
  wire  [HPROT_SIZE       -1:0] ext_HPROT;
  wire  [HTRANS_SIZE      -1:0] ext_HTRANS;
  wire                          ext_HMASTLOCK;
  wire                          ext_HREADY;
  wire                          ext_HREADYOUT;
  wire                          ext_HRESP;

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
 
 /* 
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
*/

  //GPIO
  logic [                  7:0] gpioi         [GPIO_CNT     ];
  logic [                  7:0] gpioo         [GPIO_CNT     ];
  logic [                  7:0] gpiooe        [GPIO_CNT     ];

  //UART
  logic                         uartrxd       [UART_CNT     ];
  logic                         uarttxd       [UART_CNT     ];


  //Interrupts
  usr_int_t                     usr_int;


  //devboard GPIO
  logic [ 7:0] ledi    , ledo    , ledoe;
  logic [ 7:0] switchi , switcho , switchoe;
  logic [15:0] displayi, displayo, displayoe;
  logic [47:0] expconi , expcono , expconoe;



  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //


  /** Lattice PLL
   */
  clkdiv clkdiv_inst (
    .CLKI   ( clk_i    ),   //Input clock
    .CLKOP  ( clk100   ),   //100MHz CPU/AHB clock
    .CLKOS  ( clk50    ),   //50MHz APB clock
    .CLKOS2 ( clk25    ),   //25MHz APB clock
    .CLKOS3 ( clk20    ),   //20MHz customer clock
    .LOCK   ( pll_lock ) ); //PLL locked
 


  /** Resets
   */
  assign rst_n = ~(~rst_ni | ~pll_lock | rst_sys);


  always @(posedge clk100, negedge rst_n)
    if (!rst_n) rst100_reg <= 2'h0;
    else        rst100_reg <= {1'b1, rst100_reg[1 +: $bits(rst100_reg)-1]};

  assign rst100_n = rst100_reg[0];


  always @(posedge clk50, negedge rst_n)
    if (!rst_n) rst50_reg <= 2'h0;
    else        rst50_reg <= {1'b1, rst50_reg[1 +: $bits(rst50_reg)-1]};

  assign rst50_n = rst50_reg[0];


  always @(posedge clk25, negedge rst_n)
    if (!rst_n) rst25_reg <= 2'h0;
    else        rst25_reg <= {1'b1, rst25_reg[1 +: $bits(rst25_reg)-1]};

  assign rst25_n = rst25_reg[0];


  always @(posedge clk20, negedge rst_n)
    if (!rst_n) rst20_reg <= 2'h0;
    else        rst20_reg <= {1'b1, rst20_reg[1 +: $bits(rst20_reg)-1]};

  assign rst20_n = rst20_reg[0];



  /** CPU SoC Subsystem
   */
  rv_soc_top #(
    .TECHNOLOGY         ( TECHNOLOGY         ),
    .INIT_FILE          ( INIT_FILE          ),
    .BOOTROM_SIZE       ( BOOTROM_SIZE       ),
    .RAM_SIZE           ( RAM_SIZE           ),
 
    .GPIO_CNT           ( GPIO_CNT           ),
    .UART_CNT           ( UART_CNT           ),
    .USR_SAHB_CNT       ( USR_SAHB_CNT       ),
    .USR_APB8_CNT       ( USR_APB8_CNT       ),
    .USR_APB32_CNT      ( USR_APB32_CNT      ),

    .APB_CLK_FREQ       ( 25.0               ), //in MHz
    .UART_BAUDRATE      ( 9600               ),

    .USR_INT_CNT        ( USR_INT_CNT        ),

    // CPU options
    .XLEN               ( XLEN               ),
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
    /*
     * global power-on reset (for JTAG TAP)
     */
    .poweron_rst_ni     ( 1'b1               ), //FPGA comes up all cleared

    /*
     * JTAG interface (only for RoaLogic TAP controller)
     */
    .jtag_trst_ni       ( cpu_jtag_trst_n    ), //FPGA comes up all cleared cpu_jtag_trst_n    ),
    .jtag_tck_i         ( cpu_jtag_tck       ),
    .jtag_tms_i         ( cpu_jtag_tms       ),
    .jtag_tdi_i         ( cpu_jtag_tdi       ),
    .jtag_tdo_o         ( cpu_jtag_tdo       ),
    .jtag_tdoe_o        ( cpu_jtag_tdoe      ),

    /*
     * Clock & Reset
     */
    .rst_ahb_ni         ( rst50_n            ),
    .clk_ahb_i          ( clk50              ),
    .rst_apb_ni         ( rst25_n            ),
    .clk_apb_i          ( clk25              ),

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
    .sdram_HREADY       ( sdram_HREADY       ),
    .sdram_HREADYOUT    ( sdram_HREADYOUT    ),
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
    .ext_HREADY         ( ext_HREADY         ),
    .ext_HREADYOUT      ( ext_HREADYOUT      ),
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
    .usr_PSEL8          (         ), //( usr_PSEL8          ),
    .usr_PENABLE8       (         ), //( usr_PENABLE8       ),
    .usr_PPROT8         (         ), //( usr_PPROT8         ),
    .usr_PWRITE8        (         ), //( usr_PWRITE8        ),
    .usr_PADDR8         (         ), //( usr_PADDR8         ),
    .usr_PWDATA8        (         ), //( usr_PWDATA8        ),
    .usr_PRDATA8        ( '{7'h0} ), //( usr_PRDATA8        ),
    .usr_PREADY8        ( '{1'b1} ), //( usr_PREADY8        ),
    .usr_PSLVERR8       ( '{1'b0} ), //( usr_PSLVERR8       ),

    /*
     * GPIO
     */
    .gpio_i             ( gpioi              ),
    .gpio_o             ( gpioo              ),
    .gpio_oe_o          ( gpiooe             ),

    /*
     * UART
     */
    .uart_rxd_i         ( uartrxd            ),
    .uart_txd_o         ( uarttxd            ),

    /*
     * Interrupts
     */
    .int_usr_i          ( usr_int            ) );


  //Tie off AHB busses.
  assign usr_HRDATA    = '{32'h0, 32'h0, 32'h0};
  assign usr_HREADYOUT = '{1'b1 , 1'b1 , 1'b1 };
  assign usr_HRESP     = '{HRESP_OKAY, HRESP_OKAY, HRESP_OKAY};

  assign sdram_HRDATA    = 32'h0;
  assign sdram_HREADYOUT = 1'b1;
  assign sdram_HRESP     = HRESP_OKAY;

  //Tie off APB busses
  assign usr_PRDATA32  = '{32'h0, 32'h0};
  assign usr_PREADY32  = '{1'b1 , 1'b1 };
  assign usr_PSLVERR32 = '{1'b0 , 1'b0 };

  //Tie off Interrupts
  assign usr_int = {$bits(usr_int_t){1'b0}};


  //Assign GPIO
  always_comb
  begin
      gpioi[0] = ledi;
      ledo     = gpioo [0];
      ledoe    = gpiooe[0];
      gpioi[1] = switchi;
      switcho  = gpioo [1];
      switchoe = gpiooe[1];

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


  //Assign UART
  assign uart_txd   = uarttxd[0];
  assign uartrxd[0] = uart_rxd;

  

  /*
   * Hookup SPI Flash Controller
   */
  lscc_spi_flash #(
    .LATTICE_FAMILY                   ( "common"      ),
    .S_AHB_ADR_WIDTH                  ( 24            ), //16MBytes = 128Mbit
    .S_AHB_DAT_WIDTH                  ( HDATA_SIZE    ),
    .C_PORT_ENABLE                    ( 0             ),
    .C_APB_ADR_WIDTH                  ( 11            ),
    .C_APB_DAT_WIDTH                  ( 32            ),
    .PAGE_PRG_BUF_ENA                 ( 0             ),
    .PAGE_READ_BUF_ENA                ( 0             ),
    .PAGE_PRG_BUFFER_EBR              ( 0             ),
    .PAGE_PRG_BUFFER_DISTRIBUTED_RAM  ( 1             ),
    .PAGE_READ_BUFFER_EBR             ( 0             ),
    .PAGE_READ_BUFFER_DISTRIBUTED_RAM ( 1             ),
    .SECTOR_SIZE                      ( 32768         ),
    .PAGE_SIZE                        ( 256           ),
    .CLOCK_SEL                        ( 1             ),
    .SPI_READ                         ( 8'h03         ),
    .SPI_FAST_READ                    ( 8'h0b         ),
    .SPI_BYTE_PRG                     ( 8'h02         ),
    .SPI_PAGE_PRG                     ( 8'h02         ),
    .SPI_BLK1_ERS                     ( 8'h20         ),
    .SPI_BLK2_ERS                     ( 8'h52         ),
    .SPI_BLK3_ERS                     ( 8'hd8         ),
    .SPI_CHIP_ERS                     ( 8'h60         ),
    .SPI_WRT_ENB                      ( 8'h06         ),
    .SPI_WRT_DISB                     ( 8'h04         ),
    .SPI_READ_STAT                    ( 8'h05         ),
    .SPI_WRT_STAT                     ( 8'h01         ),
    .SPI_PWD_DOWN                     ( 8'hb9         ),
    .SPI_PWD_UP                       ( 8'hab         ),
    .SPI_DEV_ID                       ( 8'h9f         ) )
   spi_ctrl_inst (
    .clk_i                            ( clk50         ),
    .rst_i                            ( rst50_n       ),

    // AHB PORT A signals
    .ahbl_hsel                        ( ext_HSEL      ),
    .ahbl_htrans                      ( ext_HTRANS    ),
    .ahbl_hburst                      ( ext_HBURST    ),
    .ahbl_hsize                       ( ext_HSIZE     ),
    .ahbl_hwrite                      ( ext_HWRITE    ),
    .ahbl_hlock                       ( ext_HMASTLOCK ),
    .ahbl_haddr                       ( ext_HADDR     ),
    .ahbl_hwdata                      ( ext_HWDATA    ),
    .ahbl_hrdata                      ( ext_HRDATA    ),
    .ahbl_hresp                       ( ext_HRESP     ),
    .ahbl_hready                      ( ext_HREADY    ),
    .ahbl_hready_out                  ( ext_HREADYOUT ),

    // APB PORT B signals
    .apb_psel                         ( 1'b0          ),
    .apb_penable                      ( 1'b0          ),
    .apb_paddr                        (  'h0          ),
    .apb_pwrite                       ( 1'b0          ),
    .apb_pwdata                       (  'h0          ),
    .apb_prdata                       (               ),
    .apb_pready                       (               ),
    .apb_pslverr                      (               ),

    // SPI flash signals
    .sclk_o                           ( flash_sclk_o  ),        // Serial clock
    .ss_n_o                           ( flash_cs_no   ),        // Chip select
    .mosi_o                           ( flash_mosi_o  ),        // Serial data from FPGA to SPI flash
    .miso_i                           ( flash_miso_i  ),        // Serial data from SPI flash to FPGA
    .wpj_o                            ( flash_wpj_o   ) );      // Write protect



    /* Hookup Lattice Special Clock Macro for SPI Flash CLock
    */
`ifndef SIM
  USRMCLK usrmclk_inst (
    .USRMCLKI  ( flash_sclk_o ),
    .USRMCLKTS (!rst50_n      )
  ) /* synthesis syn_noprune=1 */ ;
`endif

  /*
   * Hookup SDRAM controller
   *
/*
  ahb_sdr_ctl_top sdram_ctrl_inst (
    .CLK_I         ( clk50            ),

    .i_haddr       ( { {32-SDRAM_HADDR_SIZE{1'b0}}, sdram_HADDR } ),
    .i_hburst      ( sdram_HBURST     ),
    .i_hprot       ( sdram_HPROT      ),
    .i_hready      ( sdram_HREADY     ),
    .i_hsel        ( sdram_HSEL       ),
    .i_hsize       ( sdram_HSIZE      ),
    .i_htrans      ( sdram_HTRANS     ),
    .i_hwdata      ( sdram_HWDATA     ),
    .i_hwrite      ( sdram_HWRITE     ),
    .o_hrdata      ( sdram_HRDATA     ),
    .o_hready      ( sdram_HREADYOUT  ),
    .o_hresp       ( sdram_HRESP      ),

    .sdr_DQ        ( sdram_dq         ),
    .sdr_A         ( sdram_addr[10:0] ), // SPH - 11 bits needed by Controller, but 12 bits assigned to board
    .sdr_BA        ( sdram_ba         ),
    .sdr_CKE       ( sdram_cke        ),
    .sdr_CSn       ( sdram_cs_n       ),
    .sdr_RASn      ( sdram_ras_n      ),
    .sdr_CASn      ( sdram_cas_n      ),
    .sdr_WEn       ( sdram_we_n       ),
    .sdr_DQM       ( sdram_dqm        ),
    .sdr_CLK       ( sdram_clk        ),
    .sys_DLY_100US ( sdram_init_done  ), // SPH - Delay Subsystem reset release until SDRAM init done
    .pll_lock      ( sdram_pll_lock   )  // SPH - Unused
  );
*/


  /*
   * Pads, Special connections, ...
   */
  lvcmos_bidir_3v3 jtag_tdo_pad_inst (.PAD(jtag_tdo_io), .A_I(jtag_tdo), .Y_O(), .OE_I(jtag_tdoe));

  //LED pads
  lvcmos_bidir_3v3 gpio_pads [7:0] (.PAD(led_io), .A_I(ledo), .Y_O(ledi), .OE_I(ledoe));


  //Switch pads
  lvcmos_bidir_3v3 switch_pads [7:0] (.PAD(switch_io), .A_I(switcho), .Y_O(switchi), .OE_I(switch_e));


  //14 Segment Display pads
  lvcmos_bidir_3v3 display_pads [14:0] (.PAD( display_io[14:0] ), .A_I(displayo[14:0]), .Y_O(displayi[14:0]), .OE_I(displayoe[14:0]));
  assign displayi[15] = 1'b0;


  //expcon pads
  lvcmos_bidir_3v3 expcon_pads45_16 [45:0] (.PAD(expcon_io[45:0]), .A_I(expcono[45:0]), .Y_O(expconi[45:0]), .OE_I(expconoe[45:0]));
  assign expconi[47:46] = 2'b00;


 /*
  * JTAG signals
  */
  assign dbg_tdo          = cpu_jtag_tdo;
  assign cpu_jtag_tdi     = dbg_tdi;
  assign cpu_jtag_tms     = dbg_tms;
  assign cpu_jtag_tck     = dbg_tck;
  assign cpu_jtag_trst_n  = dbg_trst_n;
endmodule

