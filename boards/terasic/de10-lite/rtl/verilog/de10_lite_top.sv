/////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.          //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.    //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'    //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.    //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'    //
//                                             `---'               //
//    RISC-V                                                       //
//    Terasic DE10-Lite Development Kit Top Level                  //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//             Copyright (C) 2023 ROA Logic BV                     //
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


module de10_lite_top
import ahb3lite_pkg::*;
#(
  parameter TECHNOLOGY         = "ALTERA",
  parameter INIT_FILE          = "", // Verilog compatible format
  parameter BOOTROM_SIZE       = 4, //in kBytes (BootROM    )
  parameter RAM_SIZE           = 4, //in kBytes (on-chip RAM)
  parameter SDRAM_SIZE         = 8, //in MBytes

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
  
  parameter MULT_LATENCY       = 3,

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
  //system ports
  input         MAX10_CLK1_50,       //Main clock
                MAX10_CLK2_50,

  input  [ 1:0] KEY,                 //key[0] = rst_ni

  //JTAG
  input         dbg_trst_n,          //GPIOs
                dbg_tck,
                dbg_tms,
                dbg_tdi,
  output        dbg_tdo,


  //SDRAM Controller - SPH
  output        DRAM_CLK,
                DRAM_CKE,
                DRAM_CS_N,
                DRAM_RAS_N,
                DRAM_CAS_N,
                DRAM_WE_N,
  inout  [15:0] DRAM_DQ,
  output [12:0] DRAM_ADDR,
  output [ 1:0] DRAM_BA,
  output        DRAM_LDQM,
                DRAM_UDQM,

  //7Segment Display
  output [ 7:0] HEX0,
                HEX1,
                HEX2,
                HEX3,
                HEX4,
                HEX5,

  //LED
  output [ 9:0] LEDR,

  //Switch
  input  [ 9:0] SW,

  //VGA
  output [ 3:0] VGA_R,
                VGA_G,
                VGA_B,
                VGA_HS,
                VGA_VS,

  //Accelerometer
  output        GSENSOR_SCLK,
                GSENSOR_CS_N,
                GSENSOR_SDO,
  input         GSENSOR_SDI,
  input  [ 1:0] GSENSOR_INT,

  //Arduino
  //-----------------+-----+-----+-----+-----+-------------
  // Signal          | JP2 | JP3 | JP4 | JP7 | Comment
  //-----------------+-----+-----+-----+-----+--------------
  // ARDUINO_RESET_N                 5     3   10k pull-up
  // ARDUINO_IO[ 0]            8               Rx
  // ARDUINO_IO[ 1]            7               Tx
  // ARDUINO_IO[ 2]            6
  // ARDUINO_IO[ 3]            5
  // ARDUINO_IO[ 4]            4
  // ARDUINO_IO[ 5]            3
  // ARDUINO_IO[ 6]            2
  // ARDUINO_IO[ 7]            1
  // ARDUINO_IO[ 8]     10
  // ARDUINO_IO[ 9]      9
  // ARDUINO_IO[10]      8                     SS
  // ARDUINO_IO[11]      7           4         MOSI
  // ARDUINO_IO[12]      6           1         MISO
  // ARDUINO_IO[13]      5           3         SCK
  // ARDUINO_IO[14]      2                     SDA, 2k2 pull-up
  // ARDUINO_IO[15]      1                     SCL, 2k2 pull-up
  output        ARDUINO_RESET_N,
  inout  [15:0] ARDUINO_IO,

  //GPIO
  inout  [35:0] GPIO,

output uart_txd,
input uart_rxd
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

  localparam GPIO_CNT         = 17;
  localparam UART_CNT         = 1;
  localparam USR_SAHB_CNT     = 3;
  localparam USR_APB32_CNT    = 1;
  localparam USR_APB8_CNT     = 0;
  localparam USR_INT_CNT      = $bits(usr_int_t);

  localparam SDRAM_HADDR_SIZE = 22;
  localparam SDRAM_PORTS      = 1;
  localparam SDRAM_CPU_PORT   = 0;

  localparam HDATA_SIZE       = 32;
  localparam EXT_HADDR_SIZE   = 24;
  localparam USR_HADDR_SIZE   = 25 - $clog2(USR_SAHB_CNT ) -1;
  localparam USR32_PADDR_SIZE = 20 - $clog2(USR_APB32_CNT) -1;
  localparam USR8_PADDR_SIZE  = 20 - $clog2(USR_APB8_CNT ) -1;


  localparam GPIO_HEX0     = 0;
  localparam GPIO_HEX1     = 1;
  localparam GPIO_HEX2     = 2;
  localparam GPIO_HEX3     = 3;
  localparam GPIO_HEX4     = 4;
  localparam GPIO_HEX5     = 5;
  localparam GPIO_LEDR0    = 6;
  localparam GPIO_LEDR1    = 7;
  localparam GPIO_SW0      = 8;
  localparam GPIO_SW1      = 9;
  localparam GPIO_ARDUINO0 = 10;
  localparam GPIO_ARDUINO1 = 11;
  localparam GPIO_GPIO0    = 12;
  localparam GPIO_GPIO1    = 13;
  localparam GPIO_GPIO2    = 14;
  localparam GPIO_GPIO3    = 15;
  localparam GPIO_GPIO4    = 16;
 

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
                                sdram_rdclk100;

  wire                          pll_locked,
                                rst_n;

  //System reset from debugger
  wire                          rst_sys;

  //synchronised resets
  wire                          rst100_n,
                                rst50_n,
                                rst25_n;

  reg   [                  1:0] rst100_reg,
                                rst50_reg,
                                rst25_reg,
                                rst20_reg;


  //SDRAM APB Bus
  wire        sdram_PSEL;
  wire        sdram_PENABLE;
  wire [ 3:0] sdram_PADDR;
  wire        sdram_PWRITE;
  wire [ 3:0] sdram_PSTRB;
  wire [ 2:0] sdram_PPROT;
  wire [31:0] sdram_PWDATA;
  wire [31:0] sdram_PRDATA;
  wire        sdram_PREADY;
  wire        sdram_PSLVERR;

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

  //SDRAM Ports
  wire                          sdram_ports_HSEL      [SDRAM_PORTS];
  wire  [SDRAM_HADDR_SIZE -1:0] sdram_ports_HADDR     [SDRAM_PORTS];
  wire  [HDATA_SIZE       -1:0] sdram_ports_HWDATA    [SDRAM_PORTS];
  wire  [HDATA_SIZE       -1:0] sdram_ports_HRDATA    [SDRAM_PORTS];
  wire                          sdram_ports_HWRITE    [SDRAM_PORTS];
  wire  [HSIZE_SIZE       -1:0] sdram_ports_HSIZE     [SDRAM_PORTS];
  wire  [HBURST_SIZE      -1:0] sdram_ports_HBURST    [SDRAM_PORTS];
  wire  [HPROT_SIZE       -1:0] sdram_ports_HPROT     [SDRAM_PORTS];
  wire  [HTRANS_SIZE      -1:0] sdram_ports_HTRANS    [SDRAM_PORTS];
  wire                          sdram_ports_HMASTLOCK [SDRAM_PORTS];
  wire                          sdram_ports_HREADY    [SDRAM_PORTS];
  wire                          sdram_ports_HREADYOUT [SDRAM_PORTS];
  wire                          sdram_ports_HRESP     [SDRAM_PORTS];


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


  //////////////////////////////////////////////////////////////////
  //
  // Module Body
  //


  /** MAX10 PLL
   */
  clk_pll pll_inst (
    .rst      (~KEY[0]         ),
    .refclk   ( MAX10_CLK1_50  ),
    .outclk_0 ( clk100         ),
    .outclk_1 ( clk50          ),
    .outclk_2 ( clk25          ),
    .outclk_3 ( sdram_rdclk100 ),
    .locked   ( pll_locked     )
  );


  /** Resets
   */
  assign rst_n = ~(~pll_locked | rst_sys);


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



  /** CPU SoC Subsystem
   */
  rv_soc_top #(
    .TECHNOLOGY         ( TECHNOLOGY         ),
    .INIT_FILE          ( INIT_FILE          ),
    .BOOTROM_SIZE       ( BOOTROM_SIZE       ),
    .RAM_SIZE           ( RAM_SIZE           ),
    .SDRAM_SIZE         ( SDRAM_SIZE         ),
 
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

  //Tie off Interrupts
  assign usr_int = {$bits(usr_int_t){1'b0}};


  //Assign UART
  assign uart_txd   = uarttxd[0];
  assign uartrxd[0] = uart_rxd;

  

  /*
   * Hookup SDRAM controller
   */
  assign sdram_PSEL    = usr_PSEL32   [SDRAM_CPU_PORT];
  assign sdram_PENABLE = usr_PENABLE32[SDRAM_CPU_PORT];
  assign sdram_PADDR   = usr_PADDR32  [SDRAM_CPU_PORT];
  assign sdram_PWRITE  = usr_PWRITE32 [SDRAM_CPU_PORT];
  assign sdram_PSTRB   = usr_PSTRB32  [SDRAM_CPU_PORT];
  assign sdram_PPROT   = usr_PPROT32  [SDRAM_CPU_PORT];
  assign sdram_PWDATA  = usr_PWDATA32 [SDRAM_CPU_PORT];
  assign usr_PRDATA32  = '{sdram_PRDATA };
  assign usr_PREADY32  = '{sdram_PREADY };
  assign usr_PSLVERR32 = '{sdram_PSLVERR};

  assign sdram_ports_HSEL      = '{sdram_HSEL     };
  assign sdram_ports_HTRANS    = '{sdram_HTRANS   };
  assign sdram_ports_HBURST    = '{sdram_HBURST   };
  assign sdram_ports_HSIZE     = '{sdram_HSIZE    };
  assign sdram_ports_HADDR     = '{sdram_HADDR    };
  assign sdram_ports_HWDATA    = '{sdram_HWDATA   };
  assign sdram_ports_HRDATA    = '{sdram_HRDATA   };
  assign sdram_ports_HWRITE    = '{sdram_HWRITE   };
  assign sdram_ports_HPROT     = '{sdram_HPROT    };
  assign sdram_ports_HMASTLOCK = '{sdram_HMASTLOCK};
  assign sdram_ports_HWDATA    = '{sdram_HWDATA   };
  assign sdram_ports_HREADY    = '{sdram_HREADY   };
  assign sdram_HRESP     = sdram_ports_HRESP    [SDRAM_CPU_PORT]; 
  assign sdram_HREADYOUT = sdram_ports_HREADYOUT[SDRAM_CPU_PORT];
  assign sdram_HRDATA    = sdram_ports_HRDATA   [SDRAM_CPU_PORT];

  logic [15:0] sdram_dqi, sdram_dqo;
  logic        sdram_dqoe;

  ahb3lite_sdram_ctrl #(
    .AHB_PORTS        ( SDRAM_PORTS        ),
    .AHB_CTRL_PORT    ( SDRAM_CPU_PORT     ),
    .HADDR_SIZE       ( SDRAM_HADDR_SIZE   ),
    .HDATA_SIZE       ( HDATA_SIZE         ),

    .SDRAM_DQ_SIZE    ( $bits(DRAM_DQ  )  ),
    .SDRAM_ADDR_SIZE  ( $bits(DRAM_ADDR)  ),

    .INIT_DLY_CNT     ( 2500               ),  //in PCLK cycles
    .WRITEBUFFER_SIZE ( $bits(DRAM_DQ ) *8 ),

    .TECHNOLOGY       ( "ALTERA"           ))
  sdram_ctrl_inst (
    //APB Control/Status Interface
    .PRESETn       ( rst25_n                ),
    .PCLK          ( clk25                  ),
    .PSEL          ( sdram_PSEL             ),
    .PENABLE       ( sdram_PENABLE          ),
    .PADDR         ( sdram_PADDR            ),
    .PWRITE        ( sdram_PWRITE           ),
    .PSTRB         ( sdram_PSTRB            ),
    .PPROT         ( sdram_PPROT            ),
    .PWDATA        ( sdram_PWDATA           ),
    .PRDATA        ( sdram_PRDATA           ),
    .PREADY        ( sdram_PREADY           ),
    .PSLVERR       ( sdram_PSLVERR          ),

    //AHB Data Interface
    .HRESETn       ( rst100_n               ),
    .HCLK          ( clk100                 ),
    .HSEL          ( sdram_ports_HSEL       ),
    .HTRANS        ( sdram_ports_HTRANS     ),
    .HSIZE         ( sdram_ports_HSIZE      ),
    .HBURST        ( sdram_ports_HBURST     ),
    .HPROT         ( sdram_ports_HPROT      ),
    .HMASTLOCK     ( sdram_ports_HMASTLOCK  ),
    .HWRITE        ( sdram_ports_HWRITE     ),
    .HADDR         ( sdram_ports_HADDR      ),
    .HWDATA        ( sdram_ports_HWDATA     ),
    .HRDATA        ( sdram_ports_HRDATA     ),
    .HREADYOUT     ( sdram_ports_HREADYOUT  ),
    .HREADY        ( sdram_ports_HREADY     ),
    .HRESP         ( sdram_ports_HRESP      ),

    //SDRAM Interface
    .sdram_rdclk_i ( sdram_rdclk            ),
    .sdram_clk_o   ( DRAM_CLK               ),
    .sdram_cke_o   ( DRAM_CKE               ),
    .sdram_cs_no   ( DRAM_CS_N              ),
    .sdram_ras_no  ( DRAM_RAS_N             ),
    .sdram_cas_no  ( DRAM_CAS_N             ),
    .sdram_we_no   ( DRAM_WE_N              ),
    .sdram_ba_o    ( DRAM_BA                ),
    .sdram_addr_o  ( DRAM_ADDR              ),
    .sdram_dq_i    ( sdram_dqi              ),
    .sdram_dq_o    ( sdram_dqo              ),
    .sdram_dqoe_o  ( sdram_dqoe             ),
    .sdram_dm_o    ( {DRAM_UDQM, DRAM_LDQM} ));

  lvcmos_bidir_3v3 sdram_dq_pad [15:0] (.PAD(DRAM_DQ), .A_I(sdram_dqo), .Y_O(sdram_dqi), .OE_I(sdram_dqoe));


  /*
   * Pads, Special connections, ...
   */
  lvcmos_bidir_3v3 jtag_tdo_pad (.PAD(jtag_tdo_io), .A_I(jtag_tdo), .Y_O(), .OE_I(jtag_tdoe));

  //7 Segment Display pads
  lvcmos_bidir_3v3 hex0_pads [7:0] (.PAD(HEX0[7:0]), .A_I(gpioo[GPIO_HEX0]), .Y_O(gpioi[GPIO_HEX0]), .OE_I(8'hff));
  lvcmos_bidir_3v3 hex1_pads [7:0] (.PAD(HEX1[7:0]), .A_I(gpioo[GPIO_HEX1]), .Y_O(gpioi[GPIO_HEX1]), .OE_I(8'hff));
  lvcmos_bidir_3v3 hex2_pads [7:0] (.PAD(HEX2[7:0]), .A_I(gpioo[GPIO_HEX2]), .Y_O(gpioi[GPIO_HEX2]), .OE_I(8'hff));
  lvcmos_bidir_3v3 hex3_pads [7:0] (.PAD(HEX3[7:0]), .A_I(gpioo[GPIO_HEX3]), .Y_O(gpioi[GPIO_HEX3]), .OE_I(8'hff));
  lvcmos_bidir_3v3 hex4_pads [7:0] (.PAD(HEX4[7:0]), .A_I(gpioo[GPIO_HEX4]), .Y_O(gpioi[GPIO_HEX4]), .OE_I(8'hff));
  lvcmos_bidir_3v3 hex5_pads [7:0] (.PAD(HEX5[7:0]), .A_I(gpioo[GPIO_HEX5]), .Y_O(gpioi[GPIO_HEX5]), .OE_I(8'hff));

  lvcmos_bidir_3v3 ledr0_pads [7:0] (.PAD(LEDR[7:0]), .A_I(gpioo[GPIO_LEDR0]     ), .Y_O(gpioi[GPIO_LEDR0]     ), .OE_I(8'hff));
  lvcmos_bidir_3v3 ledr1_pads [9:8] (.PAD(LEDR[9:8]), .A_I(gpioo[GPIO_LEDR1][1:0]), .Y_O(gpioi[GPIO_LEDR1][1:0]), .OE_I(2'h3));

  lvcmos_bidir_3v3 sw0_pads [7:0] (.PAD(SW[7:0]), .A_I(gpioo[GPIO_SW0]     ), .Y_O(gpioi[GPIO_SW0]     ), .OE_I(8'h00));
  lvcmos_bidir_3v3 sw1_pads [9:8] (.PAD(SW[9:8]), .A_I(gpioo[GPIO_SW1][1:0]), .Y_O(gpioi[GPIO_SW1][1:0]), .OE_I(2'h3));
  lvcmos_bidir_3v3 arduino_rstn_pad (.PAD(ARDUINO_RESET_N), .A_I(gpioo[GPIO_SW1][7]), .Y_O(gpioi[GPIO_SW1][7]), .OE_I(1'b0));

  lvcmos_bidir_3v3 arduino0_pads [ 7:0] (.PAD(ARDUINO_IO[ 7:0]), .A_I(gpioo[GPIO_ARDUINO0]), .Y_O(gpioi[GPIO_ARDUINO0]), .OE_I(gpiooe[GPIO_ARDUINO0]));
  lvcmos_bidir_3v3 arduino1_pads [15:8] (.PAD(ARDUINO_IO[15:8]), .A_I(gpioo[GPIO_ARDUINO1]), .Y_O(gpioi[GPIO_ARDUINO1]), .OE_I(gpiooe[GPIO_ARDUINO1]));

  lvcmos_bidir_3v3 gpio0_pads [ 7: 0] (.PAD(GPIO[ 7: 0]), .A_I(gpioo[GPIO_GPIO0]     ), .Y_O(gpioi[GPIO_GPIO0]     ), .OE_I(gpiooe[GPIO_GPIO0]     ));
  lvcmos_bidir_3v3 gpio1_pads [15: 8] (.PAD(GPIO[15: 8]), .A_I(gpioo[GPIO_GPIO1]     ), .Y_O(gpioi[GPIO_GPIO1]     ), .OE_I(gpiooe[GPIO_GPIO1]     ));
  lvcmos_bidir_3v3 gpio2_pads [23:16] (.PAD(GPIO[23:16]), .A_I(gpioo[GPIO_GPIO2]     ), .Y_O(gpioi[GPIO_GPIO2]     ), .OE_I(gpiooe[GPIO_GPIO2]     ));
  lvcmos_bidir_3v3 gpio3_pads [31:24] (.PAD(GPIO[31:24]), .A_I(gpioo[GPIO_GPIO3]     ), .Y_O(gpioi[GPIO_GPIO3]     ), .OE_I(gpiooe[GPIO_GPIO3]     ));
  lvcmos_bidir_3v3 gpio4_pads [35:32] (.PAD(GPIO[35:32]), .A_I(gpioo[GPIO_GPIO4][3:0]), .Y_O(gpioi[GPIO_GPIO4][3:0]), .OE_I(gpiooe[GPIO_GPIO4][3:0]));
  assign gpioi[GPIO_GPIO4][7:4] = 4'h0;


 /*
  * JTAG signals
  */
  assign dbg_tdo          = cpu_jtag_tdo;
  assign cpu_jtag_tdi     = dbg_tdi;
  assign cpu_jtag_tms     = dbg_tms;
  assign cpu_jtag_tck     = dbg_tck;
  assign cpu_jtag_trst_n  = dbg_trst_n;
endmodule

