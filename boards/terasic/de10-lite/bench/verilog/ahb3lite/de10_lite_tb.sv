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

module de10_lite_tb;
  parameter TECHNOLOGY         = "GENERIC"; //"ALTERA";
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


  parameter BAUD_RATE          = 97656; //2457600; //9600;



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
  localparam SDRAM_ADDR_SIZE = 13;
  localparam SDRAM_DATA_SIZE = 16;


  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic                         jtag_trstn;
  logic                         jtag_tck;
  logic                         jtag_tms;
  logic                         jtag_tdi;
  logic                         jtag_tdo;

  logic                         poweron_rst_n;
  logic                         clk50;
 

  //Ports
  logic [1:0] port_key;
  logic [9:0] port_switch;

 
  //UART
  logic                         uart_rxd      [UART_CNT     ];
  logic                         uart_txd      [UART_CNT     ];
  
  //User Interrupts
  logic [USR_INT_CNT      -1:0] usr_int;


  //SDRAM signals
  wire  [SDRAM_DATA_SIZE  -1:0] sdram_dq;
  logic [SDRAM_ADDR_SIZE  -1:0] sdram_addr;
  logic [                  1:0] sdram_ba;
  logic [SDRAM_DATA_SIZE/8-1:0] sdram_dqm;
  logic                         sdram_we_n,
                                sdram_cs_n,
                                sdram_cas_n,
                                sdram_ras_n,
                                sdram_cke,
                                sdram_clk;



  //////////////////////////////////////////////////////////////////
  //
  // Testbench Body
  //

  /*
   * Instantiate top level
   */
  de10_lite_top #(
    .TECHNOLOGY         ( TECHNOLOGY         ),
    .BOOTROM_SIZE       ( BOOTROM_SIZE       ),
    .RAM_SIZE           ( RAM_SIZE           ),
    .INIT_FILE          ( BOOTROM_INIT_FILE  ),
  
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
    .MAX10_CLK1_50 ( clk50        ),
    .MAX10_CLK2_50 ( clk50        ),

    .KEY           ( port_key     ),

    /*
     * /JTAG interface (only for RoaLogic TAP controller)
     */
    .dbg_trst_n    ( jtag_trstn   ),
    .dbg_tck       ( jtag_tck     ),
    .dbg_tms       ( jtag_tms     ),
    .dbg_tdi       ( jtag_tdi     ),
    .dbg_tdo       ( jtag_tdo     ),

    /*
     * SDRAM
     */
    .DRAM_CLK      ( sdram_clk    ),
    .DRAM_CKE      ( sdram_cke    ),
    .DRAM_CS_N     ( sdram_cs_n   ),
    .DRAM_RAS_N    ( sdram_ras_n  ),
    .DRAM_CAS_N    ( sdram_cas_n  ),
    .DRAM_WE_N     ( sdram_we_n   ),
    .DRAM_DQ       ( sdram_dq     ),
    .DRAM_ADDR     ( sdram_addr   ),
    .DRAM_BA       ( sdram_ba     ),
    .DRAM_LDQM     ( sdram_dqm[0] ),
    .DRAM_UDQM     ( sdram_dqm[1] ),

    /*
     * 7-segment display
     */
    .HEX0 (),
    .HEX1 (),
    .HEX2 (),
    .HEX3 (),
    .HEX4 (),
    .HEX5 (),

    //LED
    .LEDR (),

    //Switches
    .SW            ( port_switch  ),

    /*
     * VGA
     */
    .VGA_R  (),
    .VGA_G  (),
    .VGA_B  (),
    .VGA_HS (),
    .VGA_VS (),

    /*
     * GSensor
     */
    .GSENSOR_SCLK (),
    .GSENSOR_CS_N (),
    .GSENSOR_SDO  (),
    .GSENSOR_SDI  (),
    .GSENSOR_INT  (),

    /*
     * ARDUINO
     */
    .ARDUINO_RESET_N (),
    .ARDUINO_IO      (),

    /*
     * GPIO
     */
    .GPIO            (  ) );
  


/*
 * JTAG Server
 */
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
  

  /* SDRAM Model
  */
  IS42VM32200M #(
    .no_of_bank  ( 2               ),
    .no_of_addr  ( SDRAM_ADDR_SIZE ),
    .no_of_data  ( SDRAM_DATA_SIZE ),
    .no_of_col   ( 10              ),
    .no_of_dqm   ( 2               ),
    .mem_sizes   ( 8388608         ) )
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
   * Board constants
   */
  assign port_switch[  0] = 1'b0; //indicate we're in simulation (speed up program)
  assign port_switch[9:1] = 9'h0; //tie-off


  /*
   * Generate Clocks
   */
  always #10 clk50 = ~clk50; //50MHz AHB clk


  /*
   * Generate Resets
   */
  initial
  begin
	  #55;
	  port_key[0] = 1'b0;
	  #55;
	  port_key[1] = 1'b1;
  end


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
//`ifdef USE_SDRAM_CONTROLLER
      //crude way to initialize bank0 of sdram
	$display("INFO: Load SDRAM file\n\n");
      if (SDRAM_INIT_FILE != "") $readmemh(SDRAM_INIT_FILE, sdram_memory.bank0);
//`else
//      if (SDRAM_INIT_FILE != "") sdram_memory.read_ihex(SDRAM_INIT_FILE);
//`endif

      jtag_tck   = 1'b0;
      jtag_tms   = 1'b0;
      jtag_tdi   = 1'b0;
//      jtag_tdo   = 1'b0;

      clk50      = 1'b0;

      jtag_trstn = 1'b1;

      repeat (5) @(negedge clk50);
      jtag_trstn = 'b0;

      repeat (5) @(negedge clk50);
      jtag_trstn = 'b1;
  end		

endmodule

