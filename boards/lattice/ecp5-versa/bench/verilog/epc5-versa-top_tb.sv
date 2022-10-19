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


`timescale 1ns/10ps

import ahb3lite_pkg::*;


module ecp5versa_tb;
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

  parameter ICACHE_SIZE        = 4;  //in kBytes
  parameter ICACHE_BLOCK_SIZE  = 32; //in Bytes
  parameter ICACHE_WAYS        = 2;  //'n'-way set associative
  parameter ICACHE_REPLACE_ALG = 0;

  parameter DCACHE_SIZE        = 0;  //in kBytes
  parameter DCACHE_BLOCK_SIZE  = 32; //in Bytes
  parameter DCACHE_WAYS        = 2;  //'n'-way set associative
  parameter DCACHE_REPLACE_ALG = 0;
  parameter CORES              = 1;

  
  //////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  import ahb3lite_pkg::*;
  
  localparam GPIO_CNT         = 10;
  localparam UART_CNT         = 1;
  localparam USR_SAHB_CNT     = 3;
  localparam USR_APB32_CNT    = 2;
  localparam USR_APB8_CNT     = 1;
  localparam USR_INT_CNT      = 1;
  
  
  localparam PLEN             = 32;
  localparam SDRAM_HADDR_SIZE = 31;
  localparam EXT_HADDR_SIZE   = 24;
  localparam USR_HADDR_SIZE   = 25 - $clog2(USR_SAHB_CNT ) -1;
  localparam USR32_PADDR_SIZE = 20 - $clog2(USR_APB32_CNT) -1;
  localparam USR8_PADDR_SIZE  = 20 - $clog2(USR_APB8_CNT ) -1;


  //////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic        jtag_trstn;
  logic        jtag_tck;
  logic        jtag_tms;
  logic        jtag_tdi;
  logic        jtag_tdo;

  logic        rst_n,
               clk;

  //GPIO
  wire  [ 7:0] led;
  wire  [ 7:0] switch;
  wire  [14:0] display;
  wire  [45:0] expcon;
  

  //UART
  logic        uart_rxd;
  logic        uart_txd;


  /*
   * Instantiate SoC top level
   */
  ecp5versa_top #(
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
  dut (

    .rst_ni             ( rst_n              ),
    .clk_i              ( clk                ),
    
    /*
     * /JTAG interface (only for RoaLogic TAP controller)
     */
    .dbg_trst_n         ( jtag_trstn         ),
    .dbg_tck            ( jtag_tck           ),
    .dbg_tms            ( jtag_tms           ),
    .dbg_tdi            ( jtag_tdi           ),
    .dbg_tdo            ( jtag_tdo           ),

    /*
     * FLASH
     */
    .flash_cs_no        ( ),
    .flash_miso_i       ( ),
    .flash_mosi_o       ( ),
    .flash_sclk_o       ( ),

    /*
     * GPIOs
     */
    .led_io             ( led                ),
    .switch_io          ( switch             ),
    .display_io         ( display            ),
    .expcon_io          ( expcon             ),

    /*
     * UARTs
     */
    .uart_rxd           ( uart_rxd           ),
    .uart_txd           ( uart_txd           ) );
  

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
  else 
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
endgenerate
  

  /*
   * Board constants
   */
  assign switch[  0] = 1'b0; //indicate we're in simulation (speed up program)

  assign switch[7:1] = 6'h0; //tie-off


  /*
   * Generate Clocks
   */
  always #10 clk = ~clk; //100MHz clk

  /*
   * Generate Resets
   */
  initial
  begin
	  #55;
	  rst_n = 1'b0;
	  #55;
	  rst_n = 1'b1;
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

      jtag_tck   = 1'b0;
      jtag_tms   = 1'b0;
      jtag_tdi   = 1'b0;
      jtag_tdo   = 1'b0;

      clk        = 1'b0;

      jtag_trstn = 1'b1;

      repeat (5) @(negedge clk);
      jtag_trstn = 'b0;

      repeat (5) @(negedge clk);
      jtag_trstn = 'b1;
  end		

endmodule
