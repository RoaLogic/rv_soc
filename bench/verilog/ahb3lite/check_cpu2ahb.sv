/////////////////////////////////////////////////////////////////
//                                                             //
//    ██████╗  ██████╗  █████╗                                 //
//    ██╔══██╗██╔═══██╗██╔══██╗                                //
//    ██████╔╝██║   ██║███████║                                //
//    ██╔══██╗██║   ██║██╔══██║                                //
//    ██║  ██║╚██████╔╝██║  ██║                                //
//    ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝                                //
//          ██╗      ██████╗  ██████╗ ██╗ ██████╗              //
//          ██║     ██╔═══██╗██╔════╝ ██║██╔════╝              //
//          ██║     ██║   ██║██║  ███╗██║██║                   //
//          ██║     ██║   ██║██║   ██║██║██║                   //
//          ███████╗╚██████╔╝╚██████╔╝██║╚██████╗              //
//          ╚══════╝ ╚═════╝  ╚═════╝ ╚═╝ ╚═════╝              //
//                                                             //
//    RISC-V                                                   //
//    Check CPU Memory Access to CPU AHB bus                   //
//                                                             //
/////////////////////////////////////////////////////////////////
//                                                             //
//     Copyright (C) 2016 ROA Logic BV                         //
//                                                             //
//    This confidential and proprietary software is provided   //
//  under license. It may only be used as authorised by a      //
//  licensing agreement from ROA Logic BV.                     //
//  No parts may be copied, reproduced, distributed, modified  //
//  or adapted in any form without prior written consent.      //
//  This entire notice must be reproduced on all authorised    //
//  copies.                                                    //
//                                                             //
//    TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT      //
//  SHALL ROA LOGIC BE LIABLE FOR ANY INDIRECT, SPECIAL,       //
//  CONSEQUENTIAL OR INCIDENTAL DAMAGES WHATSOEVER (INCLUDING, //
//  BUT NOT LIMITED TO, DAMAGES FOR LOSS OF PROFIT, BUSINESS   //
//  INTERRUPTIONS OR LOSS OF INFORMATION) ARISING OUT OF THE   //
//  USE OR INABILITY TO USE THE PRODUCT WHETHER BASED ON A     //
//  CLAIM UNDER CONTRACT, TORT OR OTHER LEGAL THEORY, EVEN IF  //
//  ROA LOGIC WAS ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.  //
//  IN NO EVENT WILL ROA LOGIC BE LIABLE TO ANY AGGREGATED     //
//  CLAIMS MADE AGAINST ROA LOGIC GREATER THAN THE FEES PAID   //
//  FOR THE PRODUCT                                            //
//                                                             //
/////////////////////////////////////////////////////////////////

module check_cpu2ahb #(
  parameter XLEN = 32,
  parameter PHYS_ADDR_SIZE = XLEN
)
(
  //CPU Interface
  input      [XLEN          -1:0] mem_adr,
                                  mem_d,       //from CPU
  input                           mem_req,
                                  mem_we,
  input      [XLEN/8        -1:0] mem_be,
  input      [XLEN          -1:0] mem_q,       //to CPU
  input                           mem_ack,
  input                           mem_misaligned,


  //AHB Interface
  input                           HRESETn,
                                  HCLK,
  input                           HSEL,
  input      [PHYS_ADDR_SIZE-1:0] HADDR,
  input      [XLEN          -1:0] HWDATA,
  input      [XLEN          -1:0] HRDATA,
  input                           HWRITE,
  input      [               2:0] HSIZE,
  input      [               2:0] HBURST,
  input      [               3:0] HPROT,
  input      [               1:0] HTRANS,
  input                           HMASTLOCK,
  input                           HREADY,
  input                           HRESP
);

//////////////////////////////////////////////////////////////////
//
// Constants
//
import ahb3lite_pkg::*;

typedef struct packed {
  logic [XLEN  -1:0] adr;
  logic              we;
  logic [XLEN/8-1:0] be;
  logic [XLEN  -1:0] d;
} mem_if_struct;


//////////////////////////////////////////////////////////////////
//
// Variables
//
mem_if_struct from_cpu,
                 check;
logic            dmem_req;

logic            ahb_selected,
                 ahb_write;

mem_if_struct    q[$];

////////////////////////////////////////////////////////////////
//
// Module Body
//

  /*
   * Queue push
   */
  always @(posedge HCLK)
    if (mem_req)
    begin
        from_cpu.adr <= mem_adr;
        from_cpu.we  <= mem_we;
        from_cpu.be  <= mem_be;
        from_cpu.d   <= mem_d;
    end


  always @(posedge HCLK) dmem_req <= mem_req;


  always @(posedge HCLK)
    if (dmem_req && !mem_misaligned) q.push_front(from_cpu);



  /*
   * Queue pop
   */
  assign ahb_selected = HREADY && HSEL && (HTRANS == HTRANS_NONSEQ || HTRANS == HTRANS_SEQ);

  always @(posedge HCLK)
    if ( ahb_selected )
    begin
        //Is queue empty?

        //get next item in queue
        check = q.pop_back;

        //check signals
        if (HADDR !== check.adr[PHYS_ADDR_SIZE-1:0])
          $display("ERROR  : got HADDR=%x, expected %x @%0t", HADDR, check.adr[PHYS_ADDR_SIZE-1:0], $time);

        if (HWRITE !== check.we)
          $display("ERROR  : got HWRITE=%x, expected %x @%0t", HWRITE, check.we, $time);
    end


  always @(posedge HCLK)
    if (HREADY) ahb_write <= HSEL && HWRITE && (HTRANS == HTRANS_NONSEQ || HTRANS == HTRANS_SEQ);


  always @(negedge HCLK)
   if (HREADY && ahb_write)
     if (HWDATA !== check.d)
       $display("ERROR  : got HWDATA=%x, expected %x @%0t", HWDATA, check.d, $time);
endmodule
