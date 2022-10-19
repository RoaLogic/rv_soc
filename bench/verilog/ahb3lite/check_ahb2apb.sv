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
//    Check AHB to APB accesses                                //
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

module check_ahb2apb #(
  parameter HADDR_SIZE = 32,
  parameter HDATA_SIZE = 32,
  parameter PADDR_SIZE = 10,
  parameter PDATA_SIZE =  8
)
(
  //AHB Interface
  input                      HRESETn,
                             HCLK,
  input                      HSEL,
  input [HADDR_SIZE    -1:0] HADDR,
  input [HDATA_SIZE    -1:0] HWDATA,
  input [HDATA_SIZE    -1:0] HRDATA,
  input                      HWRITE,
  input [               2:0] HSIZE,
  input [               2:0] HBURST,
  input [               3:0] HPROT,
  input [               1:0] HTRANS,
  input                      HMASTLOCK,
  input                      HREADY,
  input                      HRESP,

  //APB Interface
  input                     PRESETn,
                            PCLK,
  input                     PSEL,
  input                     PENABLE,
  input  [             2:0] PPROT,
  input                     PWRITE,
  input  [PDATA_SIZE/8-1:0] PSTRB,
  input  [PADDR_SIZE  -1:0] PADDR,
  input  [PDATA_SIZE  -1:0] PWDATA,
  input  [PDATA_SIZE  -1:0] PRDATA,
  input                     PREADY,
  input                     PSLVERR

);

//////////////////////////////////////////////////////////////////
//
// Constants
//
import ahb3lite_pkg::*;

typedef struct packed {
  logic [HADDR_SIZE-1:0] haddr;
  logic                  hwrite;
  logic [           2:0] hsize;
  logic [HDATA_SIZE-1:0] hwdata;
} ahb_if_struct;


//////////////////////////////////////////////////////////////////
//
// Functions
//
function logic [6:0] address_mask;
  input integer data_size;

  //Which bits in HADDR should be taken into account?
  case (data_size)
     1024: address_mask = 7'b111_1111; 
      512: address_mask = 7'b011_1111;
      256: address_mask = 7'b001_1111;
      128: address_mask = 7'b000_1111;
       64: address_mask = 7'b000_0111;
       32: address_mask = 7'b000_0011;
       16: address_mask = 7'b000_0001;
  default: address_mask = 7'b000_0000;
  endcase
endfunction //address_mask

function logic [9:0] data_offset (input [HADDR_SIZE-1:0] haddr);
  logic [6:0] haddr_masked;

  //Generate masked address
  haddr_masked = haddr & address_mask(HDATA_SIZE);

  //calculate bit-offset
  data_offset = 8 * haddr_masked;
endfunction //data_offset


//////////////////////////////////////////////////////////////////
//
// Variables
//
ahb_if_struct from_ahb,
                 check;

logic            ahb_req,
                 ahb_transfer,
                 ahb_write;

ahb_if_struct    q_ahb2apb[$];


logic                  is_ahbread;
logic [HADDR_SIZE-1:0] ahb_haddr;
logic [PDATA_SIZE-1:0] q_apb2ahb[$];
logic [HDATA_SIZE-1:0] check_prdata;


////////////////////////////////////////////////////////////////
//
// Module Body
//

  /*
   * AHB2APB Queue push
   */
  always @(posedge HCLK)
    if (HREADY)
    begin
        from_ahb.haddr  <= HADDR;
        from_ahb.hwrite <= HWRITE;
        from_ahb.hsize  <= HSIZE;
    end


  always @(posedge HCLK) ahb_req <= HREADY && HSEL && (HTRANS == HTRANS_NONSEQ || HTRANS == HTRANS_SEQ);


  always @(posedge HCLK)
    if (ahb_req)
    begin
        from_ahb.hwdata = HWDATA;
        q_ahb2apb.push_front(from_ahb);
    end



  /*
   * AHB2APB Queue pop
   */
  always @(posedge PCLK)
    if ( PSEL && PENABLE )
    begin
        //Is queue empty?
        if (q_ahb2apb.size() == 0)
        begin
            $display("ERROR  : starting APB cycle without AHB request @%t (%m)", $time);
        end
        else
        begin
            //get next item in queue
            check = q_ahb2apb.pop_back;

            //check signals
            if (PADDR !== check.haddr[PADDR_SIZE-1:0])
              $display("ERROR  : got PADDR=%x, expected %x @%0t (%m)", PADDR, check.haddr[PADDR_SIZE-1:0], $time);

            if (PWRITE !== check.hwrite)
              $display("ERROR  : got PWRITE=%x, expected %x @%0t (%m)", PWRITE, check.hwrite, $time);

            if (PWRITE && PWDATA !== (check.hwdata >> data_offset(check.haddr)) )
              $display("ERROR  : got PWDATA=%x, expected %x @%0t (%m)", PWDATA, check.hwdata, $time);
        end
    end



  /*
   * APB2AHB Queue push
   */
  always @(posedge PCLK)
    if (PSEL && PENABLE && !PWRITE) q_apb2ahb.push_front(PRDATA);

  always @(posedge HCLK)
    if (HREADY) ahb_haddr <= HADDR;


  /*
   * APB2AHB Queue pop
   */
  //cycle #1
  always @(posedge HCLK)
    if (HREADY)
      is_ahbread <= HSEL & ~HWRITE & (HTRANS == HTRANS_NONSEQ || HTRANS == HTRANS_SEQ);

  //cycle #2
  always @(posedge HCLK)
    if (HREADY)
      if (is_ahbread)
      begin
          //Is queue empty?
          if (q_apb2ahb.size() == 0)
          begin
              $display("ERROR  : Reading from APB, but no APB read data @%t (%m)", $time);
          end
          else
          begin
              //get next item in queue
              check_prdata = q_apb2ahb.pop_back << data_offset(ahb_haddr);

//$display ("%x, %x, %x, %x, %x, %x", ahb_haddr, HRDATA, check_prdata, (check_prdata << data_offset(ahb_haddr)), tmp_hdata, (tmp_hdata << data_offset(ahb_haddr)) );
              //check signals
              if (HRDATA !== check_prdata)
                $display("ERROR  : got HRDATA=%x, expected %x @%0t (%m)", HRDATA,  check_prdata, $time);
          end
      end

endmodule
