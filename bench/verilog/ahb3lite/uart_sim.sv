////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.         //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.   //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'   //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.   //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'   //
//                                             `---'              //
//    UART-SIM                                                    //
//    UART Simulation Model                                       //
//                                                                //
////////////////////////////////////////////////////////////////////
//                                                                //
//             Copyright (C) 2022 ROA Logic BV                    //
//             www.roalogic.com                                   //
//                                                                //
//     This source file may be used and distributed without       //
//   restrictions, provided that this copyright statement is      //
//   not removed from the file and that any derivative work       //
//   contains the original copyright notice and the associated    //
//   disclaimer.                                                  //
//                                                                //
//     This soure file is free software; you can redistribute     //
//   it and/or modify it under the terms of the GNU General       //
//   Public License as published by the Free Software             //
//   Foundation, either version 3 of the License, or (at your     //
//   option) any later versions.                                  //
//   The current text of the License can be found at:             //
//   http://www.gnu.org/licenses/gpl.html                         //
//                                                                //
//     This source file is distributed in the hope that it will   //
//   be useful, but WITHOUT ANY WARRANTY; without even the        //
//   implied warranty of MERCHANTABILITY or FITTNESS FOR A        //
//   PARTICULAR PURPOSE. See the GNU General Public License for   //
//   more details.                                                //
//                                                                //
////////////////////////////////////////////////////////////////////


module uart_sim #(
  parameter int    DATARATE  = 9600,
  parameter int    STOPBITS  = 1,     //1, 2
  parameter int    DATABITS  = 8,     //5,6,7,8
  parameter string PARITYBIT = "NONE" //NONE, EVEN, ODD
)
(
  input  rx_i,
  output tx_o
);


//////////////////////////////////////////////////////////////////
//
// Constants
//
localparam LOCAL_CLK = 100; //MHz


//////////////////////////////////////////////////////////////////
//
// Variables
//
logic clk;

logic       rx_reg, rx_reg_dly;
logic       rx_falling_edge;

logic       clk_cnt_clr,  bit_cnt_clr,
            clk_cnt_done, bit_cnt_done;
int         clk_cnt,
            bit_cnt;
logic [7:0] bit_sr,
            latch_rx;

typedef enum {IDLE, START, DATA, PARITY, STOP} fsm_t;
fsm_t fsm_state;


int fd;



//////////////////////////////////////////////////////////////////
//
// Module body
//

/*generate internal clock
*/
initial clk = 1'b0;
always #(500/LOCAL_CLK) clk = ~clk;


/*detect falling edge on incoming data
*/
always @(posedge clk)
  begin
      rx_reg          <= rx_i;
      rx_reg_dly      <= rx_reg;
      rx_falling_edge <= ~rx_reg & rx_reg_dly;
 end


/*bit count-enable
*/
always @(posedge clk)
  if (clk_cnt_clr || clk_cnt_done) clk_cnt <= LOCAL_CLK * 1000000 / DATARATE;
  else                             clk_cnt <= clk_cnt -1;

assign clk_cnt_done = ~|clk_cnt;


/*Data shift register
*/
always @(posedge clk)
  if (clk_cnt_done) bit_sr <= {rx_i, bit_sr[7:1]};


/*Data counter
*/
always @(posedge clk)
  if      (bit_cnt_clr ) bit_cnt <= DATABITS;
  else if (clk_cnt_done) bit_cnt <= bit_cnt -1;

assign bit_cnt_done = ~|bit_cnt;


/*Statemachine
*/
initial fsm_state = IDLE;

always @(posedge clk)
  begin
      clk_cnt_clr <= 1'b0;
      bit_cnt_clr <= 1'b0;

      case (fsm_state)
         //detect start bit
         IDLE  :  if (rx_falling_edge)
                  begin
                      fsm_state   <= START;
                      clk_cnt_clr <= 1'b1;
                      bit_cnt_clr <= 1'b1;
                  end

         //wait for start bit to complete
         START : if (clk_cnt_done && !clk_cnt_clr)
                  begin
                      fsm_state   <= DATA;
                      bit_cnt_clr <= 1'b1;
                  end

         //Shit in data
         DATA  : if (bit_cnt_done)
                 begin
                     fsm_state <= PARITYBIT == "NONE" ? IDLE : PARITY;
                     latch_rx  <= bit_sr;
                     $fwrite(fd, "%0c",bit_sr); $fflush(fd);
                 end

         //Parity Bit
         PARITY: if (clk_cnt_done)
                 begin
                     fsm_state <= IDLE;
                 end
      endcase
  end


/* File, maybe replace with inet connection?
*/

initial fd = $fopen("uart_rx", "w");

endmodule

