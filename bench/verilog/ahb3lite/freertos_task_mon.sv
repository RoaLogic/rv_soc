////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.         //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.   //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'   //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.   //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'   //
//                                             `---'              //
//    FreeRTOS Task Monitor                                       //
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

import biu_constants_pkg::*;

module freertos_task_mon #(
  parameter string     FILENAME      = "FREERTOS_TASK_MON",
  parameter int        XLEN          = 32,
  parameter [XLEN-1:0] ADDRESS_BASE  = 'h0000_0000,
  parameter [XLEN-1:0] ADDRESS_RANGE = 'h4000
)
(
  input            clk_i,
  input            req_i,
  input [XLEN-1:0] adr_i,
  input [XLEN-1:0] d_i,
  input [XLEN-1:0] q_i,
  input            we_i,
  input biu_size_t size_i,
  input            ack_i,
  input [XLEN-1:0] sp, ra
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
// time             t; //time

   logic [XLEN-1:0] sp, ra;
 } data_t;


  /////////////////////////////////////////////////////////////////
  //
  // Functions/Tasks
  //

  //Open log file
  function int fopen (input string filename);
    fopen = $fopen(filename, "w");

    if (!fopen) $fatal("Failed to open: %s", filename);
    else        $info ("FreeRTOS Monitor: Opened %s (%0d)", filename, fopen);
  endfunction: fopen


  //Close file
  //TODO: fd should be argument
  //      call when closing simulator (callback?)
  task fclose();
    $fclose(fd);
  endtask: fclose

  //Write datablob to file
  task fwrite (input int fd, input data_t blob);
    $fdisplay (fd, "%0h,%0h,%0b,%0h,%0h,%0h,%0t",
                   blob.adr,
                   blob.data,
                   blob.we,
                   blob.size,
		   blob.sp,
		   blob.ra,
		   $time
//		   blob.t
              );
  endtask: fwrite


  /////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  int fd;
  
  data_t queue[$],
	 queue_d,
         queue_q;


  /////////////////////////////////////////////////////////////////
  //
  // Module body
  //

  //open file
  initial fd = fopen(FILENAME);


  //store access request
  assign queue_d.adr  = adr_i;
  assign queue_d.we   = we_i;
  assign queue_d.size = size_i;
  assign queue_d.data = d_i; //gets overwritten for a read

  assign queue_d.sp   = sp;
  assign queue_d.ra   = ra;
 

  //push access request into queue
  always @(posedge clk_i)
    if (req_i) queue.push_front(queue_d);

  //wait for acknowledge and write to file
  always @(posedge clk_i)    
    if (ack_i)
    begin
        //pop request from queue
        queue_q = queue.pop_back();

        if (!queue_q.we) queue_q.data = q_i;

        //write to file
	if(queue_q.adr >= ADDRESS_BASE && queue_q.adr < (ADDRESS_BASE + ADDRESS_RANGE))
          fwrite(fd, queue_q);
    end

endmodule

