////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.         //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.   //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'   //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.   //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'   //
//                                             `---'              //
//                                                                //
//      system_definitions.h                                      //
//		System definitions for the rv_soc on the DE10-lite FPGA   //
//                                                                //
////////////////////////////////////////////////////////////////////
//                                                                //
//     Copyright (C) 2016-2024 ROA Logic BV                       //
//     www.roalogic.com                                           //
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

#include <stdint.h>
#include <stdlib.h>

#include "HAL_SDRAM.h"

////////////////////////////////
// System definitions
////////////////////////////////
#define HCLK_PERIOD_NS  10
#define PCLK_PERIOD_NS  40

#define SDRAMCONTROLLERBASE ((HAL_SDRAM_CONTROLLER*)0x00880000)
#define SDRAMDATABASE ((uint32_t*)0x80000000)

////////////////////////////////
// SDRAM IS42S16320D configuration
////////////////////////////////

// Timing parameters (in ns)
#define tRRD 12.0
#define tRFC 80.0
#define tRC  60.0
#define tRAS 42.0
#define tRCD 18.0
#define tRP  18.0
#define tWR   6.0

#define tRDV  1.0

#define REFRESHES 4096
#define REFRESH_PERIOD 64e-3 // refreshes every 64msecs

#define SDRAM_BUFFER_TIMEOUT_SETTING 3

static const tSDRAMControlRegister IS42S16320DControlRegister =
{
    .asElements.wBufTimeout = SDRAM_BUFFER_TIMEOUT_SETTING,
    .asElements.reserved1 = 0,
    .asElements.dqSize = DQ_SIZE_16,
    .asElements.ap = 0,
    .asElements.iam = IAM_LINEAR,
    .asElements.numCols = COLUMNS_8,
    .asElements.numRows = ROWS_11,
    .asElements.burstSize = BURST_SIZE_8,
    .asElements.reserved2 = 0,
    .asElements.pp = 0,
    .asElements.mode = MODE_REG_NORMAL,
    .asElements.initDone = 0,
    .asElements.enable = 1
};

static const tSDRAMTimeConfig IS42S16320DTimeConfig = 
{
    .asElements.RFC_cnt = GET_HCLK_T_PERIOD(tRFC, HCLK_PERIOD_NS),
    .asElements.RC_cnt  = GET_HCLK_T_PERIOD(tRC , HCLK_PERIOD_NS),
    .asElements.RAS_cnt = GET_HCLK_T_PERIOD(tRAS, HCLK_PERIOD_NS),
    .asElements.RCD_cnt = GET_HCLK_T_PERIOD(tRCD, HCLK_PERIOD_NS),
    .asElements.RP_cnt  = GET_HCLK_T_PERIOD(tRP , HCLK_PERIOD_NS),
    .asElements.WR_cnt  = GET_HCLK_T_PERIOD(tWR , HCLK_PERIOD_NS),
    .asElements.RRD_cnt = GET_HCLK_T_PERIOD(tRRD, HCLK_PERIOD_NS),
    .asElements.cl      = CAS_LATENCY_CL2,
    .asElements.unused1 = 0,
    .asElements.btac    = BUS_TURNAROUND,
    .asElements.RDV_cnt = 0 + 2 + GET_HCLK_T_PERIOD(tRDV, HCLK_PERIOD_NS), //Data valid at controller: PHY-delay (output + input) + PCB delay
    .asElements.unused2 = 0
};

static const uint16_t IS42S16320DtREF = (REFRESHES/REFRESH_PERIOD/HCLK_PERIOD_NS);

static const tSDRAMModeRegister IS42S16320DModeRegister =
{
    .asElements.burstLength = BURST_LENGTH_8,
    .asElements.burstType = BURST_TYPE_SEQUENTIAL,
    .asElements.latency = LATENCY_MODE_2,
    .asElements.operatingMode = OPERATING_MODE_STANDARD,
    .asElements.writeBurstMode = WRITE_BURST_MODE_BURST_LENGTH,
    .asElements.reserved = 0
};
