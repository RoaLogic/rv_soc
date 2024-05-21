////////////////////////////////////////////////////////////////////
//   ,------.                    ,--.                ,--.         //
//   |  .--. ' ,---.  ,--,--.    |  |    ,---. ,---. `--' ,---.   //
//   |  '--'.'| .-. |' ,-.  |    |  |   | .-. | .-. |,--.| .--'   //
//   |  |\  \ ' '-' '\ '-'  |    |  '--.' '-' ' '-' ||  |\ `--.   //
//   `--' '--' `---'  `--`--'    `-----' `---' `-   /`--' `---'   //
//                                             `---'              //
//                                                                //
//      Main.c                                                    //
//		Main program code for the bootrom application             //
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

#include "system_definitions.h"
#include "HAL_SDRAM.h"

#define APPLICATIONSTARTADDRESS 0x80000000

typedef void (*startApplication)(void);

int main(void)
{
    uint32_t data;
    HAL_SDRAM_initialize(SDRAMCONTROLLERBASE, 
                         SDRAMDATABASE, 
                         IS42S16320DtREF, 
                         IS42S16320DTimeConfig,
                         IS42S16320DControlRegister,
                         IS42S16320DModeRegister );


    // *(SDRAMDATABASE) = 0x12345678;

    // data = *(SDRAMDATABASE);

    while(1)
    {
        ((startApplication)APPLICATIONSTARTADDRESS)();
    }

}
