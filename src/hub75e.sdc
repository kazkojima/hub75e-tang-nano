//Copyright (C)2014-2020 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.6 Beta
//Created Time: 2020-07-05 17:51:58
create_clock -name CLK_IN -period 62.5 -waveform {0 31.25} [get_ports {CLK_IN}]
