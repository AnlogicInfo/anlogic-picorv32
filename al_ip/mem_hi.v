/************************************************************\
 **  Copyright (c) 2011/2017 Anlogic, Inc.
 **  All Right Reserved.
\************************************************************/
/************************************************************\
 ** Log	:	This file is generated by Anlogic IP Generator.
 ** File	:	/home/rgwan/anlogic/picorv32_demo/al_ip/mem_hi.v
 ** Date	:	09 02 2017
 ** TD version	:	3.0.987
\************************************************************/

`timescale 1ns / 1ps

module sysmem_hi ( doa, dia, addra, cea, clka, wea, rsta );

	output [7:0] doa;

	input  [7:0] dia;
	input  [9:0] addra;
	input  cea;
	input  clka;
	input  wea;
	input  rsta;




	AL_LOGIC_BRAM #( .DATA_WIDTH_A(8),
				.ADDR_WIDTH_A(10),
				.DATA_DEPTH_A(1024),
				.MODE("SP"),
				.REGMODE_A("NOREG"),
				.WRITEMODE_A("NORMAL"),
				.RESETMODE("SYNC"),
				.IMPLEMENT("9K"),
				.DEBUGGABLE("YES"),
				.FORCE_KEEP("ON"),
				.INIT_FILE("hi.mif"))
			inst(
				.dia(dia),
				.dib({8{1'b0}}),
				.addra(addra),
				.addrb({10{1'b0}}),
				.cea(cea),
				.ceb(1'b0),
				.ocea(1'b0),
				.oceb(1'b0),
				.clka(clka),
				.clkb(1'b0),
				.wea(wea),
				.web(1'b0),
				.rsta(rsta),
				.rstb(1'b0),
				.doa(doa),
				.dob());


endmodule