//============================================================================
// 
//  Port to MiSTer.
//  Copyright (C) 2018,2019 Sorgelig
//
//  ZX80-ZX81 replica for MiST
//  Copyright (C) 2018 Szombathelyi Gyorgy
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================
// TODO (avlixa)
//   - optimizar scandoubler (solo 2x): pend
//   - zx80 mode: only works via RGB
//   - video compuesto ZXUNO: working only with black border or inverse video
//   - BIOS video option: pending
//   - config.ini file: pending
// DONE (avlixa)
//   - scandoubler: ok,
//   - hps ioctl : ok,
//   - hps status : ok,
//   - map IO  :    ok,
//   - check reset :    ok
//   - keyboard from ps2: ok
//   - joystick mapping: ok 
//   - memory buffer en video_mixer/scandouble: ok
//   - zx80 tape loading: working
//   - zx81 tape loading: working
//   - rom file load: working
//   - loading via ear conector: done
//   - chrs: ok
//   - chroma 81: working ZXDOS+, ZXDOS, ZXUNO
//   - F5  button: menu on/off
//   - F9  button: on/off mic sound to ear
//   - F10 button: on/off tape in sound

//`define DEBUG

`ifdef ZXD
module zx81_zxdos_lx25
`elsif ZX2
module zx81_zxdos_lx16
`elsif ZX1
module zx81_zxuno_lx9
`else
module zx81_
`endif
(
	//Master input clock
	input         CLK_50M,

//	//Must be passed to hps_io module
//	inout  [45:0] HPS_BUS,

//	//Base video clock. Usually equals to CLK_SYS.
//	output        CLK_VIDEO,

//	//Multiple resolutions are supported using different CE_PIXEL rates.
//	//Must be based on CLK_VIDEO
//	output        CE_PIXEL,

`ifdef ZXD
	output  [5:0] VGA_R,
	output  [5:0] VGA_G,
	output  [5:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
`elsif ZX2
	output  [5:0] VGA_R,
	output  [5:0] VGA_G,
	output  [5:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
`elsif ZX1
	output  [2:0] VGA_R,
	output  [2:0] VGA_G,
	output  [2:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
   output wire   PAL,
   output wire   NTSC,
`endif
//	output        VGA_DE,    // = ~(VBlank | HBlank)

//	input  [11:0] HDMI_WIDTH,
//	input  [11:0] HDMI_HEIGHT,

	output        LED_USER1,  // 1 - ON, 0 - OFF.
`ifndef ZX1   
   output        LED_USER2,  // 1 - ON, 0 - OFF.
`endif
	output        AUDIO_L,
	output        AUDIO_R,
   input         AUDIO_EAR,
   
   //Keyboard
   input         PS2_CLK,
   input         PS2_DATA,

   //Joystick
`ifdef ZXD
   output        JOY_CLK,
   output        JOY_LOAD,
   input         JOY_DATA,
   output        JOY_SELECT,
`elsif ZX2
   output        JOY_CLK,
   output        JOY_LOAD,
   input         JOY_DATA,
`elsif ZX1
   // Joystick1
   input wire JOY1_U,
   input wire JOY1_D,
   input wire JOY1_L,
   input wire JOY1_R,
   input wire JOY1_A,
   input wire JOY1_B,
   inout wire JOY1_C,	

   // Joystick2
   input wire JOY2_U,
   input wire JOY2_D,
   input wire JOY2_L,
   input wire JOY2_R,
   input wire JOY2_A,
   input wire JOY2_B,
`endif

   //SRAM
`ifdef ZXD
	output [20:0]   sram_addr,
   inout   [7:0]   sram_data,
   output          sram_we_n,
	output          sram_ub_n,
	output          sram_lb_n,
`elsif ZX2
	output [20:0]   sram_addr,
   inout   [7:0]   sram_data,
   output          sram_we_n,
`elsif ZX1
	output [20:0]   sram_addr,
   inout   [7:0]   sram_data,
   output          sram_we_n,
`endif
   
	//SD-SPI
	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS

//	input         OSD_STATUS
);

assign {SD_SCK, SD_MOSI, SD_CS} = 3'bZZZ;

//assign LED_USER  = ioctl_download | tape_ready;

wire CLK_VIDEO;

//wire [1:0] ar = status[22:21];    //Aspect ratio:Original,Full Screen,[ARC1],[ARC2];
//wire       vcrop_en = status[23]; //Vertical Crop:Disabled,216p/270p(5x);
//reg        en216p;
//always @(posedge CLK_VIDEO) begin
//	en216p <= ((HDMI_WIDTH == 1920) && (HDMI_HEIGHT == 1080) && !forced_scandoubler && !scale);
//end

wire vga_de;
//video_freak video_freak
//(
//	.*,
//	.VGA_DE_IN(vga_de),
//	.ARX((!ar) ? 12'd4 : (ar - 1'd1)),
//	.ARY((!ar) ? 12'd3 : 12'd0),
//	.CROP_SIZE((en216p & vcrop_en) ? (hz50 ? 10'd270 : 10'd216) : 10'd0),
//	.CROP_OFF(0),
//	.SCALE(status[25:24]) //Scale:Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;
//);

////`include "build_id.v"
//localparam CONF_STR = {
//	"ZX81;;",
//	"F,O  P  ,Load tape;",
//	"-;",
//	"OLM,Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
//	"O6,Video frequency,50Hz,60Hz;",
//	"O7,Inverse video,Off,On;",
//	"O5,Black border,Off,On;",
//	"OCD,Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%;",
//	"-;",
//	"d1ON,Vertical Crop,Disabled,216p/270p(5x);",
//	"OOP,Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
//	"-;",
//	"O23,Stereo mix,none,25%,50%,100%;", 
//	"-;",
//	"O4,Model,ZX81,ZX80;",
//	"OHI,Slow mode speed,Original,NoWait,x2,x8;",
//	"OAB,Main RAM,16KB,32KB,48KB,1KB;",
//	"OG,Low RAM,Off,8KB;",
//	"D0OEF,CHR$128/UDG,128 Chars,64 Chars,Disabled;",
//	"OJ,QS CHRS,Enabled(F1),Disabled;",
//	"OK,CHROMA81,Disabled,Enabled;",
//	"-;",
//	"O89,Joystick,Cursor,Sinclair,ZX81;",
//	"R0,Reset;",
//	//"V,v",`BUILD_DATE
//   "V,v20210612"
//};

////////////////////   CLOCKS   ///////////////////

wire clk_sys;
wire locked;

pll pll
(	.refclk(CLK_50M),
	.outclk_0(clk_sys), //52MHz
   .rst(1'b0),
	.locked(locked)
);

reg  ce_cpu_p;
reg  ce_cpu_n;
reg  ce_6m5,ce_3m25,ce_psg;

//always @(negedge clk_sys) begin :gen_clock_enable
always @(posedge clk_sys) begin :gen_clock_enable
	reg [4:0] counter; // = 0;
	reg [1:0] turbo ; // = 0;
   if (reset) begin
      counter <= 0;
      turbo <=0;
      ce_3m25 <=0;
      ce_6m5 <=0;
      ce_psg <=0;
      ce_cpu_p<=0;
      ce_cpu_n<=0;
   end else begin
      counter <= counter + 1'd1;
      if(~slow_mode) turbo <= status[18:17]; //Slow mode speed:Original,NoWait,x2,x8;

      if(slow_mode & turbo[1]) begin
         ce_cpu_p <= (!counter[2] & !counter[1:0]) | turbo[0];
         ce_cpu_n <= ( counter[2] & !counter[1:0]) | turbo[0];
      end
      else begin
         ce_cpu_p <= !counter[3] & !counter[2:0];
         ce_cpu_n <=  counter[3] & !counter[2:0];
      end
      ce_3m25  <= !counter[3:0];
      ce_6m5   <= !counter[2:0];
      ce_psg   <= !counter[4:0];
   end
end

//////////////////////  HPS I/O  //////////////////////

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;

wire host_led1, host_led2;

wire [10:0] ps2_key;
wire [24:0] ps2_mouse;

wire  [1:0] buttons;
wire  [4:0] joystick_0;
wire  [4:0] joystick_1;
wire [31:0] status;

wire        forced_scandoubler;
wire [21:0] gamma_bus;
reg       	reset;
wire        hs_aux, vs_aux;

wire osd_window, osd_pixel;
wire [2:0] osd_bkgr;

`ifdef ZX1
wire [6:0] joy1_s, joy2_s;

assign joy1_s[0] = JOY1_U;
assign joy1_s[1] = JOY1_D;
assign joy1_s[2] = JOY1_L;
assign joy1_s[3] = JOY1_R;
assign joy1_s[4] = JOY1_A;
assign joy1_s[5] = JOY1_B;
assign joy1_s[6] = JOY1_C;

assign joy2_s[0] = JOY2_U;
assign joy2_s[1] = JOY2_D;
assign joy2_s[2] = JOY2_L;
assign joy2_s[3] = JOY2_R;
assign joy2_s[4] = JOY2_A;
assign joy2_s[5] = JOY2_B;
assign joy2_s[6] = 1'b1;
`endif

//assign LED_USER  = status[19];

//hps_io #(.STRLEN($size(CONF_STR)>>3)) hps_io
hps_io hps_io
(
	.clk_sys(clk_sys),
   .reset(reset),
   .p_o_reset_n(locked),
   .ps2_kbd_clk_in(PS2_CLK),
   .ps2_kbd_data_in(PS2_DATA),

`ifndef ZX1
   .joy_data(JOY_DATA),
   .joy_clk(JOY_CLK),
   .joy_load_n(JOY_LOAD),
   .hsync_n_s(hs_aux),
`else
   .joy1_i(joy1_s),
   .joy2_i(joy2_s),
`endif

	.ps2_key(ps2_key),
	.ps2_mouse(ps2_mouse),

	.joystick_0(joystick_0),
	.joystick_1(joystick_1),

	.buttons(buttons),
	.status(status),
	.forced_scandoubler(forced_scandoubler),

	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
   .host_led1(host_led1),  
   .host_led2(host_led2),  
   
   //ZPUFlex signals
   .osd_bkgr(osd_bkgr),
   .osd_window(osd_window), 
   .osd_pixel(osd_pixel),
   .vga_hsync_i(hs_aux),
   .vga_vsync_i(vs_aux),

   .spi_clk(SD_SCK),
   .spi_mosi(SD_MOSI),
   .spi_miso(SD_MISO),
   .spi_cs(SD_CS)
);

`ifndef ZX1   
assign LED_USER1 = host_led1;
assign LED_USER2 = ~(host_led2 || tape_led);
assign JOY_SELECT = hs_aux;
`else
assign LED_USER1 = ~(host_led1 || host_led2 || tape_led);
`endif

//zxdos inputs from HPS not available
assign gamma_bus = 0;

///////////////////   CPU   ///////////////////
wire [15:0] addr;
//wire  [7:0] cpu_din;
reg   [7:0] cpu_din;
wire  [7:0] cpu_dout;
wire        nM1;
wire        nMREQ;
wire        nIORQ;
wire        nRD;
wire        nWR;
wire        nRFSH;
wire        nHALT;
wire        nINT = addr[6];


// ZX81 upgrade
// http://searle.hostei.com/grant/zx80/zx80nmi.html
wire nWAIT = ~(nHALT & ~nNMI) | ~zx81 | (slow_mode & |status[18:17]);  //Slow mode speed:Original,NoWait,x2,x8;
wire nNMI = ~(NMIlatch & hsync) | ~zx81;

T80pa cpu
(
	.RESET_n(~reset),
	.CLK(clk_sys),
	.CEN_p(ce_cpu_p),
	.CEN_n(ce_cpu_n),
	.WAIT_n(nWAIT),
	.INT_n(nINT),
	.NMI_n(nNMI),
	.BUSRQ_n(1'b1),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(nHALT),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din)
);

always @* begin
	case({nMREQ, ~nM1 | nIORQ | nRD})
	    'b01: cpu_din <= (~nM1 & nopgen) ? 8'h00 : mem_out;
	    'b10: cpu_din <= io_dout;
	 default: cpu_din <= 8'hFF;
	endcase
end

//wire       tape_in = 0;
wire       tape_in;
//wire [7:0] io_dout;
reg  [7:0] io_dout;
always @* begin
	casex({~kbd_n, zxp_sel, ch81_sel, psg_sel})
		'b1XXX: io_dout <= { tape_in, hz50, 1'b0, key_data[4:0] & joy_kbd };
		'b01XX: io_dout <= zxp_out;
		'b001X: io_dout <= 8'hDF;
		'b0001: io_dout <= psg_out;
		'b0000: io_dout <= 8'hFF;
	endcase
end

reg       tapeloader, tapewrite_we;
wire      ch81_e = status[20] & ~nMREQ & nM1 & &addr[15:14]; //CHROMA81:Disabled,Enabled;

//wire  [7:0] mem_out;
reg  [7:0] mem_out, tape_loader_patch_r;
wire [15:0] addr_zx81_nxt, addr_zx80_nxt;
always @* begin
	casex({ tapeloader, ~status[19] & qs_e, ch81_e, rom_e, ram_e }) //QS CHRS:Enabled(F1),Disabled;
		  'b1_XX_XX: mem_out <= tape_loader_patch[zx81 ? addr_zx81_nxt : addr_zx80_nxt];
        //'b1_XX_XX: mem_out <= tape_loader_patch_r;
		  'b0_1X_XX: mem_out <= qs_out;
		  'b0_01_XX: mem_out <= ch81_out;
		  'b0_00_1X: mem_out <= rom_out;
		  'b0_00_01: mem_out <= ram_out;
		default: mem_out <= 8'hFF;
	endcase
end

//always @* begin
//	casex({ zx81, addr[9:0] }) //
//		  'h747, 'h207: tape_loader_patch_r <= tape_loader_patch[0]; //zx81-h347, zx80-h207
//		  'h748, 'h208: tape_loader_patch_r <= tape_loader_patch[1]; //zx81-h348, zx80-h208
//		  'h749, 'h209: tape_loader_patch_r <= tape_loader_patch[2]; //zx81-h349, zx80-h209
//		  'h74A, 'h20A: tape_loader_patch_r <= tape_loader_patch[3]; //zx81-h34A, zx80-h20A
//		  'h74B, 'h20B: tape_loader_patch_r <= tape_loader_patch[4]; //zx81-h34B, zx80-h20B
//		  'h74C, 'h20C: tape_loader_patch_r <= tape_loader_patch[5]; //zx81-h34C, zx80-h20C
//		  'h74D, 'h20D: tape_loader_patch_r <= tape_loader_patch[6]; //zx81-h34D, zx80-h20D
//      default: tape_loader_patch_r <= 8'h00;
//	endcase
//end
assign addr_zx81_nxt = addr[9:0] - 10'h347;
assign addr_zx80_nxt = addr[9:0] - 10'h207;

//////////////////   MEMORY   //////////////////
wire low16k_e = ~addr[15] | ~mem_size[1];
wire ramLo_e  = ~addr[14] & addr[13] & low16k_e;
wire ramHi_e  = addr[15] & mem_size[1] & (~addr[14] | (nM1 & mem_size[0]));
wire ram_e    = addr[14] | ramHi_e | (ramLo_e & status[16]); //Low RAM:Off,8KB;

//wire [15:0] ram_a;
reg [15:0] ram_a;
reg [15:0] ram_tape_addr;
always @* begin
	casex({tapeloader, ramLo_e, mem_size, addr[15:14]})
		//'b1_X_XX_XX: ram_a <= {2'b01, tape_type ? tape_addr + 14'd8 : tape_addr - 14'd1}; // loading address
      //'b1_X_XX_XX: ram_a <= {2'b01, ram_tape_addr}; // tape loading address
      'b1_X_XX_XX: ram_a <= ram_tape_addr; // tape loading address

		'b0_1_XX_XX: ram_a <= {3'b001,  ~status[15] ? rom_a : addr[12:0] }; //8K at 2000h //status 15:14 - CHR$128/UDG:128 Chars,64 Chars,Disabled;
		'b0_0_00_XX: ram_a <= {6'b010000, addr[9:0] }; //1k

		'b0_0_01_XX,                                  //16K 
		'b0_0_1X_0X,                                  //main 16k for 32K/48K
		'b0_0_10_11: ram_a <= {2'b01,     addr[13:0]}; //mirrored main 16k for 32K

		'b0_0_1X_10: ram_a <= {2'b10,     addr[13:0]}; //data 16k for 32K/48K
		'b0_0_11_11: ram_a <= {nM1, 1'b1, addr[13:0]}; //48K: last 16k on non-M1 cycle, mirrored main 16K on M1 cycle
	endcase
end


reg   [7:0] tape_in_byte,tape_in_byte_r;

// main ram - at SRAM
//wire [7:0] ram_out;
//dpram #(.ADDRWIDTH(16)) ram
////dpram #(.ADDRWIDTH(14)) ram //para debug con chiptune
//(
//	.clock(clk_sys),
//	.address_a(ram_a),
//	.data_a(tapeloader ? tape_in_byte_r : cpu_dout),
//	.wren_a((~nWR & ~nMREQ & ram_e & ~ch81_e) | tapewrite_we),
//	.q_a(ram_out)
//);
reg   [7:0]  ram_in;
reg   [20:0] ram_addr;
reg          ram_we_n;
wire wren_a = (~nWR & ~nMREQ & ram_e & ~ch81_e) | tapewrite_we;
assign sram_addr = ram_addr;
assign sram_data = ram_we_n ? 8'hZZ : ram_in;
assign sram_we_n = ram_we_n;

`ifdef ZXD
assign sram_ub_n = 1'b1;
assign sram_lb_n = 1'b0;
//assign ram_in = tapeloader ? tape_in_byte_r : cpu_dout;

wire  [7:0]  ram_out;
assign ram_out = sram_data;
always @* begin
      ram_in   <= tapeloader ? tape_in_byte_r : cpu_dout;
      ram_addr <= {5'd0, ram_a};
      ram_we_n <= !wren_a;
end
`endif      

reg        zx81;
reg  [1:0] mem_size; //0 - 1k, 1 - 16k, 2 - 32k, 3 - 48k
wire       hz50 = ~status[6]; //Video frequency:50Hz,60Hz;

`ifdef ZX1
assign PAL  = ~hz50;
assign NTSC = hz50;
`endif

//ROM ZX81 / ZX80
wire [12:0] rom_a = nRFSH ? addr[12:0] : { addr[12:9]+(addr[13] & ram_data_latch[7] & addr[8] & ~status[14]), ram_data_latch[5:0], row_counter }; //status 15:14 - CHR$128/UDG:128 Chars,64 Chars,Disabled;
wire        rom_e = ~addr[14] & ~addr[13] & (~addr[12] | zx81) & low16k_e;
wire  [7:0] rom_out;
//dpram #(.ADDRWIDTH(14), .NUMWORDS(12288), .MEM_INIT_FILE("rtl/zx8x.mem")) rom
dpram #(.ADDRWIDTH(14), .MEM_INIT_FILE("rtl/zx8x.mem")) rom
(
	.clock(clk_sys),
	//.address_a({(zx81 ? {1'b0,rom_a[12]} : 2'h2), rom_a[11:0]}),
   .address_a({ ~zx81, rom_a[12:0] }),
	.q_a(rom_out),

	.address_b(ioctl_addr[13:0]),
	.wren_b(ioctl_wr && !ioctl_index), // ioctl_index = 'b00000000 -> CARGAR ROM
	.data_b(ioctl_dout)
);

//ROM81 rom
//(  .Clk(clk_sys),
//   .A(rom_a),
//   .D(rom_out)
//);

always @(posedge clk_sys) begin :block2_reset
	reg[20:0] timeout;

	reset <= buttons[1] | status[0] | (mod[1] & Fn[11]) | |timeout; // status 0 - Reset
	if (reset) begin
		zx81 <= ~status[4]; // Model:ZX81,ZX80;
		mem_size <= status[11:10] + 1'd1; //Main RAM:16KB,32KB,48KB,1KB;
	end
   if(!locked) timeout <= 10000;
   else if(timeout) timeout <= timeout - 1;
	if(zx81 != ~status[4] || mem_size != (status[11:10] + 1'd1)) timeout <= 10000; // Model:ZX81,ZX80; //Main RAM:16KB,32KB,48KB,1KB;
end

////////////////////  TAPE  //////////////////////
reg         tape_type;
reg   [7:0] tape_ram[16383:0];
//reg         tapeloader, tapewrite_we;
reg  [13:0] tape_addr;
reg  [13:0] tape_size;
//reg   [7:0] tape_in_byte,tape_in_byte_r;
wire        tape_ready = |tape_size;  // there is data in the tape memory
// patch the load ROM routines to loop until the memory is filled from $4000(.o file ) $4009 (.p file)
// xor a; loop: nop or scf, jr nc loop, jp h0207 (jp h0203 - ZX80)
`ifdef DEBUG
reg   [7:0] tape_loader_patch[0:6]; //DEBUG
`else
reg   [7:0] tape_loader_patch[0:6] = {8'haf, 8'h00, 8'h30, 8'hfd, 8'hc3, 8'h07, 8'h02};
`endif 

always @(posedge clk_sys) begin  :block3_write_to_tape_ram
	reg old_download;

	if (reset) tape_size <= 0;
	
	if(ioctl_index[4:0] && !ioctl_index[7] && !ioctl_index[5]) begin //ioctl_index[7:5] = 'b0X0
		if (ioctl_wr) begin
         tape_ram[ioctl_addr[13:0]] <= ioctl_dout;
			tape_size <= ioctl_addr[13:0] + 14'd1;
			tape_type <= ioctl_index[6];              // 1 = .p / 0 = .o
      end
		
//		old_download <= ioctl_download;
//		if(old_download && ~ioctl_download) begin
//			tape_size <= ioctl_addr[13:0];
//			tape_type <= ioctl_index[6];              // 1 = .p / 0 = .o
//		end
	end
end

always @(posedge clk_sys) tape_in_byte <= tape_ram[tape_addr];

always @(posedge clk_sys) begin  :block4_tape_loader
	reg old_nM1;

	old_nM1 <= nM1;
	tapewrite_we <= 0;

//`ifdef DEBUG   
//   if (reset) begin
//      tape_loader_patch[0] <= 8'haf; //xor a
//      tape_loader_patch[1] <= 8'h00; //loop: nop -> cambiado por scf al final de la cinta
//      tape_loader_patch[2] <= 8'h30; //jr nc #-2 (loop)
//      tape_loader_patch[3] <= 8'hfd; //
//      tape_loader_patch[4] <= 8'hc3; //jp 0207 // 0203
//      tape_loader_patch[5] <= 8'h07; //
//      tape_loader_patch[6] <= 8'h02; //
//   end
//`endif
	
	if (~nM1 & old_nM1) begin
		if (zx81) begin
			if (addr == 16'h0347 && tape_ready) begin
				tape_loader_patch[1] <= 8'h00; //nop
				tape_loader_patch[5] <= 8'h07; //0207h
				tape_addr <= 14'h0;
				tapeloader <= 1;
            if (tape_type) ram_tape_addr <= 16'h4008;
            else ram_tape_addr <= 16'h3FFF;            
			end
			if (addr >= 16'h03c3 || addr < 16'h0347) begin
				tapeloader <= 0;
			end
		end else begin
			if (addr == 16'h0207 && tape_ready) begin
				tape_loader_patch[1] <= 8'h00; //nop
				tape_loader_patch[5] <= 8'h03; //0203h
				tape_addr <= 14'h0;
				tapeloader <= 1;
            if (tape_type) ram_tape_addr <= 16'h4008;
            else ram_tape_addr <= 16'h3FFF;            
			end
			if (addr >= 16'h024d || addr < 16'h0207) begin
				tapeloader <= 0;
			end
		end

	end

	if (tapeloader & ce_cpu_p) begin
		if (tape_addr < tape_size) begin
			tape_addr <= tape_addr + 14'd1;
         ram_tape_addr <= ram_tape_addr + 16'd1;
			tape_in_byte_r <= tape_in_byte;
			tapewrite_we <= 1;
		end else begin
         tapewrite_we <= 0;
			tape_loader_patch[1] <= 8'h37; //scf
		end
	end
end

////////////////////  VIDEO //////////////////////
// Based on the schematic:
// http://searle.hostei.com/grant/zx80/zx80.html

// character generation
wire      nopgen = addr[15] & ~mem_out[6] & nHALT;
wire      data_latch_enable = nRFSH & ce_cpu_n & ~nMREQ; //Mist
//wire      data_latch_enable = nRFSH & ce_cpu_p & ~nMREQ; //Mister
reg [7:0] ram_data_latch;
reg       nopgen_store;
reg [2:0] row_counter;
wire      shifter_start = nMREQ & nopgen_store & ce_cpu_p & shifter_en & (~zx81 | ~NMIlatch);
reg [7:0] shifter_reg;
//wire      video_out = (~status[7] ^ shifter_reg[7] ^ inverse) & !back_porch_counter & csync; //Mist
reg       inverse;
wire      video_out = (~status[7] ^ shifter_reg[7] ^ inverse); //Inverse video:Off,On;  //Mister
reg [7:0] paper_reg;
wire      border = ~paper_reg[7];
reg [7:0] attr, attr_latch;
reg       shifter_en;
reg [9:0] row_number;

reg[4:0]  back_porch_counter = 1;

always @(posedge clk_sys) begin  :block5
	reg old_csync, old_vsync;
	reg old_hsync, old_hblank;
	reg old_shifter_start;

	if (ce_6m5) begin
      old_vsync <= vsync;
		old_hsync <= hsync;
		old_csync <= csync;

		if (data_latch_enable) begin
			ram_data_latch <= mem_out;
			nopgen_store <= nopgen;
			attr_latch <= ch81_out;
		end

		if (nMREQ & ce_cpu_p) inverse <= 0;

      shifter_reg <= { shifter_reg[6:0], 1'b0 };
		old_shifter_start <= shifter_start;
		paper_reg   <= { paper_reg[6:0], 1'b0 };
		
		if (~old_shifter_start & shifter_start) begin
			shifter_reg <= (~nM1 & nopgen) ? 8'h0 : mem_out;
			inverse <= ram_data_latch[7];
			paper_reg <= 'hFF;
			attr <= ch81_dat[4] ? attr_latch : ch81_out;
		end 

		//if (old_csync & ~csync) row_counter <= row_counter + 1'd1; //Mist
		if (~old_hsync & hsync) row_counter <= row_counter + 1'd1; //Mister
      //if (~hsync & hsync2) row_number <= row_number + 1'd1;

		
      `ifdef ZX1
         if (~vsync) row_counter <= 3'h0;	
      `else
         if (~old_vsync) row_counter <= 3'h0;	
      `endif
      //if (~vsync) row_counter <= 0;	//Mist
		//if (vs) row_counter <= 0;  //Mister

		if (~old_csync & csync) back_porch_counter <= 1;
   		if (back_porch_counter) back_porch_counter <= back_porch_counter + 1'd1;

		//extended suppress to reduce garbage
		old_hblank <= hblank;
		//if(~old_hblank & hblank) shifter_en <= 0;
		//if(old_hblank & ~hblank) shifter_en <= ~NMIlatch;
      shifter_en <= 1'b1;
	end
   //if ( oldvs && !vs ) row_number <= 0;

end

//// vsync generator Mister
//reg vsync; // cleaned version
//reg vs, oldvs2, vs2;    // momentary version, sometimes used for row_counter reset trick
//always @(posedge clk_sys) begin
//   if (reset) vs <= 0;
//
//	if (~nIORQ & ~nWR & ~NMIlatch) vs <= 0;
//	if (~kbd_n & ~NMIlatch)        vs <= 1;
//   //if (zx81) begin
//      if (row_number == 10'd314 )  vs2 <= 0;
//      if (vs || row_number == 10'd308 )   vs2 <= 1;
//   //end 
//   //else vs2 <= 0;
//
//	if(!hsync) vsync<=vs;
//end

// vsync generator Mist
reg vs, oldvs2, vs2, ic18,ic19_1,ic19_2;    // momentary version, sometimes used for row_counter reset trick
wire vsync = vs;
wire csync = vsync & hsync;
always @(posedge clk_sys) begin :vsync_gen_block

	reg old_nM1;
	old_nM1 <= nM1;
   
   if (reset) vs <= 0;
   
   //if (~(nIORQ | nWR) & (~zx81 | ~NMIlatch)) vs <= 1; // stop vsync - any OUT
   //if (~kbd_n & (~zx81 | ~NMIlatch))         vs <= 0; // start vsync - keyboard IN
   if (~(nIORQ | nWR) & ~NMIlatch) vs <= 1; // stop vsync - any OUT
   if (~kbd_n & ~NMIlatch)         vs <= 0; // start vsync - keyboard IN

   //if (zx81) begin
      if (row_number == 10'd314 )  vs2 <= 0;
      if (vs || row_number == 10'd308 )   vs2 <= 1;
   //end 
   //else vs2 <= 0;

	//if(!hsync) vsync<=vs;

	if (~nIORQ) ic18 <= 1;  // if IORQ - preset HSYNC start
	if (~ic19_2) ic18 <= 0; // if sync active - preset sync end

	// And 2 M1 later the presetted sync arrives at the csync pin
	if (old_nM1 & ~nM1) begin
		ic19_1 <= ~ic18;
		ic19_2 <= ic19_1;
	end
	if (~vs) ic19_2 <= 0; //vsync keeps csync low
end

//// ZX81 upgrade
//// http://searle.hostei.com/grant/zx80/zx80nmi.html
//wire nWAIT = ~nHALT | nNMI | (slow_mode & |status[18:17]); //Slow mode speed:Original,NoWait,x2,x8;
//wire nNMI = ~NMIlatch | ~hsync;

reg slow_mode = 0;
always @(posedge clk_sys) begin  :block6
	reg [6:0] fcnt;
	reg old_halt, old_latch;

	old_latch <= NMIlatch;
	old_halt  <= nHALT;

	// Time out to enable turbo modes after reset,
	// otherwise ZX81 FW won't enter slow mode!
	if(~old_latch & NMIlatch) begin
		if(&fcnt) slow_mode <= 1;
		else fcnt <= fcnt + 1'd1;
	end
	if(old_halt & ~nHALT) slow_mode <= 0;
	
	if(reset) {fcnt,slow_mode} <= 0;
end

reg [7:0] sync_counter;
reg       NMIlatch;
reg       hsync;
always @(posedge clk_sys) begin
	if (reset) begin
      sync_counter <= 8'h00;
      hsync <= 1'b0;
   end
   else if(ce_3m25) begin
		sync_counter <= sync_counter + 1'd1;
		if(sync_counter == 8'd206 | (~nM1 & ~nIORQ)) sync_counter <= 0;
		if(sync_counter == 8'd15)  hsync <= 1;
		if(sync_counter == 8'd31)  hsync <= 0;
	end

	if (~nM1 & ~nIORQ) {hsync,sync_counter} <= 2'b00;

   if (zx81) begin
      if (~nIORQ & ~nWR & (addr[0] ^ addr[1])) NMIlatch <= addr[1];
   end
   else begin
      NMIlatch <= 0;
   end

end

//re-sync
reg hsync2, vsync2;
reg hblank, vblank;
always @(posedge clk_sys) begin :block7
	reg [8:0] cnt;
	reg [4:0] vreg;
	reg       old_hsync;
   if (reset) begin
      cnt  <= 9'h000;
      vreg <= 5'h00;
      old_hsync <= 1'b0;
   end
	else if(ce_6m5) begin
		cnt <= cnt + 1'd1;
		if(cnt == 413) cnt <= 0;

		if(cnt == 0)   hsync2 <= 1;
		if(cnt == 32)  hsync2 <= 0;

		//if(cnt == 400) hblank <= 1;
      if(cnt == 400) hblank <= 1;
		if(cnt == 72 )  hblank <= 0;

//		if(cnt == 381)   hsync2 <= 1;
//		if(cnt == 0)  hsync2 <= 0;
//		if(cnt == 368) hblank <= 1;
//		if(cnt == 40)  hblank <= 0;

//		if(cnt == 400)   hsync2 <= 1;
//		if(cnt == 19)    hsync2 <= 0;
//		if(cnt == 387)   hblank <= 1;
//		if(cnt == 59)    hblank <= 0;


		old_hsync <= hsync;
		if(~old_hsync & hsync) begin
			vreg <= {vreg[3:0], vsync}; //HE CAMBIADO ~VSYNC
			vblank <= |{vreg,vsync}; //HE CAMBIADO ~VSYNC
			vsync2 <= vreg[2];
			if(&vreg[3:2]) cnt <= 0;
		end
	end
end

//wire i,g,r,b;
reg i,g,r,b;
always @* begin
	casex({status[5] & border, ch81_dat[5], border, video_out}) //Black border:Off,On;
		'b1XXX: {i,g,r,b} <= 0;
		'b00XX: {i,g,r,b} <= {4{video_out}};
		'b011X: {i,g,r,b} <= ch81_dat[3:0];
		'b0101: {i,g,r,b} <= attr[7:4];
		'b0100: {i,g,r,b} <= attr[3:0];
	endcase
end

wire [1:0] scale = status[13:12]; //Scandoubler Fx:None,HQ2x,CRT 25%,CRT 50%;
//assign VGA_SL = scale ? scale - 1'd1 : 2'd0;
//assign VGA_F1 = 0;

reg VSync, HSync;
always @(posedge CLK_VIDEO) begin
	HSync <= hsync2;
   oldvs2 <= vs2;
	if(~HSync & hsync2) VSync <= vsync2;
   //if(~HSync & hsync2) VSync <= vsync2 || ( vs2 && zx81 );
   if(HSync & ~hsync2) row_number <= row_number + 1'd1;
   if( !oldvs2 && vs2 ) row_number <= 10'd308;
   if( oldvs2 && !vs2 ) row_number <= 10'd0;
end
 
//video_mixer #(400,1,1) video_mixer
video_mixer #(400,1) video_mixer
(
	.CLK_VIDEO(CLK_VIDEO),
   .CE_PIXEL(CE_PIXEL),
   
	.ce_pix(ce_6m5),
	//.scandoubler(scale || forced_scandoubler), //testing
	//.hq2x(scale == 1),
   .scandoubler( forced_scandoubler ),

	.VGA_DE(vga_de),
	.R({r,{3{i & r}}}),
	.G({g,{3{i & g}}}),
	.B({b,{3{i & b}}}),
   
   .HSync(~HSync),
   .VSync(VSync),

	.HBlank(hblank),
	.VBlank(vblank),

   //ZPUFlex OSD
   .osd_window(osd_window),
   .osd_pixel(osd_pixel),
   .osd_bkgr(osd_bkgr),

   .VGA_R(VGA_R),
   .VGA_G(VGA_G),
   .VGA_B(VGA_B),
   .VGA_VS(vs_aux),
   .VGA_HS(hs_aux)

);


assign VGA_VS = forced_scandoubler ? vs_aux : 1'b1;
assign VGA_HS = forced_scandoubler ? hs_aux : ~(hs_aux ^ vs_aux);
assign CLK_VIDEO = clk_sys;

//////////////////// CHROMA81 ////////////////////
// mapped at C000 and accessible only in non-M1 cycle
//wire      ch81_e = status[20] & ~nMREQ & nM1 & &addr[15:14];
wire      ch81_sel = ~nIORQ & (addr == 'h7FEF) & status[20]; //CHROMA81:Disabled,Enabled;
reg [7:0] ch81_dat = 0;

always @(posedge clk_sys) begin :block8
	reg set_m0; // = 0;
	reg old_tapeloader; // = 0;

	if(reset | ~status[20]) {set_m0, ch81_dat} <= 0; //CHROMA81:Disabled,Enabled;
	else if(ch81_sel & ~nWR) ch81_dat <= cpu_dout;
	
	if(ioctl_wr) begin
		if(ioctl_index[4:0] && (ioctl_index[7:5]==1)) begin // ioctl_index[7:5] = 'b001 -> Crhoma81
			set_m0 <= 1;
			if(ioctl_addr == 1024) ch81_dat <= ioctl_dout[3:0];
		end
		else if(~ioctl_index[5]) set_m0 <= 0;
	end
	
	old_tapeloader <= tapeloader;
	if(old_tapeloader & ~tapeloader & set_m0 & status[20]) ch81_dat[5:4] <= 2'b10; //CHROMA81:Disabled,Enabled;
end

`ifndef ZXD
reg [7:0]  ram_out;
reg [7:0] ch81_out;  //assign ch81_out = 0;

wire [13:0] ch81_addr  = nRFSH ? addr[13:0] : {ram_data_latch[7], rom_a[8:0]};
wire        ch81_wrena = ~nWR & ~nMREQ & ch81_e;
reg sram_cycle = 1'b0;

always @(posedge clk_sys) begin
   sram_cycle <= ~sram_cycle;
   if (sram_cycle || tapeloader) begin 
      ram_in   <= tapeloader ? tape_in_byte_r : cpu_dout;
      ram_addr <= {5'd0, ram_a};
      ram_we_n <= !wren_a;
      if (ram_we_n) ch81_out <= sram_data;
   end else begin
      if (ram_we_n) ram_out = sram_data;
      ram_addr <= { 7'b0000100, ch81_addr };
      ram_we_n <= ~ch81_wrena;
      ram_in   <= cpu_dout;
   end
end

`else
wire [7:0] ch81_out;  //assign ch81_out = 0;
dpram #(.ADDRWIDTH(14)) chroma81
(
	.clock(clk_sys),
	.address_a(nRFSH ? addr[13:0] : {ram_data_latch[7], rom_a[8:0]}),
	.wren_a(~nWR & ~nMREQ & ch81_e),
	.data_a(cpu_dout),
	.q_a(ch81_out),
	
	.address_b(ioctl_addr[13:0]),
	.data_b(ioctl_dout),
	.wren_b(ioctl_wr && ioctl_index[4:0] && (ioctl_index[7:5]==1) && !ioctl_addr[24:10])
);
`endif

//////////////////// QS CHRS /////////////////////
wire       qs_e = nRFSH ? (addr[15:10] == 'b100001) : (qs & (addr[15:9] == 'b0001111)); //8400-87FF / 1E00-1F00
wire [7:0] qs_out;

dpram #(.ADDRWIDTH(10)) qschrs
(
	.clock(clk_sys),
	.address_a(nRFSH ? addr[9:0] : {ram_data_latch[7], rom_a[8:0]}),
	.wren_a(~nWR & ~nMREQ & qs_e),
	.data_a(cpu_dout),
	.q_a(qs_out),
	
	.address_b(ioctl_addr[9:0]),
	.wren_b(ioctl_wr && ioctl_index[4:0] && (ioctl_index[7:5]==3) && !ioctl_addr[24:10]), // 'b011 QS CHAR
	.data_b(ioctl_dout)
);


reg qs = 0;
always @(posedge clk_sys) begin :block9_qschar
	reg qs_set; // = 0;
	reg old_f1;
	reg old_tapeloader; // = 0;
	
	old_f1 <= Fn[1];
	if(~old_f1 & Fn[1]) qs <= ~qs;
	
	if(ioctl_wr) begin
		if(ioctl_index[4:0] && (ioctl_index[7:5]==3)) qs_set <= 1; // ioctl_index[7:5] 'b011 QS CHAR
		else if(~ioctl_index[5]) qs_set <= 0;
	end
	
	old_tapeloader <= tapeloader;
	if(old_tapeloader & ~tapeloader & qs_set) qs <= 1;

	if(reset) {qs_set,qs} <= 0;
end

////////////////////  SOUND //////////////////////
wire [7:0] psg_out;
wire       psg_sel = ~nIORQ & &addr[3:0]; //xF
wire [7:0] psg_ch_a, psg_ch_b, psg_ch_c;

ym2149 psg
(
	.CLK(clk_sys),
	.CE(ce_psg),
	.RESET(reset),
	.BDIR(psg_sel & ~nWR),
	.BC(psg_sel & (&addr[7:6] ^ nWR)),
	.DI(cpu_dout),
	.DO(psg_out),
	.CHANNEL_A(psg_ch_a),
	.CHANNEL_B(psg_ch_b),
	.CHANNEL_C(psg_ch_c)
);

// Route vsync through a high-pass filter to filter out sync signals from the
// tape audio
wire [7:0] mic_out;
wire       mic_bit = mic_out > 8'd8 && mic_out < 8'd224;
reg audiomicon_r;

rc_filter_1o #(
	.R_ohms_g(33000),
	.C_p_farads_g(47000),
	.fclk_hz_g(6500000),
	.cwidth_g(18)) mic_filter
(
	.clk_i(clk_sys),
	.clken_i(ce_6m5),
	.res_i(reset),
	.din_i({1'b0, vsync && audiomicon_r, 6'd0 }),
	.dout_o(mic_out)
);

wire [8:0] audio_l = { 1'b0, psg_ch_a } + { 1'b0, psg_ch_c } + { 3'd0, mic_bit, 4'd0 };
wire [8:0] audio_r = { 1'b0, psg_ch_b } + { 1'b0, psg_ch_c } + { 3'd0, mic_bit, 4'd0 };

reg tape_data;
reg audiotapeon_r; 

sigma_delta_dac #(7) dac_l
(
	.CLK(clk_sys),
	.RESET(reset),
	.DACin( audio_l[8:1] | ( 7'b1111111 & (tape_data && audiotapeon_r))),
	.DACout(AUDIO_L)
);

sigma_delta_dac #(7) dac_r
(
	.CLK(clk_sys),
	.RESET(reset),
	.DACin(audio_r[8:1] | ( 7'b1111111 & (tape_data && audiotapeon_r))),
	.DACout(AUDIO_R)
);

always @(posedge clk_sys) begin :audio_tape_reg
	reg f10key_r;
   f10key_r <= Fn[10];
   if (!locked) begin
      audiotapeon_r   <= 1'b0;
   end
   else if ( f10key_r == 1'b1 && Fn[10] == 1'b0) audiotapeon_r <= ~audiotapeon_r;
end

always @(posedge clk_sys) begin :audio_mic_reg
	reg f9key_r;
   f9key_r <= Fn[9];
   if (!locked) begin
      audiomicon_r   <= 1'b1;
   end
   else if ( f9key_r == 1'b1 && Fn[9] == 1'b0) audiomicon_r <= ~audiomicon_r;
end

//assign AUDIO_L   = {audio_l, 6'd0};
//assign AUDIO_R   = {audio_r, 6'd0};
//assign AUDIO_S   = 0;
//assign AUDIO_MIX = status[3:2]; //Stereo mix:none,25%,50%,100%;
//assign AUDIO_L   = psg_ch_a[7];
//assign AUDIO_R   = psg_ch_c[7];


////////////////////   HID   /////////////////////

wire kbd_n = nIORQ | nRD | addr[0];
 
wire [11:1] Fn;
wire  [2:0] mod;
wire  [4:0] key_data;

keyboard kbd(
   .reset(reset),
   .clk_sys(clk_sys),
   
   .ps2_key(ps2_key),
   .addr(addr),
   .key_data(key_data),
   .Fn(Fn),   //F11-F1
   .mod(mod)  //CTRL, ALT, SHIFT
);

wire [1:0] jsel = status[9:8]; //Joystick:Cursor,Sinclair,ZX81;
wire [4:0] joy = joystick_0 | joystick_1;

//ZX81 67890
wire [4:0] joyzx = ({5{jsel[1]}} & {joy[2], joy[3], joy[0], joy[1], joy[4]});

//Sinclair 1 67890
wire [4:0] joys1 = ({5{jsel[0]}} & {joy[1:0], joy[2], joy[3], joy[4]});

//Cursor 56780
wire [4:0] joyc1 = {5{!jsel}} & {joy[2], joy[3], joy[0], 1'b0, joy[4]};
wire [4:0] joyc2 = {5{!jsel}} & {joy[1], 4'b0000};

//map to keyboard
wire [4:0] joy_kbd = {5{zxp_use}} | (({5{addr[12]}} | ~(joys1 | joyc1 | joyzx)) & ({5{addr[11]}} | ~joyc2));


reg [7:0] zxp_out = 'hFF;
reg       zxp_use = 0;
wire      zxp_sel = ~nIORQ & (addr == 'hE007);

always @(posedge clk_sys) begin
	if(reset) {zxp_use,zxp_out} <= 'hFF;
	else if(zxp_sel & ~nWR) begin
		zxp_out <= 'hFF;
		if(cpu_dout == 'hAA) zxp_out <= 'hF0;
		if(cpu_dout == 'h55) zxp_out <= 'h0F;
		if(cpu_dout == 'hA0) {zxp_use,zxp_out} <= {1'b1, ~joy[3:0],~joy[4],3'b000};
	end
end

// Loading via EAR
// cuando se lee desde cinta, la imagen se apaga, es normal.
// en un ZX81 real, la imagen se distorsiona, pero no se apaga
// estas lineas sirven para reducir el numero de lecturas de bits
// desde la cinta, y evitar que se sature el EAR del ZX81
// la he cogido de uno de los cores que he estudiado

reg tape_in_z1;
reg tape_in_z2;
reg tape_in_z3;
//wire tape_in;
reg tape_led;

always @(posedge clk_sys) begin
//   if ( ce_6m5 ) begin
      tape_in_z3 <= tape_in_z2;
      tape_in_z2 <= tape_in_z1;
      tape_in_z1 <= AUDIO_EAR;
// test past  - ok with galaxians 
      if ((tape_in_z2==1'b0 && tape_in_z1==1'b1 && AUDIO_EAR==1'b0) 
         || (tape_in_z2==1'b1 && tape_in_z1==1'b0 && AUDIO_EAR==1'b1))
         tape_data <= AUDIO_EAR;
      else 
         tape_data <= tape_in_z1;
      //tape_data <= tape_in_z2;

//// test past - mal
//      if ((tape_in_z2==1'b1 && tape_in_z1==1'b1 && AUDIO_EAR==1'b0) 
//         || (tape_in_z2==1'b0 && tape_in_z1==1'b0 && AUDIO_EAR==1'b1))
//         tape_data <= AUDIO_EAR;
//      else 
//         tape_data <= tape_in_z1;
   
////test past - mal
//   tape_data <= tape_in_z1;

////test past - mal
//   tape_data <= tape_in_z3;

////   end
   
   if (ce_psg) tape_led <= tape_data && tape_in_z1 && tape_in_z2;
end

// Emitir sonido cinta a CPU y audio desde entrada EAR
assign tape_in = (tapeloader)? 1'b0 : ~tape_data;
//assign tape_in = (tapeloader)? 1'b0 : ~AUDIO_EAR;
//assign tape_led = tape_data &&  tape_in_z1 && tape_in_z2;

endmodule

