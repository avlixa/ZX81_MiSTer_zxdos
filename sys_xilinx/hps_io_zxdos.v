//
// hps_io_zxdos.v
//
// Copyright (c) 2021 AvlixA
//
// This source file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////
// Substitute to Mister's hps_io.v module for zxdos boards
//
module hps_io //#(parameter STRLEN=0, PS2DIV=0, WIDE=0, VDNUM=1, PS2WE=0)
(
	input wire        clk_sys,
   input wire        reset,
   input wire        p_o_reset_n,
   
`ifndef ZX1
   // Joystick board signal
   input  wire joy_data,
   output wire joy_clk,
   output wire joy_load_n,
   input  wire hsync_n_s,
`else
   input  wire [6:0] joy1_i,
   input  wire [6:0] joy2_i,
`endif 
   
	// buttons up to 32
	output wire [31:0] joystick_0,
	output wire [31:0] joystick_1,
	
	output wire [1:0] buttons,
	output wire       forced_scandoubler,

	output wire [31:0] status,
	input wire  [63:0] status_in,
	input wire         status_set,
	input wire  [15:0] status_menumask,

	input wire         info_req,
	input wire   [7:0] info,

   // ARM -> FPGA download
	output wire        ioctl_download, // signal indicating an active download
	output wire [15:0] ioctl_index,        // menu index used to upload the file
	output wire        ioctl_wr,
	output wire [26:0] ioctl_addr,         // in WIDE mode address will be incremented by 2
	output wire [DW:0] ioctl_dout,
//	output reg        ioctl_upload = 0,   // signal indicating an active upload
//	input      [DW:0] ioctl_din,
//	output reg        ioctl_rd,
//	output reg [31:0] ioctl_file_ext,
//	input             ioctl_wait,

   output wire host_led1,
   output wire host_led2,

	// ps2 keyboard 
//	output            ps2_kbd_clk_out,
//	output            ps2_kbd_data_out,
	input wire        ps2_kbd_clk_in,
	input wire        ps2_kbd_data_in,

//	output            ps2_mouse_clk_out,
//	output            ps2_mouse_data_out,
//	input             ps2_mouse_clk_in,
//	input             ps2_mouse_data_in,

	// ps2 alternative interface.

	// [8] - extended, [9] - pressed, [10] - toggles with every press/release
	output wire [10:0] ps2_key,

	// [24] - toggles with every event
	output wire [24:0] ps2_mouse, 
//	output reg [15:0] ps2_mouse_ext = 0, // 15:8 - reserved(additional buttons), 7:0 - wheel movements

   //ZPUFlex signals
   output wire [2:0] osd_bkgr,
   output wire       osd_window, 
   output wire       osd_pixel,
   input wire        vga_hsync_i,
   input wire        vga_vsync_i,
   output wire       spi_clk,
   output wire       spi_mosi,
   input wire        spi_miso,
   output wire       spi_cs

);
   localparam DW = 7;

   reg [1:0] ramtype = 2'b0;
   reg videomode = 1'b1;

   //zxdos inputs from HPS not available
   assign joystick_0[31:5] = 27'b0; 
   assign joystick_1[31:5] = 27'b0; 
   assign joystick_0[4:0] = { !joy1_s[4],!joy1_s[0],!joy1_s[1],!joy1_s[2],!joy1_s[3]} | {5{host_divert_keyboard}};
   assign joystick_1[4:0] = { !joy2_s[4],!joy2_s[0],!joy2_s[1],!joy2_s[2],!joy2_s[3]} | {5{host_divert_keyboard}};
   assign buttons = 0;
   assign status[31:21] = 0;
   assign status[20] = dswitch[2]; //CHROMA81: Disabled/Enabled
   assign status[19] = dswitch[1]; //QS CHRS
   assign status[18:17] = dswitch[18:17];  //Slow mode speed: 00 - Original, 01 - NoWait, 10 - x2, 11 - x8
   assign status[16] = dswitch[0];  //Low RAM: Off/8KB
   assign status[15:14] = dswitch[16:15];  //CHR$128/UDG: 00 - 128 Chars, 01 - 64 Chars, 10 - Disabled
   assign status[13:12] = 0;
   //assign status[11:10] = ramtype; //Main RAM:16KB,32KB,48KB,1KB;
   assign status[11:10] = dswitch[12:11]; //Main RAM:16KB,32KB,48KB,1KB;
   assign status[9:8] = dswitch[14:13];  //Joystick: 00 - Cursor, 01 - Sinclair, 10 - ZX81
   assign status[7] = dswitch[3];  //Inverse video: Off/On
   assign status[6] = dswitch[5];  //Video frequency:50Hz/60Hz;
   assign status[5] = dswitch[4];  //Black border: Off/On
   assign status[4] = dswitch[10]; //Model: 0 - ZX81 / 1: ZX80
   assign status[3:1] = 0;
   assign status[0] = softreset | !host_reset_n;

		//dswitch 
		//	bit 0: Low RAM: Off/8KB
		//	bit 1: QS CHRS: off/on
		//	bit 2: CHROMA81: Disabled/Enabled
		// bit 3: Inverse video: Off/On
		// bit 4: Black border: Off/On
		// bit 5: Video frequency:50Hz/60Hz;
		//
      //  bit 9-7: Downloading File type: 000 - rom, 001 - .p, 010 - .o, 
		//	bit 10: Model: 0 - ZX81 / 1: ZX80
		// bit 12-11: Main RAM: 00 - 16KB, 01 - 32KB, 10 - 48KB, 11 - 1KB
		// bit 14-13: Joystick: 00 - Cursor, 01 - Sinclair, 10 - ZX81
		// bit 16-15: CHR$128/UDG: 00 - 128 Chars, 01 - 64 Chars, 10 - Disabled
		// bit 18-17: Slow mode speed: 00 - Original, 01 - NoWait, 10 - x2, 11 - x8
 
   assign forced_scandoubler = videomode;
//   assign ioctl_wr = 0;
//   assign ioctl_addr = 0;
//   assign ioctl_dout = 0;
//   assign ioctl_download = 0;
//   assign ioctl_index = 0;
   assign ps2_mouse = 0;

   //ps2 keyboard ****************************************************
   reg keytoggle, ctrlkey, altkey, softreset, hardreset;
   wire hardreset_ZPU, videokey_ZPU;
   reg f2key, f2key_prev, videokey, videokey_prev;
   wire kbint, released, extended;
   wire [7:0] scancode;
   wire host_ps2_clk;
   wire host_ps2_data;
   
   ps2_port ps2keyboard (
       .clk(clk_sys),      // se recomienda 1 MHz <= clk <= 600 MHz
       .enable_rcv(1'b1),  // habilitar la maquina de estados de recepcion
       .kb_or_mouse(1'b0),  // 0: kb, 1: mouse
       .ps2clk_ext(host_ps2_clk),
       .ps2data_ext(host_ps2_data),
       .kb_interrupt(kbint),  // a 1 durante 1 clk para indicar nueva tecla recibida
       .scancode(scancode), // make o breakcode de la tecla
       .released(released),  // soltada=1, pulsada=0
       .extended(extended)  // extendida=1, no extendida=0
   );

   always @(posedge clk_sys) begin
      if (reset) begin
         keytoggle <= 1'b0;
         ctrlkey <= 1'b0;
         altkey <= 1'b0;
         f2key <= 1'b0;
         videokey <=  1'b0;
         softreset <= 1'b0;
         hardreset <= 1'b0;
      end
      if (kbint) keytoggle <= ~keytoggle;
      if (kbint & scancode == 8'h11) altkey <= !released; //ALT
      if (kbint & scancode == 8'h14) ctrlkey <= !released; //CTRL
      if (kbint & extended & scancode == 8'h71) softreset <= !released & altkey & ctrlkey; //DEL+CTRL+ALT
      if (kbint & scancode == 8'h66) hardreset <= !released & altkey & ctrlkey; //BCKSPC+CTRL+ALT
      if (kbint & scancode == 8'h06) f2key <= !released; //F2 - Memory size
      if (kbint & scancode == 8'h7e) videokey <= !released; //Scrll.lock - Memory size
    end

   assign ps2_key = {keytoggle,!released,extended,scancode};
   
   always @(posedge clk_sys) begin   
      f2key_prev <= f2key;
      videokey_prev <= videokey || videokey_ZPU;
      if (reset) begin
         f2key_prev <= 0;
         videokey_prev <= 0;
      end
      //if (f2key_prev & !f2key) ramtype <= ramtype + 1'd1;
      if (videokey_prev & !videokey) videomode <= ~videomode; //1 - VGA / 0 - RGB
   end
   
   //joysticks ****************************************************
   wire [11:0] joy1_s, joy2_s;

`ifndef ZX1   
   joydecoder #(.FRECCLKIN(52),.FRECCLKOUT(16) ) joydecoder (
      .clk(clk_sys),
      .joy_data(joy_data),
      .joy_clk(joy_clk),
      .joy_load_n(joy_load_n),
      .reset(reset),
      .hsync_n_s(hsync_n_s),

      .joy1_o(joy1_s), // -- MXYZ SACB RLDU  Negative Logic
      .joy2_o(joy2_s)  // -- MXYZ SACB RLDU  Negative Logic
   );
`else
   assign joy1_s[6:0] = joy1_i;
   assign joy1_s[7]   = 1'b1;
   assign joy2_s = 8'b11111111;
//   assign joy2_s[6:0] = joy2_i;
//   assign joy2_s[7]   = 1'b1;
`endif   

   //hardreset ****************************************************

   multiboot #(.SPIADDR(24'h0B0000)) reiniciozxdos (
      .clk_icap(clk_sys),
      .REBOOT(hardreset || hardreset_ZPU)
   );

   //ZPUFLEX Control module
  
   // Control module
   wire [18:0] dswitch;
   wire [15:0] joykeys;
   wire [8:0] joy2zpuflex; // 8: 0 - ZXDOS/1 - ZXUNO, [7:0] - Joystick (SACBRLDU)

   // Host control signals, from the Control module
   wire host_reset_n;
   wire host_divert_sdcard;
   wire host_divert_keyboard;
   wire host_pal;
   wire host_select;
   wire host_start;

   wire [31:0] host_bootdata;
   wire host_bootdata_req;
   wire host_bootdata_ack;
   wire host_bootdata_download;
   //wire host_led1;
   //wire host_led2;
   wire [15:0] host_bootdata_size;

   // Block keyboard signals from reaching the host when host_divert_keyboard is high.
   assign host_ps2_data = ps2_kbd_data_in || host_divert_keyboard;
   assign host_ps2_clk  = ps2_kbd_clk_in || host_divert_keyboard;

   //Entrada de joystick para control de zpuflex (SACBRLDU)
`ifdef ZX1
   assign joy2zpuflex = { 1'b1 , // 1 - ZXUNO, 0 - ZXDOS
`else   
   assign joy2zpuflex = { 1'b0 , // 1 - ZXUNO, 0 - ZXDOS
`endif
                          ~(joy1_s[7:0] & joy2_s[7:0]) };

   CtrlModule #( .sysclk_frequency(520) ) // Sysclk frequency * 10 
      MyCtrlModule (
         .clk(clk_sys),
         .reset_n(p_o_reset_n),

         //-- Video signals for OSD
         .vga_hsync(~vga_hsync_i),
         .vga_vsync(~vga_vsync_i),
         .osd_window(osd_window),
         .osd_pixel(osd_pixel),
         .osd_bkgr(osd_bkgr),
         
         //-- PS2 keyboard
         .ps2k_clk_in(ps2_kbd_clk_in),
         .ps2k_dat_in(ps2_kbd_data_in),
         //.ps2k_clk_out(host_ps2_clk),
         //.ps2k_dat_out(host_ps2_data),
         
         //-- SD card signals
         .spi_clk(spi_clk),   //(SPI_SCK),
         .spi_mosi(spi_mosi), //(SPI_DI),
         .spi_miso(spi_miso), //(SPI_DO),
         .spi_cs(spi_cs),     //(CONF_DATA0),
         
         //-- DIP switches
         //dipswitches(15 downto 5) => open,
         //dipswitches(4) => ,
         //dipswitches(3) => ,
         //dipswitches(2) => ,
         //dipswitches(1) => QS CHRS,
         //dipswitches(0) => ,
         .dipswitches(dswitch),
         
         //--ROM size
         .size(host_bootdata_size),
         
         //-- JOY Keystrokes
         .joykeys(joykeys),
         .hard_reset(hardreset_ZPU),
         .video_mode(videokey_ZPU),         
         //-- Joystick imput
         .joy_pins(joy2zpuflex),
         
         //-- Control signals
         .host_divert_keyboard(host_divert_keyboard),
         .host_divert_sdcard(host_divert_sdcard),
         .host_reset_n(host_reset_n),
         .host_start(host_start),
         .host_select(host_select),
         
         //-- Boot data upload signals
         .host_bootdata(host_bootdata),
         .host_bootdata_req(host_bootdata_req),
         .host_bootdata_ack(host_bootdata_ack),
         .host_bootdata_download(host_bootdata_download),
         .host_led1(host_led1),
         .host_led2(host_led2)
   );


   //Rom loader to ioctl signal
   rom_loader ioctl_romloader (
      .clk(clk_sys),
      .reset(reset),

      //ZPUFlex host data transfer
      .host_bootdata(host_bootdata),
      .host_bootdata_req(host_bootdata_req),
      .host_bootdata_download(host_bootdata_download),
      .host_bootdata_ack(host_bootdata_ack),
      .host_bootdata_size(host_bootdata_size),
      .host_file_type(dswitch[9:7]),

      // ARM -> FPGA download
      .ioctl_download(ioctl_download), // signal indicating an active download
      .ioctl_index(ioctl_index),    // menu index used to upload the file
      .ioctl_wr(ioctl_wr),
      .ioctl_addr(ioctl_addr),     // in WIDE mode address will be incremented by 2
      .ioctl_dout(ioctl_dout)

   );
   
endmodule


