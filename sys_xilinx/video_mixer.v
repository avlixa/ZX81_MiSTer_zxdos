//
//
// Copyright (c) 2017,2021 Alexey Melnikov
//
// This program is GPL Licensed. See COPYING for the full license.
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

//
// LINE_LENGTH: Length of  display line in pixels
//              Usually it's length from HSync to HSync.
//              May be less if line_start is used.
//
// HALF_DEPTH:  If =1 then color dept is 4 bits per component
//
// altera message_off 10720 
// altera message_off 12161

module video_mixer
#(
	parameter LINE_LENGTH  = 768,
	parameter HALF_DEPTH   = 0
)
(
	input            CLK_VIDEO, // should be multiple by (ce_pix*4)
	output reg       CE_PIXEL,  // output pixel clock enable

	input            ce_pix,    // input pixel clock or clock_enable

	input            scandoubler,

	// color
	input [DWIDTH:0] R,
	input [DWIDTH:0] G,
	input [DWIDTH:0] B,

	// Positive pulses.
	input            HSync,
	input            VSync,
	input            HBlank,
	input            VBlank,
   
   //ZPUFlex OSD
   input wire   osd_window,
   input wire   osd_pixel,
   input wire [2:0]  osd_bkgr,

	// video output signals
`ifndef ZX1
	output wire [5:0] VGA_R,
	output wire [5:0] VGA_G,
	output wire [5:0] VGA_B,
`else
	output wire [2:0] VGA_R,
	output wire [2:0] VGA_G,
	output wire [2:0] VGA_B,
	//output wire       PAL,
	//output wire       NTSC,
`endif
	output reg       VGA_VS,
	output reg       VGA_HS,
	output reg       VGA_DE
);

   localparam DWIDTH = HALF_DEPTH ? 3 : 5;
   localparam DWIDTH_SD =  DWIDTH;
   localparam HALF_DEPTH_SD =  HALF_DEPTH;


   wire [DWIDTH:0] R_in  = R;
   wire [DWIDTH:0] G_in  = G;
   wire [DWIDTH:0] B_in  = B;

   wire hs_g, vs_g;
   wire hb_g, vb_g;
   wire [DWIDTH_SD:0] R_gamma, G_gamma, B_gamma;

   generate
      assign {R_gamma,G_gamma,B_gamma} = {R_in,G_in,B_in};
      assign {hs_g, vs_g, hb_g, vb_g} = {HSync, VSync, HBlank, VBlank};
   endgenerate

   wire [DWIDTH_SD:0] R_sd;
   wire [DWIDTH_SD:0] G_sd;
   wire [DWIDTH_SD:0] B_sd;
   wire hs_sd, vs_sd, hb_sd, vb_sd, ce_pix_sd;

   scandoubler #(.LENGTH(LINE_LENGTH), .HALF_DEPTH(HALF_DEPTH_SD)) sd
   (
      .clk_vid(CLK_VIDEO),
      .hq2x(1'b0),

      .ce_pix(ce_pix),
      .hs_in(hs_g),
      .vs_in(vs_g),
      .hb_in(hb_g),
      .vb_in(vb_g),
      .r_in(R_gamma),
      .g_in(G_gamma),
      .b_in(B_gamma),

      .ce_pix_out(ce_pix_sd),
      .hs_out(hs_sd),
      .vs_out(vs_sd),
      .hb_out(hb_sd),
      .vb_out(vb_sd),
      .r_out(R_sd),
      .g_out(G_sd),
      .b_out(B_sd)
   );

   wire [DWIDTH_SD:0] rt = (scandoubler ? R_sd : R_gamma);
   wire [DWIDTH_SD:0] gt = (scandoubler ? G_sd : G_gamma);
   wire [DWIDTH_SD:0] bt = (scandoubler ? B_sd : B_gamma);

   reg [7:0] vga_red_i, vga_green_i, vga_blue_i;

   always @(posedge CLK_VIDEO) begin :block_video
      reg [5:0] r,g,b;
      reg hde,vde,hs,vs, old_vs;
      reg old_hde;
      reg old_ce;
      reg ce_osc, fs_osc;
      
      old_ce <= ce_pix;
      ce_osc <= ce_osc | (old_ce ^ ce_pix);

      old_vs <= vs;
      if(~old_vs & vs) begin
         fs_osc <= ce_osc;
         ce_osc <= 0;
      end

      CE_PIXEL <= scandoubler ? ce_pix_sd : fs_osc ? (~old_ce & ce_pix) : ce_pix;

      if( HALF_DEPTH) begin
         r <= {rt,rt[DWIDTH_SD:DWIDTH_SD-1]};
         g <= {gt,gt[DWIDTH_SD:DWIDTH_SD-1]};
         b <= {bt,bt[DWIDTH_SD:DWIDTH_SD-1]};
      end
      else begin
         r <= rt;
         g <= gt;
         b <= bt;
      end

      hde <= scandoubler ? ~hb_sd : ~hb_g;
      vde <= scandoubler ?  vb_sd :  vb_g;
      vs  <= scandoubler ?  vs_sd :  vs_g;
      hs  <= scandoubler ?  hs_sd :  hs_g;

      if(CE_PIXEL) begin
         vga_red_i   <= (!hde || !vde) ? 0 : {r,2'b0};
         vga_green_i <= (!hde || !vde) ? 0 : {g,2'b0};
         vga_blue_i  <= (!hde || !vde) ? 0 : {b,2'b0};

         VGA_VS <= vs;
         VGA_HS <= hs;

         old_hde <= hde;
         if(old_hde ^ hde) VGA_DE <= vde & hde;
      end
   end
   

   // ZPUFLEX OSD Overlay
   wire [7:0] vga_red_o, vga_green_o, vga_blue_o;

   OSD_Overlay ZPUFlex_overlay (
      .clk(CLK_VIDEO),
      .red_in(vga_red_i),
      .green_in(vga_green_i),
      .blue_in(vga_blue_i),
      .window_in(1'b1),
      .osd_window_in(osd_window),
      .osd_pixel_in(osd_pixel),
      .osd_bkgr_in(osd_bkgr),      
      .hsync_in(hs),
      .red_out(vga_red_o),
      .green_out(vga_green_o),
      .blue_out(vga_blue_o),
      .window_out( ),
      .scanline_ena(1'b0)
   );   

`ifndef ZX1   
   assign VGA_R = vga_red_o[7:2];
   assign VGA_G = vga_green_o[7:2];
   assign VGA_B = vga_blue_o[7:2];
`else
   assign VGA_R = vga_red_o[7:5];
   assign VGA_G = vga_green_o[7:5];
   assign VGA_B = vga_blue_o[7:5];
`endif   
endmodule

