// pll.v

`timescale 1 ps / 1 ps
module pll (
		input  wire  refclk,   //  refclk.clk   //50Mhz
		input  wire  rst,      //   reset.reset
		output wire  outclk_0, // outclk0.clk   //52Mhz // 52.083 ZXUNO
      output wire  outclk_1, // outclk0.clk   //156.25Mhz
		output wire  locked    //  locked.export
	);
   
   wire clk50, clk52, clk156, clkfbout, clkfbin;

   assign outclk_0 = clk52;
   assign outclk_1 = clk156;

  // Input buffering
  //------------------------------------
  IBUFG clkin1_buf
   (.O (clk50),
    .I (refclk));


  `ifndef ZX1
  // Clocking primitive for ZXDOS/ZXDOS+
  //------------------------------------

  // Instantiation of the DCM primitive
  //    * Unused inputs are tied off
  //    * Unused outputs are labeled unused
  wire        psdone_unused;
  wire        locked_int;
  wire [7:0]  status_int;
  wire clkfb;
  wire clk0;
  wire clkfx;
  DCM_SP
  #(.CLKDV_DIVIDE          (2.000),
    .CLKFX_DIVIDE          (25),
    .CLKFX_MULTIPLY        (26),
    .CLKIN_DIVIDE_BY_2     ("FALSE"),
    .CLKIN_PERIOD          (20.0),
    .CLKOUT_PHASE_SHIFT    ("NONE"),
    .CLK_FEEDBACK          ("1X"),
    .DESKEW_ADJUST         ("SYSTEM_SYNCHRONOUS"),
    .PHASE_SHIFT           (0),
    .STARTUP_WAIT          ("FALSE"))
  dcm_sp_inst
    // Input clock
   (.CLKIN                 (clk50),
    .CLKFB                 (clkfb),
    // Output clocks
    .CLK0                  (clk0),
    .CLK90                 (),
    .CLK180                (),
    .CLK270                (),
    .CLK2X                 (),
    .CLK2X180              (),
    .CLKFX                 (clkfx),
    .CLKFX180              (),
    .CLKDV                 (),
    // Ports for dynamic phase shift
    .PSCLK                 (1'b0),
    .PSEN                  (1'b0),
    .PSINCDEC              (1'b0),
    .PSDONE                (),
    // Other control and status signals
    .LOCKED                (locked_int),
    .STATUS                (status_int),
 
    .RST                   (rst),
    // Unused pin- tie low
    .DSSEN                 (1'b0));

    assign locked = locked_int;

  // Output buffering
  //-----------------------------------
  BUFG clkf_buf
   (.O (clkfb),
    .I (clk0));

  BUFG clkout1_buf
   (.O   (clk52),
    .I   (clkfx));
  
  assign clk156 = 1'b0;
    
  `else
  
  // Clocking primitive for ZXUNO
  //------------------------------------
  // Instantiation of the PLL primitive
  //    * Unused inputs are tied off
  //    * Unused outputs are labeled unused
  wire [15:0] do_unused;
  wire        drdy_unused;
  wire        clkfbout_buf;
  wire        clkout2_unused;
  wire        clkout3_unused;
  wire        clkout4_unused;
  wire        clkout5_unused;

  PLL_BASE
  #(.BANDWIDTH              ("OPTIMIZED"),
    .CLK_FEEDBACK           ("CLKFBOUT"),
    .COMPENSATION           ("SYSTEM_SYNCHRONOUS"),
    .DIVCLK_DIVIDE          (2),
    .CLKFBOUT_MULT          (25),
    .CLKFBOUT_PHASE         (0.000),
    .CLKOUT0_DIVIDE         (12),
    .CLKOUT0_PHASE          (0.000),
    .CLKOUT0_DUTY_CYCLE     (0.500),
    .CLKOUT1_DIVIDE         (4),
    .CLKOUT1_PHASE          (0.000),
    .CLKOUT1_DUTY_CYCLE     (0.500),
    .CLKIN_PERIOD           (20.000),
    .REF_JITTER             (0.010))
  pll_base_inst
    // Output clocks
   (.CLKFBOUT              (clkfbout),
    .CLKOUT0               (clkout0),
    .CLKOUT1               (clkout1),
    .CLKOUT2               (clkout2_unused),
    .CLKOUT3               (clkout3_unused),
    .CLKOUT4               (clkout4_unused),
    .CLKOUT5               (clkout5_unused),
    // Status and control signals
    .LOCKED                (locked),
    .RST                   (rst),
     // Input clock control
    .CLKFBIN               (clkfbout_buf),
    .CLKIN                 (clk50));


  // Output buffering
  //-----------------------------------
  BUFG clkf_buf
   (.O (clkfbout_buf),
    .I (clkfbout));

  BUFG clkout1_buf
   (.O   (clk52),
    .I   (clkout0));

  BUFG clkout2_buf
   (.O   (clk156),
    .I   (clkout1));
  `endif
endmodule

