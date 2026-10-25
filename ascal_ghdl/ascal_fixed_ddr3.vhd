--------------------------------------------------------------------------------
-- ascal_fixed_ddr3 -- FRAC 4, for TANGO_VIDEO_DDR3 builds (gen.sh 4d)
-- Made from ascal_fixed.vhd: every generic and port as there, except the Avalon side for
-- the DDR3 scaler port (TANGO_VIDEO_DDR3, sys-tango docs/DDR3.md 5.7): N_DW = 128,
-- N_AW = 28 (16-byte words), RAMBASE = 0x2000_0000 (the scaler's DDR3 window, DDR3.md 5.3).
-- If a value changes: edit it here, in ascal_fixed.vhd as the case may be, and in the shim's
-- guard; rerun gen.sh.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY ascal_fixed_ddr3 IS
	PORT (
		-- Input video
		i_r   : IN  unsigned(7 DOWNTO 0);
		i_g   : IN  unsigned(7 DOWNTO 0);
		i_b   : IN  unsigned(7 DOWNTO 0);
		i_hs  : IN  std_logic;
		i_vs  : IN  std_logic;
		i_fl  : IN  std_logic;
		i_de  : IN  std_logic;
		i_ce  : IN  std_logic;
		i_clk : IN  std_logic;
		-- Output video
		o_r   : OUT unsigned(7 DOWNTO 0);
		o_g   : OUT unsigned(7 DOWNTO 0);
		o_b   : OUT unsigned(7 DOWNTO 0);
		o_hs  : OUT std_logic;
		o_vs  : OUT std_logic;
		o_de  : OUT std_logic;
		o_vbl : OUT std_logic;
		o_brd : OUT std_logic;
		o_ce  : IN  std_logic;
		o_clk : IN  std_logic;
		o_border    : IN  unsigned(23 DOWNTO 0);
		-- Framebuffer mode
		o_fb_ena    : IN  std_logic;
		o_fb_hsize  : IN  unsigned(11 DOWNTO 0);
		o_fb_vsize  : IN  unsigned(11 DOWNTO 0);
		o_fb_format : IN  unsigned(5 DOWNTO 0);
		o_fb_base   : IN  unsigned(31 DOWNTO 0);
		o_fb_stride : IN  unsigned(13 DOWNTO 0);
		-- Framebuffer palette (PALETTE=false: unused)
		pal1_clk : IN  std_logic;
		pal1_dw  : IN  unsigned(47 DOWNTO 0);
		pal1_dr  : OUT unsigned(47 DOWNTO 0);
		pal1_a   : IN  unsigned(6 DOWNTO 0);
		pal1_wr  : IN  std_logic;
		pal_n    : IN  std_logic;
		pal2_clk : IN  std_logic;
		pal2_dw  : IN  unsigned(23 DOWNTO 0);
		pal2_dr  : OUT unsigned(23 DOWNTO 0);
		pal2_a   : IN  unsigned(7 DOWNTO 0);
		pal2_wr  : IN  std_logic;
		-- Low lag PLL tuning
		o_lltune : OUT unsigned(15 DOWNTO 0);
		-- Input video parameters
		iauto   : IN  std_logic;
		himin   : IN  unsigned(11 DOWNTO 0);
		himax   : IN  unsigned(11 DOWNTO 0);
		vimin   : IN  unsigned(11 DOWNTO 0);
		vimax   : IN  unsigned(11 DOWNTO 0);
		i_hdmax : OUT unsigned(11 DOWNTO 0);
		i_vdmax : OUT unsigned(11 DOWNTO 0);
		-- Output video parameters
		run     : IN  std_logic;
		freeze  : IN  std_logic;
		mode    : IN  unsigned(4 DOWNTO 0);
		htotal  : IN  unsigned(11 DOWNTO 0);
		hsstart : IN  unsigned(11 DOWNTO 0);
		hsend   : IN  unsigned(11 DOWNTO 0);
		hdisp   : IN  unsigned(11 DOWNTO 0);
		hmin    : IN  unsigned(11 DOWNTO 0);
		hmax    : IN  unsigned(11 DOWNTO 0);
		vtotal  : IN  unsigned(11 DOWNTO 0);
		vsstart : IN  unsigned(11 DOWNTO 0);
		vsend   : IN  unsigned(11 DOWNTO 0);
		vdisp   : IN  unsigned(11 DOWNTO 0);
		vmin    : IN  unsigned(11 DOWNTO 0);
		vmax    : IN  unsigned(11 DOWNTO 0);
		vrr     : IN  std_logic;
		vrrmax  : IN  unsigned(11 DOWNTO 0);
		format  : IN  unsigned(1 DOWNTO 0);
		-- Polyphase filter coefficients (FRAC=4 -> poly_a is 8 bits)
		poly_clk : IN  std_logic;
		poly_dw  : IN  unsigned(9 DOWNTO 0);
		poly_a   : IN  unsigned(7 DOWNTO 0);
		poly_wr  : IN  std_logic;
		-- Avalon (N_DW=128, N_AW=28: the DDR3 V port)
		avl_clk           : IN  std_logic;
		avl_waitrequest   : IN  std_logic;
		avl_readdata      : IN  std_logic_vector(127 DOWNTO 0);
		avl_readdatavalid : IN  std_logic;
		avl_burstcount    : OUT std_logic_vector(7 DOWNTO 0);
		avl_writedata     : OUT std_logic_vector(127 DOWNTO 0);
		avl_address       : OUT std_logic_vector(27 DOWNTO 0);
		avl_write         : OUT std_logic;
		avl_read          : OUT std_logic;
		avl_byteenable    : OUT std_logic_vector(15 DOWNTO 0);
		reset_na          : IN  std_logic
	);
END ENTITY ascal_fixed_ddr3;

ARCHITECTURE rtl OF ascal_fixed_ddr3 IS
	SUBTYPE uint12 IS natural RANGE 0 TO 4095;
	SIGNAL o_fb_hsize_n, o_fb_vsize_n                         : uint12;
	SIGNAL himin_n, himax_n, vimin_n, vimax_n                 : uint12;
	SIGNAL i_hdmax_n, i_vdmax_n                               : uint12;
	SIGNAL htotal_n, hsstart_n, hsend_n, hdisp_n, hmin_n, hmax_n : uint12;
	SIGNAL vtotal_n, vsstart_n, vsend_n, vdisp_n, vmin_n, vmax_n : uint12;
	SIGNAL vrrmax_n                                           : uint12;
BEGIN
	o_fb_hsize_n <= to_integer(o_fb_hsize);
	o_fb_vsize_n <= to_integer(o_fb_vsize);
	himin_n      <= to_integer(himin);
	himax_n      <= to_integer(himax);
	vimin_n      <= to_integer(vimin);
	vimax_n      <= to_integer(vimax);
	htotal_n     <= to_integer(htotal);
	hsstart_n    <= to_integer(hsstart);
	hsend_n      <= to_integer(hsend);
	hdisp_n      <= to_integer(hdisp);
	hmin_n       <= to_integer(hmin);
	hmax_n       <= to_integer(hmax);
	vtotal_n     <= to_integer(vtotal);
	vsstart_n    <= to_integer(vsstart);
	vsend_n      <= to_integer(vsend);
	vdisp_n      <= to_integer(vdisp);
	vmin_n       <= to_integer(vmin);
	vmax_n       <= to_integer(vmax);
	vrrmax_n     <= to_integer(vrrmax);
	i_hdmax      <= to_unsigned(i_hdmax_n, 12);
	i_vdmax      <= to_unsigned(i_vdmax_n, 12);
	-- PALETTE=false: ascal never drives pal1_dr/pal2_dr, so in VHDL they read as
	-- their port defaults (0).  GHDL would emit 'bZ for them; drive the defaults here.
	pal1_dr      <= (OTHERS => '0');
	pal2_dr      <= (OTHERS => '0');

	u_ascal : ENTITY work.ascal
		GENERIC MAP (
			MASK         => x"FF",
			RAMBASE      => x"20000000",
			RAMSIZE      => x"00400000",
			INTER        => true,
			HEADER       => false,
			DOWNSCALE    => false,
			BYTESWAP     => true,
			PALETTE      => false,
			PALETTE2     => false,
			ADAPTIVE     => true,
			DOWNSCALE_NN => false,
			FRAC         => 4,
			OHRES        => 2048,
			IHRES        => 512,
			N_DW         => 128,
			N_AW         => 28,
			N_BURST      => 256
		)
		PORT MAP (
			i_r => i_r, i_g => i_g, i_b => i_b,
			i_hs => i_hs, i_vs => i_vs, i_fl => i_fl, i_de => i_de,
			i_ce => i_ce, i_clk => i_clk,
			o_r => o_r, o_g => o_g, o_b => o_b,
			o_hs => o_hs, o_vs => o_vs, o_de => o_de, o_vbl => o_vbl, o_brd => o_brd,
			o_ce => o_ce, o_clk => o_clk,
			o_border    => o_border,
			o_fb_ena    => o_fb_ena,
			o_fb_hsize  => o_fb_hsize_n,
			o_fb_vsize  => o_fb_vsize_n,
			o_fb_format => o_fb_format,
			o_fb_base   => o_fb_base,
			o_fb_stride => o_fb_stride,
			pal1_clk => pal1_clk, pal1_dw => pal1_dw, pal1_dr => OPEN,
			pal1_a => pal1_a, pal1_wr => pal1_wr,
			pal_n => pal_n,
			pal2_clk => pal2_clk, pal2_dw => pal2_dw, pal2_dr => OPEN,
			pal2_a => pal2_a, pal2_wr => pal2_wr,
			o_lltune => o_lltune,
			iauto => iauto,
			himin => himin_n, himax => himax_n, vimin => vimin_n, vimax => vimax_n,
			i_hdmax => i_hdmax_n, i_vdmax => i_vdmax_n,
			run => run, freeze => freeze, mode => mode,
			htotal => htotal_n, hsstart => hsstart_n, hsend => hsend_n, hdisp => hdisp_n,
			hmin => hmin_n, hmax => hmax_n,
			vtotal => vtotal_n, vsstart => vsstart_n, vsend => vsend_n, vdisp => vdisp_n,
			vmin => vmin_n, vmax => vmax_n,
			vrr => vrr, vrrmax => vrrmax_n,
			format => format,
			poly_clk => poly_clk, poly_dw => poly_dw, poly_a => poly_a, poly_wr => poly_wr,
			avl_clk           => avl_clk,
			avl_waitrequest   => avl_waitrequest,
			avl_readdata      => avl_readdata,
			avl_readdatavalid => avl_readdatavalid,
			avl_burstcount    => avl_burstcount,
			avl_writedata     => avl_writedata,
			avl_address       => avl_address,
			avl_write         => avl_write,
			avl_read          => avl_read,
			avl_byteenable    => avl_byteenable,
			reset_na          => reset_na
		);
END ARCHITECTURE rtl;
