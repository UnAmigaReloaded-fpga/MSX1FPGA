-------------------------------------------------------------------------------
--
-- MSX1 FPGA project
--
-- Copyright (c) 2016, Fabio Belavenuto (belavenuto@gmail.com)
--
-- TOP created by Victor Trucco (c) 2018
--
-- TOP Modified for Cyclone V QMTECH by Fernando Mosquera (c) 2019  
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- Please report bugs to the author, but before you do so, please
-- make sure that this is not a derivative work and that
-- you have the latest version of this file.
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;
use work.msx_pack.all;

entity mist_top is
	port (
		-- Clocks
		clk27_i				: in    std_logic;

		-- Buttons
		btn_n_i				: in    std_logic_vector(4 downto 1);


		-- SDRAM	(H57V256 = 16Mx16 = 32MB)
		sdram_clk_o			: out   std_logic								:= '0';
		sdram_cke_o			: out   std_logic								:= '0';
		sdram_ad_o			: out   std_logic_vector(12 downto 0)	:= (others => '0');
		sdram_da_io			: inout std_logic_vector(15 downto 0)	:= (others => 'Z');
		sdram_ba_o			: out   std_logic_vector( 1 downto 0)	:= (others => '0');
		sdram_dqm_o			: out   std_logic_vector( 1 downto 0)	:= (others => '1');
--		sdram_ldqm_o			: out   std_logic									:= '1';
--		sdram_udqm_o			: out   std_logic
		sdram_ras_o			: out   std_logic								:= '1';
		sdram_cas_o			: out   std_logic								:= '1';
		sdram_cs_o			: out   std_logic								:= '1';
		sdram_we_o			: out   std_logic								:= '1';


		-- SPI
		spi_do_io			: inout std_logic									:= '1';
		spi_di_i				: in    std_logic;
		spi_sck_i			: in    std_logic;
		conf_data0_i		: in    std_logic;															-- SPI_SS for user_io
		spi_ss2_i			: in    std_logic;															-- FPGA
		spi_ss3_i			: in    std_logic;															-- OSD
		spi_ss4_i			: in    std_logic;	

		-- PS2
--		ps2_clk_io			: inout std_logic								:= 'Z';
--		ps2_data_io			: inout std_logic								:= 'Z';
--		ps2_mouse_clk_io  : inout std_logic								:= 'Z';
--		ps2_mouse_data_io : inout std_logic								:= 'Z';

		-- SD Card
--		sd_cs_n_o			: out   std_logic								:= '1';
--		sd_sclk_o			: out   std_logic								:= '0';
--		sd_mosi_o			: out   std_logic								:= '0';
--		sd_miso_i			: in    std_logic;
		
		-- Joysticks
--		joy1up		   		: in    std_logic								:= '1';
--		joy1down   			: in    std_logic								:= '1';
--		joy1left		   	: in    std_logic								:= '1';
--		joy1right   			: in    std_logic								:= '1';
--		joy1fire1			: in    std_logic								:= '1';
--		joy1fire2			: in    std_logic								:= '1';
--		joy2up	    			: in    std_logic								:= '1';
--		joy2down		   	: in    std_logic								:= '1';
--		joy2left			: in    std_logic								:= '1';
--		joy2right			: in    std_logic								:= '1';
--		joy2fire1			: in    std_logic								:= '1';
--		joy2fire2			: in    std_logic								:= '1';
		
		-- Joysticks
--		JOY_CLK				: out   std_logic;
--		JOY_LOAD 			: out   std_logic;
--		JOY_DATA 			: in    std_logic;
--  		joyX_p7_o			: out   std_logic								:= '1';

		-- Audio
		dac_l_o				: out   std_logic								:= '0';
		dac_r_o				: out   std_logic								:= '0';
		ear_i				: in    std_logic;
		mic_o				: out   std_logic								:= '0';
		
--		i2s_mclk_o			: out   std_logic								:= '0';
--		i2s_bclk_o			: out   std_logic								:= '0';
--		i2s_lrclk_o			: out   std_logic								:= '0';
--		i2s_data_o			: out   std_logic								:= '0';	

		-- VGA
		vga_r_o				: out   std_logic_vector(5 downto 0)	:= (others => '0');
		vga_g_o				: out   std_logic_vector(5 downto 0)	:= (others => '0');
		vga_b_o				: out   std_logic_vector(5 downto 0)	:= (others => '0');
		vga_hsync_n_o			: out   std_logic								:= '1';
		vga_vsync_n_o			: out   std_logic								:= '1';

		uart_tx_o   			: out   std_logic
		
	);
end entity;

architecture behavior of mist_top is


	constant CONF_STR : string := "MSX1;;T1,Reset";

	function to_slv(s: string) return std_logic_vector is
		constant ss: string(1 to s'length) := s;
		variable rval: std_logic_vector(1 to 8 * s'length);
		variable p: integer;
		variable c: integer;
	begin
		for i in ss'range loop
			p := 8 * i;
			c := character'pos(ss(i));
			rval(p - 7 to p) := std_logic_vector(to_unsigned(c,8));
		end loop;
		return rval;
	end function;

	component user_io
	generic (
		STRLEN : integer := 0
	);
	port (
		conf_str			: in  std_logic_vector(8*STRLEN-1 downto 0);
		SPI_SCK				: in  std_logic;
		CONF_DATA0			: in  std_logic;
		SPI_DI				: in  std_logic;
		SPI_DO	 			: out std_logic;
		joystick_0			: out std_logic_vector( 7 downto 0);
		joystick_1			: out std_logic_vector( 7 downto 0);
		joystick_analog_0		: out std_logic_vector(15 downto 0);
		joystick_analog_1		: out std_logic_vector(15 downto 0);
		buttons				: out std_logic_vector( 1 downto 0);
		switches			: out std_logic_vector( 1 downto 0);
		scandoubler_disable		: out std_logic;
		status				: out std_logic_vector( 7 downto 0);
		-- SD Card Emulation
		sd_lba				: in  std_logic_vector(31 downto 0);
		sd_rd				: in  std_logic;
		sd_wr				: in  std_logic;
		sd_ack				: out std_logic;
		sd_conf				: in  std_logic;
		sd_sdhc				: in  std_logic;
		sd_dout				: out std_logic_vector( 7 downto 0);
		sd_dout_strobe			: out std_logic;
		sd_din				: in  std_logic_vector( 7 downto 0);
		sd_din_strobe			: out std_logic;
		sd_mounted			: out std_logic;
		-- ps2 keyboard emulation
		ps2_clk				: in  std_logic;
		ps2_kbd_clk			: out std_logic;
		ps2_kbd_data			: out std_logic;
		ps2_mouse_clk			: out std_logic;
		ps2_mouse_data			: out std_logic;
		-- serial com port 
		serial_data			: in  std_logic_vector( 7 downto 0);
		serial_strobe			: in  std_logic
	);
	end component user_io;

	component osd
	port (
		pclk, sck, ss, sdi, hs_in, vs_in	: in  std_logic;
		red_in, blue_in, green_in		: in  std_logic_vector(5 downto 0);
		red_out, blue_out, green_out		: out std_logic_vector(5 downto 0);
		osd_enable				: out std_logic
	);
	end component osd;

	component sd_card
	port (
		io_lba				: out std_logic_vector(31 downto 0);
		io_rd				: out std_logic;
		io_wr				: out std_logic;
		io_ack				: in  std_logic;
		io_sdhc				: out std_logic;
		io_conf				: out std_logic;
		io_din				: in  std_logic_vector(7 downto 0);
		io_din_strobe			: in  std_logic;
		io_dout				: out std_logic_vector(7 downto 0);
		io_dout_strobe			: in  std_logic;
		allow_sdhc			: in  std_logic;
		sd_cs				: in  std_logic;
		sd_sck				: in  std_logic;
		sd_sdi				: in  std_logic;
		sd_sdo				: out std_logic
	);
	end component sd_card;

	-- Buttons
	signal btn_por_n_s		: std_logic;
	signal btn_reset_n_s		: std_logic;
	signal btn_scan_s		: std_logic;

	-- Resets
	signal pll_locked_s		: std_logic;
	signal por_s			: std_logic;
	signal reset_s			: std_logic;
	signal soft_reset_k_s		: std_logic;
	signal soft_reset_s_s		: std_logic;
	signal soft_por_s		: std_logic;
	signal soft_rst_cnt_s		: unsigned(7 downto 0)	:= X"FF";

	-- Clocks
	signal clock_sdram_s		: std_logic;
	signal clock_master_s		: std_logic;
	signal clock_vdp_s		: std_logic;
	signal clock_cpu_s		: std_logic;
	signal clock_psg_en_s		: std_logic;
	signal clock_8m_s		: std_logic;
	signal clock_3m_s		: std_logic;
	signal turbo_on_s		: std_logic;
	signal clock_vga_s		: std_logic;
	signal clock_dvi_s		: std_logic;
	signal clock_vga2x_s		: std_logic;
	

	-- RAM
	signal ram_addr_s		: std_logic_vector(22 downto 0);		-- 8MB
	signal ram_data_from_s		: std_logic_vector( 7 downto 0);
	signal ram_data_to_s		: std_logic_vector( 7 downto 0);
	signal ram_ce_s			: std_logic;
	signal ram_oe_s			: std_logic;
	signal ram_we_s			: std_logic;

	-- VRAM memory
	signal vram_addr_s		: std_logic_vector(13 downto 0);		-- 16K
	signal vram_do_s			: std_logic_vector( 7 downto 0);
	signal vram_di_s			: std_logic_vector( 7 downto 0);
--	signal vram_ce_s			: std_logic;
--	signal vram_oe_s			: std_logic;
	signal vram_we_s			: std_logic;

	-- Audio
	signal audio_scc_s		: signed(14 downto 0);
	signal audio_psg_s		: unsigned( 7 downto 0);
	signal beep_s				: std_logic;
	signal audio_l_s			: signed(15 downto 0);
	signal audio_r_s			: signed(15 downto 0);
	signal audio_l_amp_s		: signed(14 downto 0);
	signal audio_r_amp_s		: signed(14 downto 0); 
	signal volumes_s			: volumes_t;

	-- Video
	signal rgb_r_s				: std_logic_vector( 3 downto 0);
	signal rgb_g_s				: std_logic_vector( 3 downto 0);
	signal rgb_b_s				: std_logic_vector( 3 downto 0);
	signal rgb_hsync_n_s		: std_logic;
	signal rgb_vsync_n_s		: std_logic;
	signal vga_en_s			: std_logic;
	signal ntsc_pal_s			: std_logic;

	signal rgb_col_s			: std_logic_vector( 3 downto 0);
	signal cnt_hor_s			: std_logic_vector( 8 downto 0);
	signal cnt_ver_s			: std_logic_vector( 7 downto 0);
	signal vga_hsync_n_s		: std_logic;
	signal vga_vsync_n_s		: std_logic;
	signal vga_blank_s		: std_logic;
	signal vga_col_s			: std_logic_vector( 3 downto 0);
	signal vga_r_s				: std_logic_vector( 3 downto 0);
	signal vga_g_s				: std_logic_vector( 3 downto 0);
	signal vga_b_s				: std_logic_vector( 3 downto 0);
	signal scanlines_en_s	: std_logic;
	signal odd_line_s			: std_logic;
	signal sound_hdmi_l_s	: std_logic_vector(15 downto 0);
	signal sound_hdmi_r_s	: std_logic_vector(15 downto 0);

--	signal tdms_r_s			: std_logic_vector( 9 downto 0);
--	signal tdms_g_s			: std_logic_vector( 9 downto 0);
--	signal tdms_b_s			: std_logic_vector( 9 downto 0);
--	signal tdms_p_s			: std_logic_vector( 3 downto 0);
--	signal tdms_n_s			: std_logic_vector( 3 downto 0);

	-- signals to connect sd card emulation with io controller
	signal sd_lba_s					: std_logic_vector(31 downto 0);
	signal sd_rd_s						: std_logic;
	signal sd_wr_s						: std_logic;
	signal sd_ack_s					: std_logic;
	signal sd_conf_s					: std_logic;
	signal sd_sdhc_s					: std_logic;
	signal sd_allow_sdhc_s			: std_logic;
	signal sd_allow_sdhcD_s			: std_logic;
	signal sd_allow_sdhcD2_s		: std_logic;
	signal sd_allow_sdhc_changed_s	: std_logic;
	-- data from io controller to sd card emulation
	signal sd_data_in_s				: std_logic_vector(7 downto 0);
	signal sd_data_in_strobe_s		: std_logic;
	signal sd_data_out_s				: std_logic_vector(7 downto 0);
	signal sd_data_out_strobe_s	: std_logic;
	-- sd card emulation
	signal sd_cs_s						: std_logic;
	signal sd_sck_s					: std_logic;
	signal sd_sdi_s					: std_logic;
	signal sd_sdo_s					: std_logic;

	-- PS/2
	signal ps2_clk_s			: std_logic;
	signal ps2counter_q		: unsigned(10 downto 0);
	signal ps2_keyboard_clk_in_s	: std_logic;
	signal ps2_keyboard_dat_in_s	: std_logic;

	
	-- Keyboard
	signal rows_s				: std_logic_vector( 3 downto 0);
	signal cols_s				: std_logic_vector( 7 downto 0);
	signal caps_en_s			: std_logic;
	signal extra_keys_s		: std_logic_vector( 3 downto 0);
	signal keyb_valid_s		: std_logic;
	signal keyb_data_s		: std_logic_vector( 7 downto 0);
	signal keymap_addr_s		: std_logic_vector( 8 downto 0);
	signal keymap_data_s		: std_logic_vector( 7 downto 0);
	signal keymap_we_s		: std_logic;

	-- Joystick
	signal joy1_out_s			: std_logic;
	signal joy2_out_s			: std_logic;

	-- Bus
	signal bus_addr_s			: std_logic_vector(15 downto 0);
	signal bus_data_from_s	: std_logic_vector( 7 downto 0)	:= (others => '1');
	signal bus_data_to_s		: std_logic_vector( 7 downto 0);
	signal bus_rd_n_s			: std_logic;
	signal bus_wr_n_s			: std_logic;
	signal bus_m1_n_s			: std_logic;
	signal bus_iorq_n_s		: std_logic;
	signal bus_mreq_n_s		: std_logic;
	signal bus_sltsl1_n_s	: std_logic;
	signal bus_sltsl2_n_s	: std_logic;
	signal bus_int_n_s		: std_logic;
	signal bus_wait_n_s		: std_logic;

	-- JT51
	signal jt51_cs_n_s		: std_logic;
	signal jt51_data_from_s	: std_logic_vector( 7 downto 0)	:= (others => '1');
	signal jt51_hd_s			: std_logic								:= '0';
	signal jt51_left_s		: signed(15 downto 0)				:= (others => '0');
	signal jt51_right_s		: signed(15 downto 0)				:= (others => '0');
		
	-- OPLL
	signal opll_cs_n_s		: std_logic							:= '1';
	signal opll_mo_s			: signed(12 downto 0)			:= (others => '0');
	signal opll_ro_s			: signed(12 downto 0)			:= (others => '0');

	-- MIDI interface
	signal midi_cs_n_s		: std_logic								:= '1';
	signal midi_data_from_s	: std_logic_vector( 7 downto 0)	:= (others => '1');
	signal midi_hd_s			: std_logic								:= '0';
	signal midi_int_n_s		: std_logic								:= '1';
	

	-- JOYSTICKS
	signal joy1up			: std_logic								:= '1';
	signal joy1down			: std_logic								:= '1';
	signal joy1left			: std_logic								:= '1';
	signal joy1right		: std_logic								:= '1';
	signal joy1fire1		: std_logic								:= '1';
	signal joy1fire2		: std_logic								:= '1';
--	signal joy1fire3		: std_logic								:= '1';
--	signal joy1start		: std_logic								:= '1';
	signal joy2up			: std_logic								:= '1';
	signal joy2down			: std_logic								:= '1';
	signal joy2left			: std_logic								:= '1';
	signal joy2right		: std_logic								:= '1';
	signal joy2fire1		: std_logic								:= '1';
	signal joy2fire2		: std_logic								:= '1';
--	signal joy2fire3		: std_logic								:= '1';
--	signal joy2start		: std_logic								:= '1';
	
   component pll1 is
	Port ( 	
		inclk0   : in  std_logic := '0'; -- refclk.clk
		c0 		: out std_logic;        -- outclk0.clk
		c1 		: out std_logic;        -- outclk1.clk
		c2 		: out std_logic;        -- outclk2.clk
		c3 		: out std_logic;        -- outclk3.clk
		locked   : out std_logic         -- locked.export

	);
   end component;
	
		
begin

-- PLL1
 	pll:  pll1
	port map (
		inclk0		=> clk27_i,
		c0				=> clock_master_s,		-- 21.477 MHz					[21.484]
		c1				=> clock_sdram_s,			-- 85.908 MHz (4x master)	[85.937]
		c2				=> sdram_clk_o,			-- 85.908 MHz -90°
		c3       	=> clock_8m_s,          --  8.000
		locked		=> pll_locked_s
	);

	-- Clocks
	clks: entity work.clocks
	port map (
		clock_i			=> clock_master_s,
		por_i			=> not pll_locked_s,
		turbo_on_i		=> turbo_on_s,
		clock_vdp_o		=> clock_vdp_s,
		clock_5m_en_o		=> open,
		clock_cpu_o		=> clock_cpu_s,
		clock_psg_en_o		=> clock_psg_en_s,
		clock_3m_o		=> clock_3m_s
	);

	-- The MSX1
	the_msx: entity work.msx
	generic map (
		hw_id_g			=> 8,
		hw_txt_g		=> "Cyclone V Board",
		hw_version_g		=> actual_version,
		video_opt_g		=> 1,						-- dblscan configurable
		ramsize_g		=> 8192,
		hw_hashwds_g		=> '0'
	)
	port map (
		-- Clocks
		clock_i			=> clock_master_s,
		clock_vdp_i		=> clock_vdp_s,
		clock_cpu_i		=> clock_cpu_s,
		clock_psg_en_i		=> clock_psg_en_s,
		-- Turbo
		turbo_on_k_i		=> extra_keys_s(3),	-- F11
		turbo_on_o		=> turbo_on_s,
		-- Resets
		reset_i			=> reset_s,
		por_i			=> por_s,
		softreset_o		=> soft_reset_s_s,
		-- Options
		opt_nextor_i	=> '1',
		opt_mr_type_i	=> "00",
		opt_vga_on_i	=> '1',
		-- RAM
		ram_addr_o		=> ram_addr_s,
		ram_data_i		=> ram_data_from_s,
		ram_data_o		=> ram_data_to_s,
		ram_ce_o		=> ram_ce_s,
		ram_we_o		=> ram_we_s,
		ram_oe_o		=> ram_oe_s,
		-- ROM
		rom_addr_o		=> open,
		rom_data_i		=> ram_data_from_s,
		rom_ce_o		=> open,
		rom_oe_o		=> open,
		-- External bus
		bus_addr_o		=> bus_addr_s,
		bus_data_i		=> bus_data_from_s,
		bus_data_o		=> bus_data_to_s,
		bus_rd_n_o		=> bus_rd_n_s,
		bus_wr_n_o		=> bus_wr_n_s,
		bus_m1_n_o		=> bus_m1_n_s,
		bus_iorq_n_o		=> bus_iorq_n_s,
		bus_mreq_n_o		=> bus_mreq_n_s,
		bus_sltsl1_n_o		=> bus_sltsl1_n_s,
		bus_sltsl2_n_o		=> bus_sltsl2_n_s,
		bus_wait_n_i		=> bus_wait_n_s,
		bus_nmi_n_i		=> '1',
		bus_int_n_i		=> bus_int_n_s,
		-- VDP RAM
		vram_addr_o		=> vram_addr_s,
		vram_data_i		=> vram_do_s,
		vram_data_o		=> vram_di_s,
		vram_ce_o		=> open,--vram_ce_s,
		vram_oe_o		=> open,--vram_oe_s,
		vram_we_o		=> vram_we_s,
		-- Keyboard
		rows_o			=> rows_s,
		cols_i			=> cols_s,
		caps_en_o		=> caps_en_s,
		keyb_valid_i		=> keyb_valid_s,
		keyb_data_i		=> keyb_data_s,
		keymap_addr_o		=> keymap_addr_s,
		keymap_data_o		=> keymap_data_s,
		keymap_we_o		=> keymap_we_s,
		-- Audio
		audio_scc_o		=> audio_scc_s,
		audio_psg_o		=> audio_psg_s,
		beep_o			=> beep_s,
		volumes_o		=> volumes_s,
		-- K7
		k7_motor_o		=> open,
		k7_audio_o		=> mic_o,
		k7_audio_i		=> ear_i,
		-- Joystick
		joy1_up_i		=> joy1up,
		joy1_down_i		=> joy1down,
		joy1_left_i		=> joy1left,
		joy1_right_i		=> joy1right,
		joy1_btn1_i		=> joy1fire1,
		joy1_btn1_o		=> open,
		joy1_btn2_i		=> joy1fire2,
		joy1_btn2_o		=> open,
		joy1_out_o		=> open,
		joy2_up_i		=> joy2up,
		joy2_down_i		=> joy2down,
		joy2_left_i		=> joy2left,
		joy2_right_i		=> joy2right,
		joy2_btn1_i		=> joy2fire1,
		joy2_btn1_o		=> open,
		joy2_btn2_i		=> joy2fire2,
		joy2_btn2_o		=> open,
		joy2_out_o		=> open,
		-- Video
--		cnt_hor_o		=> cnt_hor_s,
--		cnt_ver_o		=> cnt_ver_s,
--		rgb_r_o			=> rgb_col_s,
--		rgb_g_o			=> open,
--		rgb_b_o			=> open,
--		hsync_n_o		=> open,
--		vsync_n_o		=> open,

		cnt_hor_o		=> open,
		cnt_ver_o		=> open,
		rgb_r_o			=> rgb_r_s,
		rgb_g_o			=> rgb_g_s,
		rgb_b_o			=> rgb_b_s,
		hsync_n_o		=> rgb_hsync_n_s,
		vsync_n_o		=> rgb_vsync_n_s,

		ntsc_pal_o			=> ntsc_pal_s,
		vga_on_k_i			=> extra_keys_s(2),		-- Print Screen
		scanline_on_k_i	=> extra_keys_s(1),		-- Scroll Lock
		vga_en_o				=> vga_en_s,
		scanline_en_o		=> open,
		vertfreq_on_k_i	=> extra_keys_s(0),		-- Pause/Break

		-- SPI/SD
		spi_cs_n_o		=> sd_cs_s,
		spi_sclk_o		=> sd_sck_s,
		spi_mosi_o		=> sd_sdo_s,
		spi_miso_i		=> sd_sdi_s,
		sd_pres_n_i		=> '0',
		sd_wp_i			=> '0',
--		spi_cs_n_o		=> sd_cs_n_o,
--		spi_sclk_o		=> sd_sclk_o,
--		spi_mosi_o		=> sd_mosi_o,
--		spi_miso_i		=> sd_miso_i,
--		sd_pres_n_i		=> '0',
--		sd_wp_i			=> '0',
		-- DEBUG
		D_wait_o		=> open,
		D_slots_o		=> open,
		D_ipl_en_o		=> open
	);

   --joyX_p7_o <= not joy1_out_s;		-- for Sega Genesis joypad
	--joy2_p7_o <= not joy2_out_s;		-- for Sega Genesis joypad

	-- RAM
	ram: entity work.ssdram256Mb
	generic map (
		freq_g		=> 85
	)
	port map (
		clock_i		=> clock_sdram_s,
		reset_i		=> reset_s,
		refresh_i	=> '1',
		-- Static RAM bus
		addr_i		=> "00" & ram_addr_s,
		data_i		=> ram_data_to_s,
		data_o		=> ram_data_from_s,
		cs_i		=> ram_ce_s,
		oe_i		=> ram_oe_s,
		we_i		=> ram_we_s,
		-- SD-RAM ports
		mem_cke_o	=> sdram_cke_o,
		mem_cs_n_o	=> sdram_cs_o,
		mem_ras_n_o	=> sdram_ras_o,
		mem_cas_n_o	=> sdram_cas_o,
		mem_we_n_o	=> sdram_we_o,
		mem_udq_o	=> sdram_dqm_o(1),
		mem_ldq_o	=> sdram_dqm_o(0),
		mem_ba_o	=> sdram_ba_o,
		mem_addr_o	=> sdram_ad_o,
		mem_data_io	=> sdram_da_io
	);

	-- VRAM
	vram: entity work.spram
	generic map (
		addr_width_g => 14,
		data_width_g => 8
	)
	port map (
		clk_i		=> clock_master_s,
		we_i		=> vram_we_s,
		addr_i		=> vram_addr_s,
		data_i		=> vram_di_s,
		data_o		=> vram_do_s
	);

	-- Keyboard PS/2
	keyb: entity work.keyboard
	port map (
		clock_i				=> clock_3m_s,
		reset_i				=> reset_s,
		-- MSX
		rows_coded_i		=> rows_s,
		cols_o				=> cols_s,
		keymap_addr_i		=> keymap_addr_s,
		keymap_data_i		=> keymap_data_s,
		keymap_we_i			=> keymap_we_s,
		-- LEDs
		led_caps_i			=> caps_en_s,
		-- PS/2 interface
--		ps2_clk_io			=> ps2_clk_io,
--		ps2_data_io			=> ps2_data_io,
		ps2_clk_io		=> ps2_keyboard_clk_in_s,
		ps2_data_io		=> ps2_keyboard_dat_in_s,
		-- Direct Access
		keyb_valid_o		=> keyb_valid_s,
		keyb_data_o			=> keyb_data_s,
		--
		reset_o				=> soft_reset_k_s,
		por_o					=> soft_por_s,
		reload_core_o		=> open,
		extra_keys_o		=> extra_keys_s
	);

		
	-- Audio
	mixer: entity work.mixers
	port map (
		clock_i			=> clock_master_s,
		reset_i			=> reset_s,
		volumes_i		=> volumes_s,
		beep_i			=> beep_s,
		ear_i			=> ear_i,
		audio_scc_i		=> audio_scc_s,
		audio_psg_i		=> audio_psg_s,
		jt51_left_i		=> jt51_left_s,
		jt51_right_i		=> jt51_right_s,
		opll_mo_i		=> opll_mo_s,
		opll_ro_i		=> opll_ro_s,
		audio_mix_l_o		=> audio_l_s,
		audio_mix_r_o		=> audio_r_s
	);


	-- Left Channel
	audiol : entity work.dac_dsm2v
	generic map (
		nbits_g	=> 15
	)
	port map (
		reset_i		=> reset_s,
		clock_i		=> clock_3m_s,
		dac_i		=> audio_l_amp_s,
		dac_o		=> dac_l_o
	);

	-- Right Channel
	audior : entity work.dac_dsm2v
	generic map (
		nbits_g	=> 15
	)
	port map (
		reset_i		=> reset_s,
		clock_i		=> clock_3m_s,
		dac_i		=> audio_r_amp_s,
		dac_o		=> dac_r_o
	);

	-- Glue logic

	-- Resets
	btn_por_n_s		<= btn_n_i(2) or btn_n_i(4);
	btn_reset_n_s		<= btn_n_i(3) or btn_n_i(4);

	por_s			<= '1'	when pll_locked_s = '0' or soft_por_s = '1' or btn_por_n_s = '0'		else '0';
	reset_s			<= '1'	when soft_rst_cnt_s = X"01"                 or btn_reset_n_s = '0'	else '0';

	process(reset_s, clock_master_s)
	begin
		if reset_s = '1' then
			soft_rst_cnt_s	<= X"00";
		elsif rising_edge(clock_master_s) then
			if (soft_reset_k_s = '1' or soft_reset_s_s = '1' or por_s = '1') and soft_rst_cnt_s = X"00" then
				soft_rst_cnt_s	<= X"FF";
			elsif soft_rst_cnt_s /= X"00" then
				soft_rst_cnt_s <= soft_rst_cnt_s - 1;
			end if;
		end if;
	end process;

	---------------------------------
	-- scanlines
	btnscl: entity work.debounce
	generic map (
		counter_size_g	=> 16
	)
	port map (
		clk_i				=> clock_master_s,
		button_i			=> btn_n_i(1) or btn_n_i(2),
		result_o			=> btn_scan_s
	);
	
	process (por_s, btn_scan_s)
	begin
		if por_s = '1' then
			scanlines_en_s <= '0';
		elsif falling_edge(btn_scan_s) then
			scanlines_en_s <= not scanlines_en_s;
		end if;
	end process;

--	-- VGA framebuffer
--	vga: entity work.vga
--	port map (
--		I_CLK			=> clock_master_s,
--		I_CLK_VGA	=> clock_vga_s,
--		I_COLOR		=> rgb_col_s,
--		I_HCNT		=> cnt_hor_s,
--		I_VCNT		=> cnt_ver_s,
--		O_HSYNC		=> vga_hsync_n_s,
--		O_VSYNC		=> vga_vsync_n_s,
--		O_COLOR		=> vga_col_s,
--		O_HCNT		=> open,
--		O_VCNT		=> open,
--		O_H			=> open,
--		O_BLANK		=> vga_blank_s
--	);

	-- Scanlines
--	process(vga_hsync_n_s,vga_vsync_n_s)
--	begin
--		if vga_vsync_n_s = '0' then
--			odd_line_s <= '0';
--		elsif rising_edge(vga_hsync_n_s) then
--			odd_line_s <= not odd_line_s;
--		end if;
--	end process;
--
--	-- Index => RGB 
--	process (clock_vga_s)
--		variable vga_col_v	: integer range 0 to 15;
--		variable vga_rgb_v	: std_logic_vector(15 downto 0);
--		variable vga_r_v		: std_logic_vector( 3 downto 0);
--		variable vga_g_v		: std_logic_vector( 3 downto 0);
--		variable vga_b_v		: std_logic_vector( 3 downto 0);
--		type ram_t is array (natural range 0 to 15) of std_logic_vector(15 downto 0);
--		constant rgb_c : ram_t := (
--				--      RB0G
--				0  => X"0000",
--				1  => X"0000",
--				2  => X"240C",
--				3  => X"570D",
--				4  => X"5E05",
--				5  => X"7F07",
--				6  => X"D405",
--				7  => X"4F0E",
--				8  => X"F505",
--				9  => X"F707",
--				10 => X"D50C",
--				11 => X"E80C",
--				12 => X"230B",
--				13 => X"CB09",
--				14 => X"CC0C",
--				15 => X"FF0F"
--		);
--	begin
--		if rising_edge(clock_vga_s) then
--			vga_col_v := to_integer(unsigned(vga_col_s));
--			vga_rgb_v := rgb_c(vga_col_v);
--			if scanlines_en_s = '1' then
--				--
--				if vga_rgb_v(15 downto 12) > 1 and odd_line_s = '1' then
--					vga_r_s <= vga_rgb_v(15 downto 12) - 2;
--				else
--					vga_r_s <= vga_rgb_v(15 downto 12);
--				end if;
--				--
--				if vga_rgb_v(11 downto 8) > 1 and odd_line_s = '1' then
--					vga_b_s <= vga_rgb_v(11 downto 8) - 2;
--				else
--					vga_b_s <= vga_rgb_v(11 downto 8);
--				end if;
--				--
--				if vga_rgb_v(3 downto 0) > 1 and odd_line_s = '1' then
--					vga_g_s <= vga_rgb_v(3 downto 0) - 2;
--				else
--					vga_g_s <= vga_rgb_v(3 downto 0);
--				end if;
--			else
--				vga_r_s <= vga_rgb_v(15 downto 12);
--				vga_b_s <= vga_rgb_v(11 downto  8);
--				vga_g_s <= vga_rgb_v( 3 downto  0);
--			end if;
--		end if;
--	end process;




--	vga_r_o			<= vga_r_s & '0';
--	vga_g_o			<= vga_g_s & '0';
--	vga_b_o			<= vga_b_s & '0';
--	vga_hsync_n_o	<= vga_hsync_n_s;
--	vga_vsync_n_o	<= vga_vsync_n_s;


	osd_inst : osd	port map (
		pclk			=> clock_master_s,
		sdi			=> spi_di_i,
		sck			=> spi_sck_i,
		ss				=> spi_ss3_i,
		red_in		=> rgb_r_s & "00",
		green_in		=> rgb_g_s & "00",
		blue_in		=> rgb_b_s & "00",
		hs_in			=> rgb_hsync_n_s,
		vs_in			=> rgb_vsync_n_s,
		red_out		=> vga_r_o,
		green_out	=> vga_g_o,
		blue_out		=> vga_b_o,
		osd_enable	=> open
	);



	-- RGB/VGA Output
--	vga_r_o			<= rgb_r_s & '0';
--	vga_g_o			<= rgb_g_s & '0';
--	vga_b_o			<= rgb_b_s & '0';
	vga_hsync_n_o		<= rgb_hsync_n_s	when vga_en_s = '1'	else (rgb_hsync_n_s and rgb_vsync_n_s);
	vga_vsync_n_o		<= rgb_vsync_n_s	when vga_en_s = '1'	else '1';
--	vga_ntsc_o		<= not ntsc_pal_s;
--	vga_pal_o		<= ntsc_pal_s;


userio_inst : user_io
	generic map (
		STRLEN => CONF_STR'length
	)
	port map (
		conf_str					=> to_slv(CONF_STR),
		SPI_SCK					=> spi_sck_i,
		CONF_DATA0				=> conf_data0_i,
		SPI_DI					=> spi_di_i,
		SPI_DO	 				=> spi_do_io,
		joystick_0				=> open,
		joystick_1				=> open,
		joystick_analog_0		=> open,
		joystick_analog_1		=> open,
		buttons					=> open,
		switches					=> open,
		scandoubler_disable	=> open,
		status					=> open,
		-- SD Card Emulation
		sd_lba					=> sd_lba_s,
		sd_rd						=> sd_rd_s,
		sd_wr						=> sd_wr_s,
		sd_ack					=> sd_ack_s,
		sd_sdhc					=> sd_sdhc_s,
		sd_conf					=> sd_conf_s,
 		sd_dout					=> sd_data_in_s,
 		sd_dout_strobe			=> sd_data_in_strobe_s,
		sd_din					=> sd_data_out_s,
		sd_din_strobe			=> sd_data_out_strobe_s,
		sd_mounted				=> open,
		-- ps2 keyboard emulation
		ps2_clk					=> ps2_clk_s,
		ps2_kbd_clk				=> ps2_keyboard_clk_in_s,
		ps2_kbd_data			=> ps2_keyboard_dat_in_s,
		ps2_mouse_clk			=> open,
		ps2_mouse_data			=> open,
		-- serial com port 
		serial_data				=> (others => '0'),
		serial_strobe			=> '0'
	);

sd_card_d: component sd_card
	port map (
		-- connection to io controller
		io_lba			=> sd_lba_s,
		io_rd				=> sd_rd_s,
		io_wr				=> sd_wr_s,
		io_ack			=> sd_ack_s,
		io_conf			=> sd_conf_s,
		io_sdhc			=>	sd_sdhc_s,
		io_din			=> sd_data_in_s,
		io_din_strobe	=> sd_data_in_strobe_s,
		io_dout			=> sd_data_out_s,
		io_dout_strobe	=> sd_data_out_strobe_s,
		allow_sdhc		=> '1',
		-- connection to host
		sd_cs				=> sd_cs_s,
		sd_sck			=> sd_sck_s,
		sd_sdi			=> sd_sdo_s,
		sd_sdo			=> sd_sdi_s		
	);

	
	-- Peripheral BUS control
	bus_data_from_s		<= jt51_data_from_s	when jt51_hd_s = '1'	else
							   midi_data_from_s	when midi_hd_s = '1'	else
								(others => '1');
	bus_wait_n_s		<= '1';--midi_wait_n_s;
	bus_int_n_s		<= '1';--midi_int_n_s;

	-- JT51
	jt51_cs_n_s <= '0' when bus_addr_s(7 downto 1) = "0010000" and bus_iorq_n_s = '0' and bus_m1_n_s = '1'	else '1';	-- 0x20 - 0x21

	jt51: entity work.jt51_wrapper
	port map (
		clock_i			=> clock_3m_s,
		reset_i			=> reset_s,
		addr_i			=> bus_addr_s(0),
		cs_n_i			=> jt51_cs_n_s,
		wr_n_i			=> bus_wr_n_s,
		rd_n_i			=> bus_rd_n_s,
		data_i			=> bus_data_to_s,
		data_o			=> jt51_data_from_s,
		has_data_o		=> jt51_hd_s,
		ct1_o			=> open,
		ct2_o			=> open,
		irq_n_o			=> open,
		p1_o			=> open,
		-- Low resolution output (same as real chip)
		sample_o		=> open,
		left_o			=> open,
		right_o			=> open,
		-- Full resolution output
		xleft_o			=> jt51_left_s,
		xright_o		=> jt51_right_s,
		-- unsigned outputs for sigma delta converters, full resolution		
		dacleft_o		=> open,
		dacright_o		=> open
	);

	-- OPLL
	opll_cs_n_s	<= '0' when bus_addr_s(7 downto 1) = "0111110" and bus_iorq_n_s = '0' and bus_m1_n_s = '1'	else '1';	-- 0x7C - 0x7D

	opll1 : entity work.opll 
	port map (
		clock_i		=> clock_master_s,
		clock_en_i	=> clock_psg_en_s,
		reset_i		=> reset_s,
		data_i		=> bus_data_to_s,
		addr_i		=> bus_addr_s(0),
		cs_n       	 => opll_cs_n_s,
		we_n       	=> bus_wr_n_s,
		melody_o	=> opll_mo_s,
		rythm_o		=> opll_ro_s
	);

		-- MIDI
	midi_cs_n_s	<= '0' when bus_addr_s(7 downto 1) = "0111111" and bus_iorq_n_s = '0' and bus_m1_n_s = '1'	else '1';	-- 0x7E - 0x7F

	-- MIDI interface
	midi: entity work.midiIntf
	port map (
		clock_i			=> clock_8m_s,
		reset_i			=> reset_s,
		addr_i			=> bus_addr_s(0),
		cs_n_i			=> midi_cs_n_s,
		wr_n_i			=> bus_wr_n_s,
		rd_n_i			=> bus_rd_n_s,
		data_i			=> bus_data_to_s,
		data_o			=> midi_data_from_s,
		has_data_o		=> midi_hd_s,
		-- Outs
		int_n_o			=> midi_int_n_s,
		wait_n_o		=> open,
		tx_o			=> uart_tx_o
	);

	-- DEBUG
--	leds_n_o(0)		<= not turbo_on_s;
--	leds_n_o(1)		<= not caps_en_s;
--	leds_n_o(2)		<= not soft_reset_k_s;
--	leds_n_o(3)		<= not soft_por_s;

end architecture;