-------------------------------------------------------------------------------
--
-- MSX1 FPGA project
--
-- Copyright (c) 2016, Fabio Belavenuto (belavenuto@gmail.com)
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

entity qmxc6slx25_top is
	port (
		-- Clock (50MHz)
		clock_50M_i				: in    std_logic;
		-- SDRAM (MT48LC16M16A2 = 16Mx16 = 32MB)
--		sdram_clock_o			: out   std_logic									:= '0';
--		sdram_cke_o    	  	: out   std_logic									:= '0';
--		sdram_addr_o			: out   std_logic_vector(12 downto 0)		:= (others => '0');
--		sdram_dq_io				: inout std_logic_vector(15 downto 0)		:= (others => 'Z');
--		sdram_ba_o				: out   std_logic_vector( 1 downto 0)		:= (others => '0');
--		sdram_dqml_o			: out   std_logic;
--		sdram_dqmh_o			: out   std_logic;
--		sdram_cs_n_o   	  	: out   std_logic									:= '1';
--		sdram_we_n_o			: out   std_logic									:= '1';
--		sdram_cas_n_o			: out   std_logic									:= '1';
--		sdram_ras_n_o			: out   std_logic									:= '1';
		
			-- SRAM
		sram_addr_o			: out   std_logic_vector(20 downto 0)	:= (others => '0');
		sram_data_io		: inout std_logic_vector( 7 downto 0)	:= (others => 'Z');
		sram_we_n_o			: out   std_logic								:= '1';
		sram_ceoe_n_o		: out   std_logic								:= '1';
      sram_ub_n_o       : out   std_logic								:= '1';
		sram_lb_n_o       : out   std_logic								:= '0';
				
		
		-- SPI FLASH (FPGA and Aux)
		flash_clk_o				: out   std_logic									:= '0';
		flash_data_i			: in    std_logic;
		flash_data_o			: out   std_logic									:= '0';
		flash_cs_n_o			: out   std_logic									:= '1';
		-- VGA 4:4:4
		vga_r_o					: out   std_logic_vector(5 downto 0)		:= (others => '0');
		vga_g_o					: out   std_logic_vector(5 downto 0)		:= (others => '0');
		vga_b_o					: out   std_logic_vector(5 downto 0)		:= (others => '0');
		vga_hs_o					: out   std_logic									:= '1';
		vga_vs_o					: out   std_logic									:= '1';
		-- Keys and Leds
		keys_n_i					: in    std_logic_vector(1 downto 0);
		leds_n_o					: out   std_logic_vector(1 downto 0)		:= (others => '1');
		-- PS/2 Keyboard
		ps2_clk_io				: inout std_logic									:= 'Z';
		ps2_dat_io		 		: inout std_logic									:= 'Z';
		
		-- Audio
    	i2s_mclk_o				: out   std_logic									:= '0';
		i2s_bclk_o				: out   std_logic									:= '0';
		i2s_lrclk_o				: out   std_logic									:= '0';
		i2s_data_o				: out   std_logic									:= '0';
		
		dac_l_o 					: out   std_logic			:= '0';
		dac_r_o					: out   std_logic			:= '0';
		ear_i						: in    std_logic;

		-- SD Card
		sd_cs_n_o				: out   std_logic									:= '1';
		sd_sclk_o				: out   std_logic									:= '0';
		sd_mosi_o				: out   std_logic									:= '0';
		sd_miso_i				: in    std_logic;
--		sd_pres_n_i				: in    std_logic;
--		sd_wp_i					: in    std_logic;
		
		-- Joysticks
		JOY_CLK					: out   std_logic;
		JOY_LOAD 				: out   std_logic;
		JOY_DATA 				: in    std_logic;
      joy_select           : out   std_logic;
		
		-- Others
		uart_tx_o				: out   std_logic									:= '1'
		
		-- Debug
		--led_o						: out   std_logic
	);
end;

architecture behavior of qmxc6slx25_top is

	-- Resets
	signal por_cnt_s			: unsigned(7 downto 0)				:= (others => '1');
	signal por_clock_s		: std_logic;
	signal por_s				: std_logic;
	signal reset_s				: std_logic;
	signal soft_por_s			: std_logic;
	signal soft_reset_k_s	: std_logic;
	signal soft_reset_s_s	: std_logic;
	signal soft_rst_cnt_s	: unsigned(7 downto 0)	:= X"FF";
	signal reload_s			: std_logic;

	-- Clocks
	signal clock_master_s	: std_logic;
	signal clock_sdram_s		: std_logic;
	signal clock_vdp_s		: std_logic;
	signal clock_cpu_s		: std_logic;
	signal clock_psg_en_s	: std_logic;
	signal clk50       	   : std_logic;
	signal clock_3m_s			: std_logic;
	signal turbo_on_s			: std_logic;
	signal clock_8m_s			: std_logic;
	signal locked				: std_logic;
   signal clk_16_div       : std_logic_vector(3 downto 0) := (others=>'0');
   signal clk16		      : std_logic;  -- 16,66Mhz For ZXDOS JOY


	-- RAM
	signal ram_addr_s			: std_logic_vector(22 downto 0);		-- 8MB
	signal ram_data_from_s	: std_logic_vector( 7 downto 0);
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
	signal audio_psg_s		: unsigned(7 downto 0);
	signal beep_s				: std_logic;
	signal tapein_s			: std_logic_vector(7 downto 0);
	signal audio_l_s			: signed(15 downto 0);
	signal audio_r_s			: signed(15 downto 0);
	signal audio_l_amp_s		: signed(14 downto 0);
	signal audio_r_amp_s		: signed(14 downto 0);
	signal volumes_s			: volumes_t;

	-- Video
	signal rgb_r_s				: std_logic_vector( 3 downto 0);
	signal rgb_g_s				: std_logic_vector( 3 downto 0);
	signal rgb_b_s				: std_logic_vector( 3 downto 0);
	signal y_grey           : std_logic_vector( 9 downto 0);
	signal vga_grey         : std_logic_vector( 1 downto 0):= "00";
	signal rgb_hsync_n_s		: std_logic;
	signal rgb_vsync_n_s		: std_logic;
	signal vga_en_s			: std_logic;
	signal ntsc_pal_s			: std_logic;

	-- Keyboard
	signal rows_s				: std_logic_vector( 3 downto 0);
	signal cols_s				: std_logic_vector( 7 downto 0);
	signal caps_en_s			: std_logic;
	signal extra_keys_s		: std_logic_vector( 8 downto 0); -- F3, F10, F1, F4, F5, F11, Print Screen, Scroll Lock, Pause/Break
	signal keyb_valid_s		: std_logic;
	signal keyb_data_s		: std_logic_vector( 7 downto 0);
	signal keymap_addr_s		: std_logic_vector( 8 downto 0);
	signal keymap_data_s		: std_logic_vector( 7 downto 0);
	signal keymap_we_s		: std_logic;
	signal reload_core_s		: std_logic;
	
	-- SPI
	signal spi_mosi_s			: std_logic;
	signal spi_miso_s			: std_logic;
	signal spi_sclk_s			: std_logic;
	signal sdspi_cs_n_s		: std_logic;
	signal flspi_cs_n_s		: std_logic;

	-- Bus
	signal bus_addr_s			: std_logic_vector(15 downto 0);
	signal bus_data_from_s	: std_logic_vector( 7 downto 0)		:= (others => '1');
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

	--- OPLL
	signal opll_cs_n_s		: std_logic := '1';
	signal opll_mo_s			: signed(12 downto 0)				:= (others => '0');
	signal opll_ro_s			: signed(12 downto 0)				:= (others => '0');

	-- MIDI interface
	signal midi_cs_n_s		: std_logic								:= '1';
	signal midi_data_from_s	: std_logic_vector( 7 downto 0)	:= (others => '1');
	signal midi_hd_s			: std_logic								:= '0';
	signal midi_int_n_s		: std_logic								:= '1';
	
	
	-- JOYSTICKS
	signal joy1up			: std_logic;
	signal joy1down		: std_logic;
	signal joy1left		: std_logic;
	signal joy1right		: std_logic;
	signal joy1fire1		: std_logic;
	signal joy1fire2		: std_logic;
	signal joy1fire3		: std_logic;
	signal joy1start		: std_logic;
	signal joy2up			: std_logic;
	signal joy2down		: std_logic;
	signal joy2left		: std_logic;
	signal joy2right		: std_logic;
	signal joy2fire1		: std_logic;
	signal joy2fire2		: std_logic;
	signal joy2fire3		: std_logic;
	signal joy2start		: std_logic;
   signal joyAmd           		: std_logic_vector(11 downto 0);
   signal joyBmd						: std_logic_vector(11 downto 0);
	
	-- SD
	signal sd_cs_n_s		: std_logic;

   
	component vga_to_greyscale is 
	port (
			   r_in			: in std_logic_vector( 9 downto 0)	:= (others => '0');
				g_in			: in std_logic_vector( 9 downto 0)	:= (others => '0');
				b_in			: in std_logic_vector( 9 downto 0)	:= (others => '0');
				y_out			: out std_logic_vector( 9 downto 0)	:= (others => '1')
	);
	end component;

begin

	-- PLL
	pll_1: entity work.pll1
	port map (
		CLK_IN1	=> clock_50M_i,
		CLK_OUT1	=> clock_master_s,		-- 21.429 MHz (6x NTSC)
--		CLK_OUT2	=> open, --clock_sdram_s,			-- 85.716 MHz (4x master)
--		CLK_OUT3	=> open, -- sdram_clock_o,			-- 85.716 MHz -90
		CLK_OUT2	=> clock_8m_s,				-- 8 MHz (for MIDI)
		CLK_OUT3	=> clk50,
		LOCKED 	=> locked
	);

	-- Clocks
	clks: entity work.clocks
	port map (
		clock_i			=> clock_master_s,
		por_i				=> por_clock_s,
		turbo_on_i		=> turbo_on_s,
		clock_vdp_o		=> clock_vdp_s,
		clock_5m_en_o	=> open,
		clock_cpu_o		=> clock_cpu_s,
		clock_psg_en_o	=> clock_psg_en_s,
		clock_3m_o		=> clock_3m_s
	);

	-- The MSX1
	the_msx: entity work.msx
	generic map (
		hw_id_g			=> 7,
		hw_txt_g			=> "ZX-DOS+ Board 2MB",
		hw_version_g	=> actual_version,
		video_opt_g		=> 1,						-- dblscan configurable
		ramsize_g		=> 2048,
		hw_hashwds_g	=> '1'
	)
	port map (
		-- Clocks
		clock_i			=> clock_master_s,
		clock_vdp_i		=> clock_vdp_s,
		clock_cpu_i		=> clock_cpu_s,
		clock_psg_en_i	=> clock_psg_en_s,
		-- Turbo
		turbo_on_k_i	=> extra_keys_s(3),	-- F11
		turbo_on_o		=> turbo_on_s,
		-- Resets
		reset_i			=> reset_s,
		por_i				=> por_s,
		softreset_o		=> soft_reset_s_s,
		reload_o			=> reload_s,
		-- Options
		opt_nextor_i	=> '1',
		opt_mr_type_i	=> "00",
		opt_vga_on_i	=> '1',
		-- RAM
		ram_addr_o		=> ram_addr_s,
		ram_data_i		=> ram_data_from_s,
		ram_data_o		=> ram_data_to_s,
		ram_ce_o			=> ram_ce_s,
		ram_we_o			=> ram_we_s,
		ram_oe_o			=> ram_oe_s,
		-- ROM
		rom_addr_o		=> open,
		rom_data_i		=> ram_data_from_s,
		rom_ce_o			=> open,
		rom_oe_o			=> open,
		-- External bus
		bus_addr_o		=> bus_addr_s,
		bus_data_i		=> bus_data_from_s,
		bus_data_o		=> bus_data_to_s,
		bus_rd_n_o		=> bus_rd_n_s,
		bus_wr_n_o		=> bus_wr_n_s,
		bus_m1_n_o		=> bus_m1_n_s,
		bus_iorq_n_o	=> bus_iorq_n_s,
		bus_mreq_n_o	=> bus_mreq_n_s,
		bus_sltsl1_n_o	=> bus_sltsl1_n_s,
		bus_sltsl2_n_o	=> bus_sltsl2_n_s,
		bus_wait_n_i	=> bus_wait_n_s,
		bus_nmi_n_i		=> '1',
		bus_int_n_i		=> '1', --bus_int_n_s,
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
		keyb_valid_i	=> keyb_valid_s,
		keyb_data_i		=> keyb_data_s,
		keymap_addr_o	=> keymap_addr_s,
		keymap_data_o	=> keymap_data_s,
		keymap_we_o		=> keymap_we_s,
		-- Audio
		audio_scc_o		=> audio_scc_s,
		audio_psg_o		=> audio_psg_s,
		beep_o			=> beep_s,
		volumes_o		=> volumes_s,
		-- K7
		k7_motor_o		=> open,
		k7_audio_o		=> open,
		k7_audio_i		=> ear_i,
		-- Joystick
		joy1_up_i		=> joy1up,
		joy1_down_i		=> joy1down,
		joy1_left_i		=> joy1left,
		joy1_right_i	=> joy1right,
		joy1_btn1_i		=> joy1fire1,
		joy1_btn1_o		=> open,
		joy1_btn2_i		=> joy1fire2,
		joy1_btn2_o		=> open,
		joy1_out_o		=> open,
		joy2_up_i		=> joy2up,
		joy2_down_i		=> joy2down,
		joy2_left_i		=> joy2left,
		joy2_right_i	=> joy2right,
		joy2_btn1_i		=> joy2fire1,
		joy2_btn1_o		=> open,
		joy2_btn2_i		=> joy2fire2,
		joy2_btn2_o		=> open,
		joy2_out_o		=> open,
		-- Video
		rgb_r_o			=> rgb_r_s,
		rgb_g_o			=> rgb_g_s,
		rgb_b_o			=> rgb_b_s,
		hsync_n_o		=> rgb_hsync_n_s,
		vsync_n_o		=> rgb_vsync_n_s,
		ntsc_pal_o		=> ntsc_pal_s,
		vga_on_k_i		=> extra_keys_s(2),		-- Print Screen
		vga_en_o			=> vga_en_s,
		scanline_on_k_i=> extra_keys_s(1),		-- Scroll Lock
		scanline_en_o	=> open,
		vertfreq_on_k_i=> extra_keys_s(0),		-- Pause/Break
		-- SPI/SD
--		flspi_cs_n_o	=> flspi_cs_n_s,
--		spi_cs_n_o		=> sdspi_cs_n_s,
--		spi_sclk_o		=> spi_sclk_s,
--		spi_mosi_o		=> spi_mosi_s,
--		spi_miso_i		=> spi_miso_s,
--		sd_pres_n_i		=> '0',
--		sd_wp_i			=> '0',
		flspi_cs_n_o	=> open,
		spi_cs_n_o		=> sd_cs_n_s,
		spi_sclk_o		=> sd_sclk_o,
		spi_mosi_o		=> sd_mosi_o,
		spi_miso_i		=> sd_miso_i,
		sd_pres_n_i		=> '0',
		sd_wp_i			=> '0',
		-- DEBUG
		D_wait_o			=> open,
		D_slots_o		=> open,
		D_ipl_en_o		=> open
	);
   sd_cs_n_o	<= sd_cs_n_s;
	-- RAM
--	ram: entity work.ssdram256Mb
--	generic map (
--		freq_g		=> 85
--	)
--	port map (
--		clock_i		=> clock_sdram_s,
--		reset_i		=> reset_s,
--		refresh_i	=> '1',
--		-- Static RAM bus
--		addr_i		=> "00" & ram_addr_s,
--		data_i		=> ram_data_to_s,
--		data_o		=> ram_data_from_s,
--		cs_i			=> ram_ce_s,
--		oe_i			=> ram_oe_s,
--		we_i			=> ram_we_s,
--		-- SD-RAM ports
--		mem_cke_o	=> sdram_cke_o,
--		mem_cs_n_o	=> sdram_cs_n_o,
--		mem_ras_n_o	=> sdram_ras_n_o,
--		mem_cas_n_o	=> sdram_cas_n_o,
--		mem_we_n_o	=> sdram_we_n_o,
--		mem_udq_o	=> sdram_dqmh_o,
--		mem_ldq_o	=> sdram_dqml_o,
--		mem_ba_o		=> sdram_ba_o,
--		mem_addr_o	=> sdram_addr_o,
--		mem_data_io	=> sdram_dq_io
--	);

   -- RAM
	sram_addr_o			<= ram_addr_s(20 downto 0);
	sram_data_io		<= ram_data_to_s	when ram_we_s = '1'	else (others => 'Z');
	ram_data_from_s	<= sram_data_io;
	sram_we_n_o			<= not ram_we_s;
	sram_ceoe_n_o		<= '0';

	-- VRAM
	vram: entity work.spram
	generic map (
		addr_width_g => 14,
		data_width_g => 8
	)
	port map (
		clk_i		=> clock_master_s,
		we_i		=> vram_we_s,
		addr_i	=> vram_addr_s,
		data_i	=> vram_di_s,
		data_o	=> vram_do_s
	);

	-- Keyboard PS/2
	keyb: entity work.keyboard
	port map (
		clock_i			=> clock_3m_s,
		reset_i			=> reset_s,
		-- MSX
		rows_coded_i	=> rows_s,
		cols_o			=> cols_s,
		keymap_addr_i	=> keymap_addr_s,
		keymap_data_i	=> keymap_data_s,
		keymap_we_i		=> keymap_we_s,
		-- LEDs
		led_caps_i		=> caps_en_s,
		-- PS/2 interface
		ps2_clk_io		=> ps2_clk_io,
		ps2_data_io		=> ps2_dat_io,
		-- Direct Access
		keyb_valid_o	=> keyb_valid_s,
		keyb_data_o		=> keyb_data_s,
		--
		reset_o			=> soft_reset_k_s,
		por_o				=> soft_por_s,
		reload_core_o	=> reload_core_s,
		extra_keys_o	=> extra_keys_s
	);
		-- JOYSTICKS
--		joy: entity work.joydecoder
--	port map (
--		clk				=> clock_8m_s,
--		joy_data			=> JOY_DATA,
--		joy_clk			=> JOY_CLK,
--		joy_load 		=> JOY_LOAD,
--		clock_locked   => locked,
--		joy1up  			=> joy1up,
--		joy1down			=> joy1down,
--		joy1left			=> joy1left,
--		joy1right		=> joy1right,
--		joy1fire1		=> joy1fire1,
--		joy1fire2		=> joy1fire2,
--		joy1fire3		=> joy1fire3,
--		joy1start		=> joy1start,
--		joy2up  			=> joy2up,
--		joy2down			=> joy2down,
--		joy2left			=> joy2left,
--		joy2right		=> joy2right,
--		joy2fire1		=> joy2fire1,
--		joy2fire2		=> joy2fire2,
--		joy2fire3		=> joy2fire3,
--		joy2start		=> joy2start
--	);
   --clock joystick
   process (clk50)
   begin
		if rising_edge(clk50) then
			if (clk_16_div = 2) then --50/3 = 16,6Mhz
				clk_16_div <= (others=>'0');
				clk16 <= '1';
			else
				clk_16_div <= clk_16_div + 1;
				clk16 <= '0';
			end if;
      end if;
   end process;
   
   decodificador_joysticks : entity work.joydecoder
   port map
   (
    --clk => clk_28_div(0), --14Mhz
	 clk => clk16, --12,7Mhz a 17,5Mhz
    joy_data => JOY_DATA, 
    joy_clk => JOY_CLK, 
    joy_load_n => JOY_LOAD, 
	 reset => not locked,
	 hsync_n_s => rgb_hsync_n_s,

	 joy1_o => joyAmd,  -- MXYZ SACB RLDU  Negative Logic
	 joy2_o => joyBmd   -- MXYZ SACB RLDU  Negative Logic
   );   
	joy_select <= rgb_hsync_n_s;

   joy1up  <= joyAmd(0);
   joy1down	<= joyAmd(1);
   joy1left	<= joyAmd(2);
   joy1right <= joyAmd(3);
   joy1fire1	<= joyAmd(4);
   joy1fire2	<= joyAmd(5);
   joy1fire3	<= joyAmd(6);
   joy1start <= joyAmd(7);
   joy2up <= joyBmd(0);
   joy2down	<= joyBmd(1);
   joy2left	<= joyBmd(2);
   joy2right <= joyBmd(3);
   joy2fire1	<= joyBmd(4);
   joy2fire2	<= joyBmd(5);
   joy2fire3	<= joyBmd(6);
   joy2start <= joyBmd(7);
	
	-- Audio
	mixer: entity work.mixers
	port map (
		clock_i			=> clock_master_s,
		reset_i			=> reset_s,
		volumes_i		=> volumes_s,
		beep_i			=> beep_s,
		ear_i				=> ear_i,
		audio_scc_i		=> audio_scc_s,
		audio_psg_i		=> audio_psg_s,
		jt51_left_i		=> jt51_left_s,
		jt51_right_i	=> jt51_right_s,
		opll_mo_i		=> opll_mo_s,
		opll_ro_i		=> opll_ro_s,
		audio_mix_l_o	=> audio_l_s,
		audio_mix_r_o	=> audio_r_s
	);

	-- I2S out
	i2s : entity work.i2s_transmitter
	generic map (
		mclk_rate		=> 10714500,		-- unusual values
		sample_rate		=> 83707, --167414,
		preamble			=> 0,
		word_length		=> 16
	)
	port map (
		clock_i			=> clock_master_s, --clock_master_s,	-- 21.477 MHz (2xMCLK)
		reset_i			=> reset_s,
		-- Parallel input
		pcm_l_i			=> std_logic_vector(audio_l_amp_s & '0'),
		pcm_r_i			=> std_logic_vector(audio_r_amp_s & '0'),
		i2s_mclk_o		=> i2s_mclk_o,
		i2s_lrclk_o		=> i2s_lrclk_o,
		i2s_bclk_o		=> i2s_bclk_o,
		i2s_d_o			=> i2s_data_o
	);

--	i2s_soud : entity work.audio_top
--port map
--(
--		clk_50MHz 	=> clock_50M_i,
--		dac_MCLK 	=> i2s_mclk_o,
--		dac_LRCK 	=> i2s_lrclk_o,
--		dac_SCLK 	=> i2s_bclk_o,
--		dac_SDIN 	=> i2s_data_o,
--		L_data 		=> std_logic_vector(audio_l_amp_s & '0'),
--		R_data 		=> std_logic_vector(audio_r_amp_s & '0')
-- );

	audio_l_amp_s	<= audio_l_s(15) & audio_l_s(13 downto 0);
	audio_r_amp_s	<= audio_r_s(15) & audio_r_s(13 downto 0);

	-- Left Channel
	audiol : entity work.dac_dsm2v
	generic map (
		nbits_g	=> 15
	)
	port map (
		reset_i	=> reset_s,
		clock_i	=> clock_3m_s,
		dac_i		=> audio_l_amp_s,
		dac_o		=> dac_l_o
	);

	-- Right Channel
	audior : entity work.dac_dsm2v
	generic map (
		nbits_g	=> 15
	)
	port map (
		reset_i	=> reset_s,
		clock_i	=> clock_3m_s,
		dac_i		=> audio_r_amp_s,
		dac_o		=> dac_r_o
	);

	-- Multiboot
--	mb: entity work.multiboot
--	generic map (
--		bit_g			=> 2
--	)
--	port map (
--		reset_i		=> por_s,
--		clock_i		=> clock_vdp_s,
--		start_i		=> reload_s,
--		spi_addr_i	=> X"000000"
--	);
   mb: entity work.multiboot
	port map (
		clk_icap		=> clock_8m_s,
      REBOOT		=> reload_core_s
   );
	-- Glue logic 

	-- Glue logic

	-- Power-on counter
	process (clock_master_s)
	begin
		if rising_edge(clock_master_s) then
			if por_cnt_s /= 0 then
				por_cnt_s <= por_cnt_s - 1;
			end if;
		end if;
	end process;

   -- Resets
	por_clock_s	<= '1'	when por_cnt_s /= 0																else '0';
	por_s			<= '1'	when por_cnt_s /= 0  or soft_por_s = '1'   or keys_n_i(1) = '0' 	else '0';  -- or extra_keys_s(6) = '1'  -- F1
	reset_s		<= '1'	when soft_rst_cnt_s = X"00" or por_s = '1' or keys_n_i(0) = '0'   else '0';  -- or extra_keys_s(8) = '1'  -- F3

	process(clock_master_s)
	begin
		if rising_edge(clock_master_s) then
			if reset_s = '1' or por_s = '1' then
				soft_rst_cnt_s	<= X"FF";
			elsif (soft_reset_k_s = '1' or soft_reset_s_s = '1') and soft_rst_cnt_s /= X"00" then
				soft_rst_cnt_s <= soft_rst_cnt_s - 1;
			end if;
		end if;
	end process;

	-- Video

   vga_to_grey: vga_to_greyscale
	port map (
		r_in		=> rgb_r_s(3 downto 0) & rgb_r_s(3 downto 0) & rgb_r_s(3 downto 2),
		g_in		=> rgb_g_s(3 downto 0) & rgb_g_s(3 downto 0) & rgb_g_s(3 downto 2),
		b_in		=> rgb_b_s(3 downto 0) & rgb_b_s(3 downto 0) & rgb_b_s(3 downto 2),
		------
		y_out	   => y_grey
	);
	
	
	-- Change between COLOR to GREYSCALE  pressing F10

   process (extra_keys_s(7))
     begin
        if rising_edge(extra_keys_s(7)) then
             vga_grey <= vga_grey + 1; 
        end if;
		  case vga_grey is
		      --color
		      when "00" =>  vga_r_o	<= (rgb_r_s(3 downto 0) & rgb_r_s(3 downto 2)); 
				              vga_g_o	<= (rgb_g_s(3 downto 0) & rgb_g_s(3 downto 2));
								  vga_b_o	<= (rgb_b_s(3 downto 0) & rgb_b_s(3 downto 2));
				--gray
				when "01" =>  vga_r_o	<=  y_grey(9 downto 4);
				              vga_g_o   <=  y_grey(9 downto 4);
								  Vga_b_o  	<=  y_grey(9 downto 4);
				
				--orange
				when "10" =>  vga_r_o	<=  y_grey(9 downto 4);
				              vga_g_o   <=  "00" & y_grey(9 downto 6);
								  Vga_b_o  	<=  "0000" & y_grey(9 downto 8);
				--green
				when others =>  vga_r_o	<=  "000000";
				              vga_g_o   <=  y_grey(9 downto 4);
								  Vga_b_o  	<=  "000000";				
		  end case;
		  
   end process;



	-- VGA Output
--	vga_r_o	<= rgb_r_s & rgb_r_s(3 downto 2);
--	vga_g_o	<= rgb_g_s & rgb_g_s(3 downto 2);
--	vga_b_o	<= rgb_b_s & rgb_b_s(3 downto 2);
	vga_hs_o	<= rgb_hsync_n_s;
	vga_vs_o	<= rgb_vsync_n_s;

	-- SD and Flash
	
--	spi_miso_s		<= sd_miso_i when sdspi_cs_n_s = '0' else flash_data_i;
--   sd_mosi_o 		<= spi_mosi_s;
--	sd_sclk_o 		<= spi_sclk_s;
--	sd_cs_n_o 		<= sdspi_cs_n_s;
--	flash_data_o	<= spi_mosi_s;
--	flash_clk_o		<= spi_sclk_s;
--	flash_cs_n_o	<= flspi_cs_n_s;

	


	-- Peripheral BUS control
	bus_data_from_s	<= jt51_data_from_s	when jt51_hd_s = '1'	else
							   midi_data_from_s	when midi_hd_s = '1'	else
								(others => '1');
	bus_wait_n_s	<= '1';
	bus_int_n_s		<= midi_int_n_s;

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
		ct1_o				=> open,
		ct2_o				=> open,
		irq_n_o			=> open,
		p1_o				=> open,
		-- Low resolution output (same as real chip)
		sample_o			=> open,
		left_o			=> open,
		right_o			=> open,
		-- Full resolution output
		xleft_o			=> jt51_left_s,
		xright_o			=> jt51_right_s,
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
		cs_n			=> opll_cs_n_s,
		we_n			=> bus_wr_n_s,
		melody_o		=> opll_mo_s,
		rythm_o		=> opll_ro_s
	);

	-- MIDI3
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
		wait_n_o			=> open,
		tx_o				=> uart_tx_o
	);

	-- DEBUG
	--led_o		<= not sd_cs_n_s;
	leds_n_o(0) <= not sd_cs_n_s; -- '1';--sdspi_cs_n_s;
	leds_n_o(1) <= '1';--flspi_cs_n_s;

end architecture;