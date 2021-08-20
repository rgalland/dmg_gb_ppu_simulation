----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 07/18/2021 11:48:02 AM
-- Design Name: 
-- Module Name: ppu2_tb - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

use STD.TEXTIO.all;
    

entity ppu2_tb is
--  Port ( );
end ppu2_tb;

architecture Behavioral of ppu2_tb is

    component ppu2 is
        port ( 
            clk : in STD_LOGIC;
            rst_n : in STD_LOGIC;
            -- ff40 contrl
            ff40_din : in std_logic_vector(7 downto 0);
            -- ff42, ff43 y, x offsets regs
            ff42_din : in unsigned(7 downto 0);
            ff43_din : in unsigned(7 downto 0);
            -- ff45, lyc reg
            ff45_din : unsigned(7 downto 0);
            -- ff47, bgp reg
            ff47_din : std_logic_vector(7 downto 0);
            -- ff48-9, obp0-1 regs
            ff48_din : std_logic_vector(7 downto 0);
            ff49_din : std_logic_vector(7 downto 0);
            -- ff4a, ff4b y and x window regs
            ff4a_din : std_logic_vector(7 downto 0);
            ff4b_din : std_logic_vector(7 downto 0);
            -- vram
            vram_addr : out std_logic_vector(12 downto 0);
            vram_din  : in std_logic_vector(7 downto 0);
            -- oam
            oam_addr : out std_logic_vector(6 downto 0);
            oam_din  : in std_logic_vector(15 downto 0);
            -- lcd pins
            pin_st : out std_logic; -- hsync
            pin_s : out std_logic; -- vsync
            pin_ld : out std_logic_vector(1 downto 0); -- lcd data
            pin_cp : out std_logic; -- clock
            pin_cpl : out std_logic; -- latch
            pin_cpg : out std_logic -- control
        );
    end component;
    
    signal clk : std_logic := '0';
    signal rst_n : std_logic;
    
    signal vram_addr : std_logic_vector(12 downto 0);
    signal vram_dout : std_logic_vector(7 downto 0);
    signal oam_addr : std_logic_vector(6 downto 0);
    signal oam_dout : std_logic_vector(15 downto 0);
    
    constant VRAM_SIZE : integer := 8192;
    type VRAM_T is array (0 to VRAM_SIZE - 1) of character;
    type VRAM_FILE_T is file of VRAM_T;
    signal vram : VRAM_T;

    constant OAM_SIZE : integer := 160;
    type OAM_T is array (0 to OAM_SIZE - 1) of character;
    type OAM_FILE_T is file of OAM_T;
    signal oam : OAM_T;
    
    signal ff40_din : std_logic_vector(7 downto 0);
    signal ff42_din, ff43_din, ff45_din : unsigned(7 downto 0);
    signal ff47_din, ff48_din, ff49_din, ff4a_din, ff4b_din : std_logic_vector(7 downto 0);
    
    signal pin_st : std_logic; -- hsync
    signal pin_s : std_logic; -- vsync
    signal pin_ld : std_logic_vector(1 downto 0); -- lcd data
    signal pin_cp : std_logic; -- clock
    signal pin_cpl : std_logic; -- latch
    signal pin_cpg : std_logic; -- control
    
    constant BG_TEST : string  := "3";
    constant SP_TEST : string  := "4";
    constant WIN_TEST : string := "5";
    constant PAL_TEST : string := "6";
    constant SP2_TEST : string := "7";
    
    constant TEST : string := SP2_TEST;

begin
      
    vram_dout <= std_logic_vector(to_unsigned(character'pos(vram(to_integer(unsigned(vram_addr)))), 8));  
    oam_dout(7 downto 0) <= std_logic_vector(to_unsigned(character'pos(oam(to_integer(unsigned(oam_addr)) * 2)), 8));  
    oam_dout(15 downto 8) <= std_logic_vector(to_unsigned(character'pos(oam(to_integer(unsigned(oam_addr)) * 2 + 1)), 8));  

    uut : ppu2
        port map ( 
            clk => clk,
            rst_n => rst_n,
            -- ff40 contrl
            ff40_din => ff40_din,
            -- ff42, ff43 y, x offsets regs
            ff42_din => ff42_din,
            ff43_din => ff43_din,
            -- ff45, lyc reg
            ff45_din => ff45_din,
            -- ff47, bgp reg
            ff47_din => ff47_din,
            -- ff48-9, obp0-1 regs
            ff48_din => ff48_din,
            ff49_din => ff49_din,
            -- ff4a, ff4b y and x window regs
            ff4a_din => ff4a_din,
            ff4b_din => ff4b_din,
            -- vram
            vram_addr => vram_addr,
            vram_din => vram_dout,
            -- oam
            oam_addr => oam_addr,
            oam_din => oam_dout,
            pin_st => pin_st,
            pin_s => pin_s,
            pin_ld => pin_ld,
            pin_cp => pin_cp,
            pin_cpl => pin_cpl,
            pin_cpg => pin_cpg);
    
    clk <= not clk after (240/2) *  1 ns;
            
    test_proc : process
        file vram_file : VRAM_FILE_T;
        file oam_file : OAM_FILE_T;
        variable I : integer := 0;
    begin
        case TEST is
            when BG_TEST => 
                ff40_din <= X"C1";
                ff42_din <= X"00";
                ff43_din <= X"00";
                ff45_din <= X"00";
                ff47_din <= X"E4";
                ff48_din <= X"E4";
                ff49_din <= X"1B";
                ff4a_din <= X"00";
                ff4b_din <= X"07";
                FILE_OPEN (vram_file, "vram/graphics3.vrm", read_mode);
                FILE_OPEN (oam_file, "oam/empty.oam", read_mode);
            when SP_TEST => 
                ff40_din <= X"C6";
                ff42_din <= X"00";
                ff43_din <= X"00";
                ff45_din <= X"00";
                ff47_din <= X"E4";
                ff48_din <= X"E4";
                ff49_din <= X"1B";
                ff4a_din <= X"00";
                ff4b_din <= X"07";
                FILE_OPEN (vram_file, "vram/graphics4.vrm", read_mode);
                FILE_OPEN (oam_file, "oam/graphics4.oam", read_mode);
            when SP2_TEST => 
                ff40_din <= X"C6";
                ff42_din <= X"00";
                ff43_din <= X"00";
                ff45_din <= X"00";
                ff47_din <= X"E4";
                ff48_din <= X"E4";
                ff49_din <= X"1B";
                ff4a_din <= X"00";
                ff4b_din <= X"07";
                FILE_OPEN (vram_file, "vram/graphics4.vrm", read_mode);
                FILE_OPEN (oam_file, "oam/graphics4_2.oam", read_mode);
            when WIN_TEST => 
                ff40_din <= X"E1";
                ff42_din <= X"00";  --X"44";
                ff43_din <= X"00";  --X"44";
                ff45_din <= X"00";
                ff47_din <= X"E4";
                ff48_din <= X"E4";
                ff49_din <= X"1B";
                ff4a_din <= X"80";
                ff4b_din <= X"07";
                FILE_OPEN (vram_file, "vram/graphics5.vrm", read_mode);
                FILE_OPEN (oam_file, "oam/empty.oam", read_mode);
            when others => -- Tests background and dummy sprites
                ff40_din <= X"C3";
                ff42_din <= X"00";
                ff43_din <= X"00";
                ff45_din <= X"00";
                ff47_din <= X"E4";
                ff48_din <= X"E4";
                ff49_din <= X"1B";
                ff4a_din <= X"00";
                ff4b_din <= X"07";
                FILE_OPEN (vram_file, "vram/graphics3.vrm", read_mode);
                FILE_OPEN (oam_file, "oam/graphics3.oam", read_mode);
        end case;
        
        if not ENDFILE(vram_file) then
            READ (vram_file, vram);
        else
            -- print issue message
        end if;
        FILE_CLOSE (vram_file);
        
        if not ENDFILE(oam_file) then
            READ (oam_file, oam);
        else
            -- print issue message
        end if;
        FILE_CLOSE (oam_file);
    
        rst_n <= '0';
        wait for 10 us;
        rst_n <= '1';
        
        -- specific loops based on examples
        case TEST is
            when WIN_TEST | SP2_TEST => 
                while I < 256 loop
                    wait for 17 ms;
                    ff42_din <= ff42_din + 1;
                    ff43_din <= ff43_din + 1;
                    I := I + 1;
                end loop;
            when others => null;
        end case;
        
        
        wait;
    end process;        


end Behavioral;
