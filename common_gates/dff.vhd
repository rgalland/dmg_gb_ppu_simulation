----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 08/09/2021 05:00:01 PM
-- Design Name: 
-- Module Name: dff - Behavioral
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity dff is
    Port ( clk : in STD_LOGIC;
           rst_n : in STD_LOGIC;
           d : in STD_LOGIC;
           q : out STD_LOGIC;
           q_n : out STD_LOGIC);
end dff;

architecture Behavioral of dff is

    signal q_out : std_logic;
    
begin

    process(clk, rst_n)
    begin
        if rst_n = '0' then
            q_out <= '0';
        elsif rising_edge(clk) then
            q_out <= d;        
        end if;
    end process;
    
    q <= q_out;
    q_n <= not q_out; 

end Behavioral;
