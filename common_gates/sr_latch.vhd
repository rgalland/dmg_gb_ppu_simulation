----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 07/23/2021 03:04:49 PM
-- Design Name: 
-- Module Name: sr_latch - Behavioral
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

entity sr_latch is
    Port ( s : in STD_LOGIC;
           r : in STD_LOGIC;
           q : out STD_LOGIC;
           q_n : out STD_LOGIC);
end sr_latch;

architecture Behavioral of sr_latch is

    signal q_temp, q_n_temp : std_logic;

begin

    q_n_temp <= s nor q_temp;
    q_temp <= r nor q_n_temp;
    
    q <= q_temp; 
    q_n <= q_n_temp;

end Behavioral;
