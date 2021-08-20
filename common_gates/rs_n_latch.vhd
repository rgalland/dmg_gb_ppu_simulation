----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 07/23/2021 09:36:55 PM
-- Design Name: 
-- Module Name: rs_n_latch - Behavioral
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

entity rs_n_latch is
    Port ( r_n : in STD_LOGIC;
           s_n : in STD_LOGIC;
           q : out STD_LOGIC;
           q_n : out STD_LOGIC);
end rs_n_latch;

architecture Behavioral of rs_n_latch is

signal q_temp, q_n_temp : std_logic;

begin

    q_temp <= s_n nand q_n_temp;
    q_n_temp <= r_n nand q_temp;
    
    q <= q_temp; 
    q_n <= q_n_temp;

end Behavioral;
