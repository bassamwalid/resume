library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.std_logic_arith.ALL;
use IEEE.std_logic_unsigned.all;

entity Test is
--  Port ( );
end Test;

architecture Behavioral of Test is
component Traffic
  Port (clk, reset, sensor1, sensor2, sensor3 , sensor4 : in std_logic ;
        G1, Y1, R1, G2, Y2, R2: out std_logic ;
        
        lcd_rs : out STD_LOGIC;
        lcd_e : out STD_LOGIC;
        lcd_rw : out STD_LOGIC;
        data : out STD_LOGIC_VECTOR (7 downto 0);
        
        buzzer: out std_logic := '0');
        end component;
signal clk: std_logic:='0';
signal reset: std_logic:='0';

signal y1: std_logic;
signal r1: std_logic;
signal g1: std_logic;
signal y2: std_logic;
signal r2: std_logic;
signal g2: std_logic;

signal buzzer: std_logic;

signal sensor1: std_logic;
signal sensor2: std_logic;
signal sensor3: std_logic;
signal sensor4: std_logic;

signal lcd_rs :  STD_LOGIC; --law ba 0 yb2a ht3mel operation law ba 1 yb2a hatda5al letter
signal   lcd_e :  STD_LOGIC;
signal lcd_rw :  STD_LOGIC;
signal data :  STD_LOGIC_VECTOR (7 downto 0);

begin
uut:Traffic port map(clk,reset,sensor1,sensor2,sensor3,sensor4,g1,y1,r1,g2,y2,r2,lcd_rs,lcd_e,lcd_rw,data,buzzer);

clk_process: process
begin
    clk<='0';
    wait for 0.5ns;
    clk<='1';
    wait for 0.5ns;
end process;

stim_proc:process
begin
    reset<='1';
    wait for 0.5ns;
    reset<='0';
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '0';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '0';
    sensor3 <= '1';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '0';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '0';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '1';
    wait for 6000ns; 
    sensor1 <= '0';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '0';
    sensor3 <= '1';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '0';
    sensor4 <= '1';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '0';
    wait for 1000ns;
    sensor1 <= '1';
    sensor2 <= '1';
    sensor3 <= '1';
    sensor4 <= '0';
    wait;
    
end process;

end Behavioral;