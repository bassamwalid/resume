library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;



entity lcd16x2 is
    Port ( clk : in STD_LOGIC;
           lcd_rs : out STD_LOGIC;
           lcd_e : out STD_LOGIC;
           lcd_rw : out STD_LOGIC;
           data : out STD_LOGIC_VECTOR (7 downto 0)
);    
                
end lcd16x2;

architecture Behavioral of lcd16x2 is
type arr is array(1 to 38) of std_logic_vector(7 downto 0);
constant data_rom: arr := (
    X"38", X"0C", X"06", X"01", X"C0", 
    X"53", X"4C", X"4F", X"57", X"20", X"44", X"4F", X"57", X"4E", X"2C", X"50", X"4C", X"45", X"41", X"53", X"45", 
    X"0D", X"C0",
    X"44", X"52", X"49", X"56", X"45", X"20", X"53", X"41", X"46", X"45", X"4C", X"59", X"20", X"3A", X"29");
signal en_timing : integer range  0 to 100000;
signal data_pos : integer range  1 to 39;
signal flag: std_logic := '0';
begin
lcd_rw<='0';
process(clk)
begin
if rising_edge(clk) then
    if (flag = '0') then 
    
        if en_timing<=50000 then --hena han7ot el data then wait 
            en_timing<=en_timing+1;
            lcd_e<='1';
            data<=data_rom(data_pos)(7 downto 0);
        elsif en_timing>50000 and en_timing<100000 then --wait abit then input new letter in next cycle
            en_timing<=en_timing+1;
            lcd_e<='0';
        elsif en_timing=100000 then --when you rach 100000, go to next letter and start cycle
            data_pos<=data_pos+1;
            en_timing<=0;
        end if;
        
        
        if (data_pos<=5) then --ma3na 2no haya5od operation
            lcd_rs<='0';
        elsif (data_pos>5 and data_pos < 22) then --mn 2wel hena hya5od letters
            lcd_rs<='1';
        elsif (data_pos = 22) then
            lcd_rs <= '0';
        elsif (data_pos >= 24 and data_pos <39) then
            lcd_rs <= '1';
        elsif (data_pos = 39) then
            flag <= '1';
        end if;
    end if;    
end if;

end process;



end Behavioral;