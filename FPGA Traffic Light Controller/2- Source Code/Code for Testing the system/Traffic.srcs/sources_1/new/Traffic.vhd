library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.std_logic_arith.ALL;
use IEEE.std_logic_unsigned.all;

entity Traffic is
  Port (clk, reset, sensor1, sensor2, sensor3 , sensor4 : in std_logic ;
        G1, Y1, R1, G2, Y2, R2: out std_logic ;
        
        lcd_rs : out STD_LOGIC;
        lcd_e : out STD_LOGIC;
        lcd_rw : out STD_LOGIC;
        data : out STD_LOGIC_VECTOR (7 downto 0);
        
        buzzer: out std_logic := '0');
end Traffic;

architecture Behavioral of Traffic is
    type state_type is (S0, S1, S2, S3, S4, S5, S6);
    signal S : state_type; 
    signal count1: std_logic_vector(6 downto 0):= "0000000";
    signal count2: integer := 0; 
    signal count3: integer := 0; -- Sensor1,2 -> red2
    signal count4: integer := 0;
    
    
    component lcd16x2 is 
    port(clk : in STD_LOGIC;
               lcd_rs : out STD_LOGIC;
               lcd_e : out STD_LOGIC;
               lcd_rw : out STD_LOGIC;
               data : out STD_LOGIC_VECTOR (7 downto 0));
    end   component;
               
               
    begin
    
    lcd: lcd16x2 port map(clk,lcd_rs,lcd_e,lcd_rw,data);
    
    
    
        States : process (clk, reset)
            begin
            
                -- reset 3adi gedan byd5lna fel idle state
                if (reset = '1') then 
                    S <= S0;
                    count2 <=  100;
                    count1 <= "0000000";
                -- el code el le lazma    
                elsif (clk = '1' and clk'event) then
                    --count da by2olena nroo7 lel next light wala la2, 2ma yb2a 
                      --ba 0 yb2a 27na da5lna 5alsna el state el 27na feha       
                    if (count2 = 0) then
                        count2 <= 100;
                        if (count1 = "0000000") then
                            case (S) is
                                when S0 => 
                                           count1 <= "0000011";
                                           S <= S1;
				-- IDEAL CASE ALL OUTPUTS CAN BE 0
                                when S1 => 
                                           count1 <= "0111100";
                                           S <= S2;
                                when S2 => 
                                           count1 <= "0000011";
                                           S <= S3;
                                when S3 => 
                                           count1 <= "0000011";
                                           S <= S4;
                                when S4 => 
                                           count1 <= "0111100";
                                           S <= S5;
			                    when S5 => 
                                           count1 <= "0000011";
                                           S <= S6;
			                    when S6 => 
                                           count1 <= "0000011";
                                           S <= S1;
                                when others => S <= S0;
                            end case;
                        else
                            count1 <= count1 - '1';
                        end if;
                    else
                        count2 <= count2 - 1;
                    end if;
                    
                  end if;
        end process;
        
        Outputs : process(S)
        Begin 
            case (S) is
             when S0 =>                    
                                           G1 <= '1';
					                       Y1 <= '1';
                                           R1 <= '1';
					                       G2 <= '1';
					                       Y2 <= '1';
					                       R2 <= '1';
				-- IDEAL CASE ALL OUTPUTS CAN BE 0
                                       
                            when S1 => 
                                           G1 <= '0';
					                       Y1 <= '1';
                                           R1 <= '0';
					                       G2 <= '0';
					                       Y2 <= '0';
					                       R2 <='1';
                            when S2 => 
                                           G1 <= '1';
					                       Y1 <= '0';
                                           R1 <= '0';
					                       G2 <= '0';
					                       Y2 <= '0';
					                       R2 <='1';
                            when S3 => 
                                           G1 <= '0';
					                       Y1 <= '1';
                                           R1 <='0';
					                       G2 <= '0';
					                       Y2 <= '0';
					                       R2 <='1';
                            when S4 =>
    	                                   G1 <= '0';
					                       Y1 <= '0';
                                           R1 <= '1';
					                       G2 <= '0';
					                       Y2 <= '1';
					                       R2 <='0';
			                when S5 => 
    	                                   G1 <= '0';
					                       Y1 <= '0';
                                           R1 <= '1';
					                       G2 <= '1';
					                       Y2 <= '0';
					                       R2 <='0';
			                when S6 => 
    	                                   G1 <= '0';
					                       Y1 <= '0';
                                           R1 <= '1';
					                       G2 <= '0';
					                       Y2 <= '1';
					                       R2 <='0';
            end case;
        end process;
        
        sensor : process(clk)
            begin
                if (clk = '1' and clk'event) then --1
                
                
                        if (s=s4 or s=s5 or s=s6) then
                        
                            if (sensor1 = '0' or sensor2 = '0') then --Red2
                                count4 <= count4 + 1;
                            end if; --3
                            
                        elsif (s=s1 or s=s2 or s=s3) then
                            if (sensor3 = '0' or sensor4 = '0') then --Red1
                                count4 <= count4 + 1;
                            end if;
                             
                        else
                            count3 <= 0;
                            buzzer <= '0';
                            count4 <= 0;
                        end if;   
                        
                        if (count4 > 0) then
                            count4 <= 0; 
                            count3 <= 50;
                        end if; 
                            
                        if (count3 <= 0) then
                             buzzer <= '0';
                        else 
                             buzzer <= '1';
                             count3 <= count3 - 1;
                        end if;
                    
                end if; --1
             end process;   
        
end Behavioral;