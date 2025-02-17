library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ENC_Unit is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;
        input_data : in  std_logic_vector(18 downto 0);
        output_data: out std_logic_vector(18 downto 0);
        done       : out std_logic
    );
end ENC_Unit;

architecture Behavioral of ENC_Unit is
begin
    process(clk, rst)
    begin
        if rst = '1' then
            output_data <= (others => '0');
            done <= '0';
        elsif rising_edge(clk) then
            if start = '1' then
                -- Placeholder: Encryption computation would go here
                output_data <= not input_data; -- For simplicity, just inverting the input data
                done <= '1';
            else
                done <= '0';
            end if;
        end if;
    end process;
end Behavioral;
