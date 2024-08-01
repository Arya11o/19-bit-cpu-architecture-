library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity CPU is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        instruction : in  std_logic_vector(18 downto 0);
        data_in    : in  std_logic_vector(18 downto 0);
        data_out   : out std_logic_vector(18 downto 0);
        addr       : out std_logic_vector(18 downto 0);
        mem_read   : out std_logic;
        mem_write  : out std_logic
    );
end CPU;

architecture Behavioral of CPU is

    signal r1, r2, r3       : std_logic_vector(18 downto 0); -- General-purpose registers
    signal pc               : std_logic_vector(18 downto 0); -- Program Counter
    signal sp               : std_logic_vector(18 downto 0); -- Stack Pointer
    signal opcode           : std_logic_vector(5 downto 0);  -- Opcode extraction
    signal immediate        : std_logic_vector(18 downto 0); -- Immediate data
    signal alu_result       : std_logic_vector(18 downto 0);
    signal alu_op           : std_logic_vector(2 downto 0);
    signal fft_start, enc_start, dec_start : std_logic;
    signal fft_done, enc_done, dec_done    : std_logic;
    signal fft_input, enc_input, dec_input : std_logic_vector(18 downto 0);
    signal fft_output, enc_output, dec_output : std_logic_vector(18 downto 0);

    component ALU
        Port (
            A      : in  std_logic_vector(18 downto 0);
            B      : in  std_logic_vector(18 downto 0);
            op     : in  std_logic_vector(2 downto 0);
            result : out std_logic_vector(18 downto 0)
        );
    end component;

begin

    -- ALU Instantiation
    ALU_inst : ALU
        Port map (
            A => r2,
            B => r3,
            op => alu_op,
            result => alu_result
        );

    -- Main Control Process
    process(clk, rst)
    begin
        if rst = '1' then
            r1 <= (others => '0');
            r2 <= (others => '0');
            r3 <= (others => '0');
            pc <= (others => '0');
            sp <= (others => '0');
            data_out <= (others => '0');
            mem_read <= '0';
            mem_write <= '0';
            fft_start <= '0';
            enc_start <= '0';
            dec_start <= '0';
        elsif rising_edge(clk) then
            opcode <= instruction(18 downto 13);
            immediate <= instruction(12 downto 0); -- Assuming immediate is part of the instruction
            case opcode is
                -- Arithmetic Instructions
                when "000000" => -- ADD
                    alu_op <= "000";
                    r1 <= alu_result;
                when "000001" => -- SUB
                    alu_op <= "001";
                    r1 <= alu_result;
                when "000010" => -- MUL
                    alu_op <= "010";
                    r1 <= alu_result;
                when "000011" => -- DIV
                    alu_op <= "011";
                    r1 <= alu_result;
                when "000100" => -- INC
                    r1 <= std_logic_vector(unsigned(r1) + 1);
                when "000101" => -- DEC
                    r1 <= std_logic_vector(unsigned(r1) - 1);

                -- Logical Instructions
                when "000110" => -- AND
                    alu_op <= "100";
                    r1 <= alu_result;
                when "000111" => -- OR
                    alu_op <= "101";
                    r1 <= alu_result;
                when "001000" => -- XOR
                    alu_op <= "110";
                    r1 <= alu_result;
                when "001001" => -- NOT
                    r1 <= not r2;

                -- Control Flow Instructions
                when "001010" => -- JMP
                    pc <= immediate;
                when "001011" => -- BEQ
                    if r1 = r2 then
                        pc <= immediate;
                    end if;
                when "001100" => -- BNE
                    if r1 /= r2 then
                        pc <= immediate;
                    end if;
                when "001101" => -- CALL
                    sp <= std_logic_vector(unsigned(sp) - 1);
                    -- Store current PC to stack (assume memory interface for stack)
                    addr <= sp;
                    mem_write <= '1';
                    data_out <= pc;
                    pc <= immediate;
                when "001110" => -- RET
                    sp <= std_logic_vector(unsigned(sp) + 1);
                    -- Load PC from stack (assume memory interface for stack)
                    addr <= sp;
                    mem_read <= '1';

                -- Memory Access Instructions
                when "001111" => -- LD
                    addr <= immediate;
                    mem_read <= '1';
                    r1 <= data_in;
                when "010000" => -- ST
                    addr <= immediate;
                    mem_write <= '1';
                    data_out <= r1;

                -- Custom Instructions
                when "100000" => -- FFT
                    fft_input <= r1; -- Assuming r1 holds the data for FFT input
                    fft_start <= '1';
                    if fft_done = '1' then
                        addr <= immediate; -- Store FFT result at immediate address
                        data_out <= fft_output;
                        mem_write <= '1';
                        fft_start <= '0';
                    end if;

                when "100001" => -- ENC
                    enc_input <= r1; -- Assuming r1 holds the data for encryption input
                    enc_start <= '1';
                    if enc_done = '1' then
                        addr <= immediate; -- Store encrypted data at immediate address
                        data_out <= enc_output;
                        mem_write <= '1';
                        enc_start <= '0';
                    end if;

                when "100010" => -- DEC
                    dec_input <= r1; -- Assuming r1 holds the data for decryption input
                    dec_start <= '1';
                    if dec_done = '1' then
                        addr <= immediate; -- Store decrypted data at immediate address
                        data_out <= dec_output;
                        mem_write <= '1';
                        dec_start <= '0';
                    end if;

                when others =>
                    -- Default case or handle other instructions
                    null;
            end case;
        end if;
    end process;

    -- Instantiate external FFT, Encryption, and Decryption components
    -- These components are assumed to be external IP cores or modules.

    FFT_Module : entity work.FFT_Unit
        port map (
            clk => clk,
            rst => rst,
            start => fft_start,
            input_data => fft_input,
            output_data => fft_output,
            done => fft_done
        );

    ENC_Module : entity work.ENC_Unit
        port map (
            clk => clk,
            rst => rst,
            start => enc_start,
            input_data => enc_input,
            output_data => enc_output,
            done => enc_done
        );

    DEC_Module : entity work.DEC_Unit
        port map (
            clk => clk,
            rst => rst,
            start => dec_start,
            input_data => dec_input,
            output_data => dec_output,
            done => dec_done
        );

end Behavioral;
