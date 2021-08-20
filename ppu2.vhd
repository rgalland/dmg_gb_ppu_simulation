----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 07/16/2021 05:47:38 PM
-- Design Name: 
-- Module Name: ppu2 - Behavioral
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

entity ppu2 is
    Port ( 
        clk       : in STD_LOGIC;
        rst_n     : in STD_LOGIC;
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
        -- lcd pins -- signals to be inverted again to make sense (this must happen at pin level) 
        pin_st : out std_logic; -- hsync
        pin_s : out std_logic; -- vsync
        pin_ld : out std_logic_vector(1 downto 0); -- lcd data
        pin_cp : out std_logic; -- clock
        pin_cpl : out std_logic; -- latch
        pin_cpg : out std_logic -- control
    );
end ppu2;

architecture Behavioral of ppu2 is
    
    -- div reg
    signal ff04_din : std_logic_vector(7 downto 0) := X"00"; 
    
    -- ff40 contrl aliases
    --signal ff40_din : std_logic_vector(7 downto 0) := X"A1";    -- lcd en, win en, bg en
    alias lcd_en : std_logic is ff40_din(7);
    alias win_tm_alt : std_logic is ff40_din(6);
    alias win_en : std_logic is ff40_din(5);
    alias tile_data_sel : std_logic is ff40_din(4);
    alias bg_tm_alt : std_logic is ff40_din(3);
    
    -- ff41 stat
    signal ff41_dout : std_logic_vector(7 downto 0);
    alias lyc_detect : std_logic is ff41_dout(2);
    alias lcd_mode : std_logic_vector(1 downto 0) is ff41_dout(1 downto 0);
    
    -- ff44, ly
    signal ff44_dout : unsigned(7 downto 0);
    
    -- muxed output signals found on several pages.
    signal ma_sel : std_logic_vector(3 downto 0);
    signal ma : std_logic_vector(12 downto 0);
    
    --page 1
    signal rst_video, rst_video_n : std_logic; 
    signal clk1, clk2, clk2_n, clk4, clk5 : std_logic;
    -- page 6
    signal vypo : std_logic; 
    -- page 21
    signal myta_d, myta_q, nype_q, ropo_q, voga_q, vena_q_n, rutu_d : std_logic;
    signal l143, napo, napo_n, nype, pago, paly, pure_s, rupo, sela, tady, talu, talu_n, wego, wodu, wusa, xajo, xymu, xymu_n, xyvo : std_logic; 
    signal int_oam, int_hbl, int_vbl, int_vbl_buf : std_logic;
    signal line_count_us : unsigned(6 downto 0);    
    signal h_count_us, v_count_us : unsigned(7 downto 0);    
    -- page 24
    signal pygo_q, paho_q, line_zero : std_logic;
    signal loby, nafy, nyka, pofy, poky, pory, roxo, segu, segu_n, tofu, tomu, clkpipe : std_logic; 
    -- page 25
    signal cota, cota_n, xucy, wuko : std_logic;
    -- page 26, bg x, y and sp pri fifo
    signal bgy, bgx : unsigned(7 downto 0);
    signal obj_pri : std_logic;
    signal sp_pri_set_n, sp_pri_rst_n : std_logic_vector(7 downto 0);
    signal sp_pri_shifter : unsigned(7 downto 0);
    -- page 27
    signal sobu_q, suda_q, nopa_q, pyco_q, nunu_q, ryfa_q, rene_q, sovy_q, puxa_q, nyze_q, lyzu_q, lovy_q_n : std_logic;
    signal sobu_d, sary_d, pyco_d, ryfa_d, sovy_d, puxa_d : std_logic;
    signal lebo, lena, lury, lyry, mofu, mosu, myma, myvo, neta, nydy, nyxu, pecu, pore, potu, pova, rejo, repu, roxy, roze, seca, sary, sylo, taka, tave, tevo, veku, vetu, xaco, xofo, xuha : std_logic;
    signal winl_count : unsigned(4 downto 0);
    signal winh_count : unsigned(7 downto 0);
    signal bg_offset_count, bg_count : unsigned(2 downto 0);
    -- page 28
    signal anel_q : std_logic;
    signal dma_run, oam_addr_render, oam_addr_parse, clk3 : std_logic;
    signal sp_table_count_us : unsigned(5 downto 0);
    signal oam_a_cs, oam_b_cs : std_logic; 
    signal acyl, anom, asen, atej, azyb, besu, byva, feto, wyja : std_logic;
    signal oam_addr_o : std_logic_vector(6 downto 0);        
    -- page 29
    signal dezy_q, byba_q, doba_q, catu_d, ceno_q_n, seba_q, tobu_q, tyfo_q, tyfo_q_n, wuvu_q_n : std_logic;
    signal spr_match : std_logic;   -- '1' when oam data y value is < then 
    signal sp_bp_count_us : unsigned(2 downto 0);
    signal y_diff : unsigned(8 downto 0);   -- includes carry flag 
    signal sp_count_us : unsigned(3 downto 0);
    signal sprite_x_detect, sprite_x_priority, next_sprite_x_dis, sprite_x_priority_n, sprite_x_buff : std_logic_vector(9 downto 0);
    signal sp_ma : std_logic_vector(12 downto 0);
    signal sprite_data_clk, sprite_data_rst_n : std_logic_vector(9 downto 0);  

    type SPRITE_X_T is array (0 to 9) of std_logic_vector(7 downto 0);
    signal sprite_x : SPRITE_X_T;
    signal abez, aror, avap, buza, cake, catu, ceha, dyty, fepo, gava, puco, sycu, tacu, tame, texy, toma, tuly, tuvo, vonu, wuty, xado, xefy, xoce, xono, xupy, xupy_n, xyso : std_logic;
    -- page 30
    -- save oam sprite index to retrieve tile number and attributes during rendering
    signal sp_index, selected_sp : std_logic_vector(5 downto 0);
    type OAM_SPRITE_INDICES_T is array (0 to 9) of std_logic_vector(5 downto 0);
    signal OAM_SPRITE_INDICES : OAM_SPRITE_INDICES_T; 
    type OAM_SPRITE_YOFFSET_T is array (0 to 9) of std_logic_vector(3 downto 0);
    signal OAM_SPRITE_YOFFSET : OAM_SPRITE_YOFFSET_T;
    signal sp_y_offset : std_logic_vector(3 downto 0);     
    -- page 31
    signal oam_a_data0, oam_a_data1, oam_b_data0, oam_b_data1 : std_logic_vector(7 downto 0); 
    alias sp_priority : std_logic is oam_a_data1(7);
    alias sp_yflip : std_logic is oam_a_data1(6);
    alias sp_xflip : std_logic is oam_a_data1(5);
    alias sp_palette : std_logic is oam_a_data1(4);
    
    signal baxo, depo, gomo, yzos : std_logic;
    -- page 32
    signal bg_bp0_set_n, bg_bp0_reset_n, bg_bp1_set_n, bg_bp1_reset_n, bg_bp0, bg_bp1 : std_logic_vector(7 downto 0);
    signal bg_bp0_shifter, bg_bp1_shifter : unsigned(7 downto 0);
    -- page 33
    signal sp_bp0_set_n, sp_bp0_reset_n, sp_bp1_set_n, sp_bp1_reset_n, sp_bp0, sp_bp1 : std_logic_vector(7 downto 0);
    signal sp_bp0_shifter, sp_bp1_shifter : unsigned(7 downto 0);
    -- page 34
    signal obp0_px_n, obp1_px_n : std_logic; 
    signal sp_present_n, sp_pal_set_n, sp_pal_rst_n : std_logic_vector(7 downto 0);
    signal sp_pal_shifter : unsigned(7 downto 0);
    -- page 35
    signal tade, rajy, woxa, xula, bg_px_n, nypo, nuxo, numa, pobu : std_logic; 
    signal vyro, vata, volo, vugo, ledo, lyky, lopu, laru : std_logic; 
    signal bg_px, sp0_px, sp1_px : std_logic_vector(1 downto 0); 
    
    
    
    component sr_latch is
        Port ( s : in STD_LOGIC;
               r : in STD_LOGIC;
               q : out STD_LOGIC;
               q_n : out STD_LOGIC);
    end component;
    
    component rs_n_latch is
        Port ( r_n : in STD_LOGIC;
               s_n : in STD_LOGIC;
               q : out STD_LOGIC;
               q_n : out STD_LOGIC);
    end component;
    
    component dff is
        Port ( clk : in STD_LOGIC;
               rst_n : in STD_LOGIC;
               d : in STD_LOGIC;
               q : out STD_LOGIC;
               q_n : out STD_LOGIC);
    end component;

     
begin

    -- errors (L to X, A to G, checked so far):
    -- tevo is or_3 and not nor_3
    -- sacu is or_2 and not nor_2
    -- tuvo is nor_3 and not or_3  
    -- ajep is nand_2 and not and_2 
    -- aver is nand_2 and not and_2 
    -- bota is nand_3 and not nor_3
    -- bycu is nand_3 and not nor_3
    -- care is and_3 and not or_3
    -- xymu, besu, wusa, roxy, rupo, pynu, wuje and poky are SR latch and not or_2
    -- lony and taka are /R/S latch and not and_2    
    -- vypo is not p10_b?
    -- incorrectly labelled P10_B should be GND
    
      
            
    --page 1
    clk1 <= not clk;
    clk2 <= clk;
    clk2_n <= not clk2;
    clk4 <= not clk;
    clk5 <= clk;
    rst_video <= not rst_n;  
    
    -- page 6
    vypo <= '1';    -- this assumption seems correct
    
    --page 21
    tady <= atej nor tofu;
    
    VOGA_FF : dff
        Port map ( clk => clk2, rst_n => tady, d => wodu, q => voga_q, q_n => open);
    
    wego <= tofu or voga_q;
    
    XYMU_SRL : sr_latch 
        port map (s => wego, r => avap, q => open, q_n => xymu);
    
    xymu_n <= not xymu;
    
    xajo <= h_count_us(0) and h_count_us(3); 
    
    WUSA_SRL : sr_latch 
        port map (s => xajo, r => wego, q => wusa, q_n => open);
    
    pin_cp <= pova nor (clkpipe and wusa); 
    
    VENA_FF : dff
        Port map ( clk => wuvu_q_n, rst_n => rst_n, d => vena_q_n, q => talu, q_n => vena_q_n);
    
    process(l143, rst_video, talu)
    begin
        if (l143 or rst_video) = '1' then
            line_count_us <= (others => '0');
        elsif rising_edge(talu) then
            line_count_us <= line_count_us + 1;
        end if;
    end process;
    
    rutu_d <= line_count_us(6) and line_count_us(5) and line_count_us(4) and line_count_us(0);
    talu_n <= not talu;
    
    RUTU_FF : dff
        Port map ( clk => talu_n, rst_n => rst_n, d => rutu_d, q => l143, q_n => open);
    
    myta_d <= v_count_us(7) and v_count_us(4) and v_count_us(3) and v_count_us(0);
    
    NYPE_FF : dff
        Port map ( clk => talu, rst_n => rst_n, d => l143, q => nype_q, q_n => nype);
    
    xyvo <= v_count_us(7) and v_count_us(4);
    
    POPU_FF : dff
        Port map ( clk => nype_q, rst_n => rst_n, d => xyvo, q => int_vbl, q_n => open);
    
    NAPO_FF : dff
        Port map ( clk => int_vbl, rst_n => rst_n, d => napo_n, q => napo, q_n => napo_n);
    
    sela <= l143;
    pure_s <= not l143;  
    wodu <= not fepo and h_count_us(7) and h_count_us(5) and h_count_us(2) and h_count_us(1) and h_count_us(0);
    int_oam <= sela and not int_vbl;
    int_hbl <= wodu and not int_vbl;
    int_vbl_buf <= int_vbl;
    
    process(clkpipe, tady)
    begin
        if (tady) = '0' then
            h_count_us <= (others => '0');
        elsif rising_edge(clkpipe) then
            h_count_us <= h_count_us + 1;
        end if;
    end process;
    
    lcd_mode <= not (xymu nor acyl) & not (xymu nor int_vbl); 
    
    -- V count
    MYTA_FF : dff
        Port map ( clk => nype, rst_n => rst_n, d => myta_d, q => myta_q, q_n => open);
    
    process(l143, rst_video, myta_q)
    begin
        if (rst_video or myta_q) = '1' then
            v_count_us <= (others => '0');
        elsif rising_edge(l143) then
            v_count_us <= v_count_us + 1;
        end if;
    end process;
    
    paly <= '1' when ff45_din = v_count_us else '0';    
    
    ROPO_FF : dff
        Port map ( clk => talu, rst_n => rst_n, d => paly, q => ropo_q, q_n => open);
    
    pago <= '1'; -- only goes low when writing to ff41 
    
    RUPO_SR_LATCH : sr_latch  
        port map (s => ropo_q, r => pago, q_n => rupo);
        
    lyc_detect <= not rupo;
    
    -- page 23
    ff44_dout <= v_count_us;    
    
    -- page 24 - complete
    tofu <= not rst_n;
    loby <= not xymu;
    nafy <= mosu nor loby;
    
    NYKA_FF : dff
        Port map ( clk => clk2, rst_n => nafy, d => lyry, q => nyka, q_n => open);
    
    PORY_FF : dff
        Port map ( clk => clk2_n, rst_n => nafy, d => nyka, q => pory, q_n => open);
    
    PYGO_FF : dff
        Port map ( clk => clk2, rst_n => xymu, d => pory, q => pygo_q, q_n => open);
    
    tomu <= not sylo;
    
    POKY_SRL : sr_latch
        port map ( s => pygo_q, r => loby, q => poky);    
    
    segu <= not (sylo and poky and not (fepo or wodu or clk2_n));
    roxo <= not segu;
    clkpipe <= segu or roxy;
    
    PAHO_FF : dff
        Port map ( clk => roxo, rst_n => xymu, d => h_count_us(3), q => paho_q, q_n => open);
    
    pofy <= not (paho_q or tofu or (avap nor pofy));
    pin_st <= not pofy;
    
    line_zero <= '1' when v_count_us = 0 else '0';
    
    MEDA_FF : dff
        Port map ( clk => nype, rst_n => rst_n, d => line_zero, q => open, q_n => pin_s);
        
    pin_cpl <= not ((ff40_din(7) and not pure_s) or (not ff40_din(7) and ff04_din(0)));     
    
    -- page 25 only
    xucy <= neta nand pore;
    wuko <= pore nand potu;
    
    cota <= (acyl nand xyso) and (tacu nand tuvo); -- dma excluded
    
    -- page 26
    bgy <= v_count_us + ff42_din;
    bgx <= h_count_us + ff43_din;
    
    sp_pri_set_n <= sp_present_n nand (depo & depo & depo & depo & depo & depo & depo & depo);
    sp_pri_rst_n <= not sp_present_n nand not (depo & depo & depo & depo & depo & depo & depo & depo);
    
    SP_PRI_SHIFTER_proc : process(clkpipe, sp_pri_set_n, sp_pri_rst_n)
    begin
        if sp_pri_set_n /= X"FF" or sp_pri_rst_n /= X"FF" then
            for I in 0 to 7 loop
                if sp_pri_set_n(I) = '0' then
                    sp_pri_shifter(I) <= '1';
                end if;
                if sp_pri_rst_n(I) = '0' then
                    sp_pri_shifter(I) <= '0';
                end if;
            end loop;
        elsif rising_edge(clkpipe) then
            sp_pri_shifter <= shift_left(sp_pri_shifter, 1);
        end if;
    end process;   
    
    obj_pri <= sp_pri_shifter(7);
    
    -- page 27 - complete
    veku <= wuty nor tave;
    
    TAKA_RS_N : rs_n_latch 
        Port map( r_n => veku, s_n => seca, q => taka);
    
    sobu_d <= fepo and not tomu and lyry and not taka;    
    
    seca <= not (((not suda_q) and sobu_q) or (not rst_n) or atej);
    
    SOBU_FF : dff
        Port map ( clk => clk5, rst_n => vypo, d => sobu_d, q => sobu_q, q_n => open);
    
    SUDA_FF : dff
        Port map ( clk => clk4, rst_n => vypo, d => sobu_q, q => suda_q, q_n => open);
    
    sary_d <= '1' when (v_count_us = unsigned(ff4a_din)) and ff40_din(5) = '1' else '0';

    SARY_FF : dff
        Port map ( clk => talu, rst_n => rst_n, d => sary_d, q => sary, q_n => open);
    
    repu <= rst_video or int_vbl;
    
    REJO_SR_latch : sr_latch
        port map (s => sary, r => repu, q => rejo);  

    segu_n <= not segu;
    pyco_d <= '1' when (h_count_us = unsigned(ff4b_din)) and (rejo = '1') else '0';

    PYCO_FF : dff
        Port map ( clk => segu_n, rst_n => rst_n, d => pyco_d, q => pyco_q, q_n => open);
    
    NUNU_FF : dff
        Port map ( clk => clk2_n, rst_n => rst_n, d => pyco_q, q => nunu_q, q_n => open);
    
    xaco <= ff40_din(5) and not atej and rst_n; 
    xofo <= not xaco;      
    
    PYNU_SRL : sr_latch 
        port map (s => nunu_q, r => xofo, q => pore);

    NOPA_FF : dff
        Port map ( clk => clk2, rst_n => rst_n, d => pore, q => nopa_q, q_n => open);
    
    nyxu <= not (avap or mosu or tevo);
    mosu <= not nopa_q and pore;   
    tave <= xymu and not poky and nyka and pory;   
    sovy_d <= not (rst_video or pory or not (mosu or sovy_d)); 
    sylo <= not sovy_d; 
    
    RYFA_FF : dff
        Port map ( clk => segu, rst_n => xymu, d => ryfa_d, q => ryfa_q, q_n => open);
    
    RENE_FF : dff
        Port map ( clk => clk2, rst_n => xymu, d => ryfa_q, q => rene_q, q_n => open);
    
    SOVY_FF : dff
        Port map ( clk => clk2, rst_n => rst_n, d => sovy_d, q => sovy_q, q_n => open);
    
    vetu <= tevo and pore;
    
    tevo <= (rene_q nor not ryfa_q) or (sovy_q and sylo) or tave;
    
    process (vetu, xaco)
    begin
        if xaco = '0' then
            winl_count <= (others => '0');
        elsif rising_edge(vetu) then
            winl_count <= winl_count + 1;
        end if;
    end process;
    
    process (pore, int_vbl, rst_video)
    begin
        if (int_vbl or rst_video) = '1' then
            winh_count <= (others => '0');
        elsif falling_edge(pore) then
            winh_count <= winh_count + 1;
        end if;
    end process;
    
    ROXY_SRL : sr_latch
        port map (s => xymu_n, r => pova, q => roxy);
        
    roze <= '0' when bg_offset_count = "111" else '1';
    pecu <= roxo nand roze;
    ryfa_d <= roze nor pyco_d; 
    
    process (pecu, tevo, xymu_n)
    begin
        if (tevo nor xymu_n) = '0' then
            bg_offset_count <= (others => '0');
        elsif rising_edge(pecu) then
            bg_offset_count <= bg_offset_count + 1;
        end if;
    end process;
    
    puxa_d <= '1' when roxy = '1' and (bg_offset_count = ff43_din(2 downto 0)) else '0';
    
    PUXA_FF : dff
        Port map ( clk => roxo, rst_n => xymu, d => puxa_d, q => puxa_q, q_n => open);
    
    NYZE_FF : dff
        Port map ( clk => clk2_n, rst_n => xymu, d => puxa_q, q => nyze_q, q_n => open);
    
    pova <= not nyze_q and puxa_q;
    
    lyry <= nyxu and bg_count(0) and bg_count(2);
    lebo <= clk2 nand not lyry; 
    
    process (lebo, nyxu)
    begin
        if nyxu = '0' then
            bg_count <= (others => '0');
        elsif rising_edge(lebo) then
            bg_count <= bg_count + 1;
        end if;
    end process;
    
    LYZU_FF : dff
        Port map ( clk => clk2, rst_n => xymu, d => bg_count(0), q => lyzu_q, q_n => open);
    
    myvo <= not clk2;
    
    LOVY_FF : dff
        Port map ( clk => myvo, rst_n => nyxu, d => lyry, q => open, q_n => lovy_q_n);
    
    mofu <= not (loby or not bg_count(0) or lyzu_q) and not bg_count(1);
    potu <= lena and not bg_count(1) and not bg_count(2);
    neta <= lena and (not bg_count(1) nand not bg_count(2));
    nydy <= not (loby or not bg_count(0) or lyzu_q) and bg_count(1) and not bg_count(2);
    xuha <= bg_count(2);
    
    lury <= xymu and lovy_q_n;
    
    LONY_RS_N : rs_n_latch 
        Port map( r_n => lury, s_n => nyxu, q => lena);
        
    myma <= not lena;  
    
    -- page 28
    ANEL_FF : dff
        Port map ( clk => xupy_n, rst_n => rst_n, d => catu, q => anel_q, q_n => open);
    
    atej <= not ((not catu or anel_q) and abez);
    azyb <= not atej;
    anom <= rst_video nor atej;
    byva <= atej nor rst_video;
    
    asen <= rst_video or avap;
    
    BESU_SRL : sr_latch
        port map (s => catu, r => asen, q => besu);
    
    dma_run <= '0';
    acyl <= not dma_run and besu;
    oam_addr_render <= xymu nand (not dma_run);
    oam_addr_parse <= not acyl;
    
    
    -- on page 29
    WUVU_FF : dff
        Port map ( clk => clk1, rst_n => rst_n, d => wuvu_q_n, q => open, q_n => wuvu_q_n);
    
    xupy <= not wuvu_q_n;
    xupy_n <= not xupy;
    
    WOSU_FF : dff
        Port map ( clk => clk2, rst_n => rst_n, d => wuvu_q_n, q => open, q_n => xoce);
    
    xyso <= wuvu_q_n or xoce;
    
    CENO_FF : dff
        Port map ( clk => xupy, rst_n => rst_n, d => besu, q => ceha, q_n => ceno_q_n);
    
    buza <= xymu and ceno_q_n;
    abez <= not rst_video;
    dyty <= not (xoce and ceha and spr_match);  -- care is and_3 and not or_3
    
    catu_d <= sela and not xyvo;
        
    CATU_FF : dff
        Port map ( clk => xupy, rst_n => rst_n, d => catu_d, q => catu, q_n => open);
    
    BYBA_FF : dff
        Port map ( clk => xupy, rst_n => anom, d => feto, q => byba_q, q_n => open);
    
    DOBA_FF : dff
        Port map ( clk => clk2, rst_n => anom, d => byba_q, q => doba_q, q_n => open);
    
    avap <= not (doba_q or not anom or not byba_q);
    
    feto <= sp_table_count_us(5) and sp_table_count_us(2) and sp_table_count_us(1) and sp_table_count_us(0);
    gava <= feto or xupy;
    
    -- sprite y check
    -- inc 6 bit counter until 39 is reached when xupy goes high
    process(anom, gava)
    begin
        if anom = '0' then
            sp_table_count_us <= (others => '0');
        elsif rising_edge(gava) then
            sp_table_count_us <= sp_table_count_us + 1;
        end if;
    end process;
    
    oam_addr <= oam_addr_o; 
    
    oam_addr_o <= std_logic_vector(sp_table_count_us & "0") when oam_addr_parse = '0' else
                  selected_sp & "1"                         when oam_addr_render = '0' else  
                  "ZZZZZZZ";
    
    clk3 <= (acyl nand xoce) nand (tuvo nand tyfo_q_n);
    
    wuty <= tyfo_q_n nor (not (seba_q and vonu and sp_bp_count_us(0)));
    xefy <= not wuty;
    tuly <= sp_bp_count_us(1);
    texy <= (not xymu) nor (tuly nor vonu);
    xono <= baxo and texy;  -- baxo is D5 h flip attribute of sprite
    tuvo <= not (xymu_n or tuly or sp_bp_count_us(2));  
    tame <= sp_bp_count_us(0) nand sp_bp_count_us(2);
    toma <= clk4 nand tame;
    sycu <= not (not sp_bp_count_us(0) or not xymu or tyfo_q);
    tacu <= tyfo_q nand (not sp_bp_count_us(0));
    xado <= tuly nand sycu;
    puco <= sycu nand vonu;
    
    TYFO_FF : dff
        Port map ( clk => clk4, rst_n => vypo, d => sp_bp_count_us(0), q => tyfo_q, q_n => tyfo_q_n);
    
    SEBA_FF : dff
        Port map ( clk => clk4, rst_n => xymu, d => vonu, q => seba_q, q_n => open);
              
    process(seca, toma)
    begin
        if seca = '0' then
            sp_bp_count_us <= (others => '0');
        elsif rising_edge(toma) then
            sp_bp_count_us <= sp_bp_count_us + 1;
        end if;
    end process;   
    
    --sprite y comparator
    process(clk3)
    begin
        if rising_edge(clk3) then
            oam_b_data0 <= oam_din(7 downto 0); -- oam data is acutally inverted
        end if;
    end process;
    
    cota_n <= cota;
    
    process(cota_n)
    begin
        if rising_edge(cota_n) then
            oam_b_data1 <= oam_b_data0;
        end if;
    end process;
    
    y_diff <= resize(unsigned(oam_b_data1), 9) + resize((not v_count_us), 9); -- P10_B is '0' so no carry
    
    spr_match <= y_diff(8) and not y_diff(7) and not y_diff(6) and not y_diff(5) and not y_diff(4) and
                 (y_diff(3) or ff40_din(2));
    
    -- logic is normally inverted when going to chip
    sp_ma <= '0' & oam_b_data1(7 downto 1) & ((oam_b_data1(0) and not ff40_din(2)) or ((not sp_yflip xor sp_y_offset(3)) and ff40_din(2))) &
        (not (sp_yflip & sp_yflip & sp_yflip) xor std_logic_vector(sp_y_offset(2 downto 0))) & vonu;
    
    TOBU_FF : dff
        Port map ( clk => clk5, rst_n => xymu, d => tuly, q => tobu_q, q_n => open);
    
    VONU_FF : dff
        Port map ( clk => clk5, rst_n => xymu, d => tobu_q, q => vonu, q_n => open);
    
    DEZY_FF : dff
        Port map ( clk => clk1, rst_n => rst_n, d => dyty, q => dezy_q, q_n => open);
    
    cake <= (sp_count_us(1) and sp_count_us(3)) or dezy_q;
    
    process(azyb, cake)
    begin
        if azyb = '0' then
            sp_count_us <= (others => '0');
        elsif rising_edge(cake) then
            sp_count_us <= sp_count_us + 1;
        end if;
    end process;   
        
    process(dyty, sp_count_us)
        variable sp_detect : std_logic;
    begin
        for I in 0 to 9 loop
            if to_unsigned(I, 4) = sp_count_us then
                sp_detect := '0';
            else
                sp_detect := '1';
            end if;
            sprite_data_clk(I) <= dyty or sp_detect; 
        end loop;
    end process;
    
    -- sprite x priority
    aror <= ff40_din(1) and xymu and not ceha;
    
    X_PRIORITY_GEN : for I in 0 to 9 generate
        process(aror, sprite_x_detect, sprite_x_priority, next_sprite_x_dis)
        begin
            if I = 0 then
                sprite_x_priority(I) <= aror and sprite_x_detect(I);  
                next_sprite_x_dis(I) <= sprite_x_priority(I);
            else
                sprite_x_priority(I) <= aror and sprite_x_detect(I) and not next_sprite_x_dis(I - 1);
                next_sprite_x_dis(I) <= sprite_x_priority(I) or next_sprite_x_dis(I - 1);
            end if;
        end process; 
        
        sprite_data_rst_n(I) <= sprite_x_buff(I) nor (not byva);  
        
    end generate X_PRIORITY_GEN;
    
    fepo <= aror when sprite_x_detect /= "0000000000" else '0';    -- no sprites detected
    
    sprite_x_priority_n <= not sprite_x_priority;
    
    process(byva, wuty)
    begin
        if byva = '0' then
            sprite_x_buff <= (others => '0');
        elsif rising_edge(wuty) then
            sprite_x_buff <= sprite_x_priority;
        end if;
    end process;
      
    -- page 30
    process(xupy)
    begin
        if rising_edge(xupy) then
            sp_index <= oam_addr_o(6 downto 1);
        end if;
    end process;   
    
    SPITE_INDEX_GEN : for I in 0 to 9 generate
        process(sprite_data_clk(I))
        begin
            if rising_edge(sprite_data_clk(I)) then
                OAM_SPRITE_INDICES(I) <= selected_sp;  
                OAM_SPRITE_YOFFSET(I) <= sp_y_offset;
            end if;
        end process;
    end generate SPITE_INDEX_GEN;
    
    -- current sprite selection
    with (buza & not sprite_x_priority) select
        selected_sp <= sp_index when "01111111111",
                       OAM_SPRITE_INDICES(0) when "11111111110",
                       OAM_SPRITE_INDICES(1) when "11111111101",
                       OAM_SPRITE_INDICES(2) when "11111111011",
                       OAM_SPRITE_INDICES(3) when "11111110111",
                       OAM_SPRITE_INDICES(4) when "11111101111",
                       OAM_SPRITE_INDICES(5) when "11111011111",
                       OAM_SPRITE_INDICES(6) when "11110111111",
                       OAM_SPRITE_INDICES(7) when "11101111111",
                       OAM_SPRITE_INDICES(8) when "11011111111",
                       OAM_SPRITE_INDICES(9) when "10111111111",
                       "ZZZZZZ" when others;            
    
    with (fepo & not sprite_x_priority) select
        sp_y_offset <= std_logic_vector(y_diff(3 downto 0)) when "01111111111",
                       OAM_SPRITE_YOFFSET(0) when "11111111110",
                       OAM_SPRITE_YOFFSET(1) when "11111111101",
                       OAM_SPRITE_YOFFSET(2) when "11111111011",
                       OAM_SPRITE_YOFFSET(3) when "11111110111",
                       OAM_SPRITE_YOFFSET(4) when "11111101111",
                       OAM_SPRITE_YOFFSET(5) when "11111011111",
                       OAM_SPRITE_YOFFSET(6) when "11110111111",
                       OAM_SPRITE_YOFFSET(7) when "11101111111",
                       OAM_SPRITE_YOFFSET(8) when "11011111111",
                       OAM_SPRITE_YOFFSET(9) when "10111111111",
                       "ZZZZ" when others;            
    
    
    
    -- on page 31
    -- sprite x data acquisition and checks
    process(clk3)
    begin
        if rising_edge(clk3) then
            oam_a_data0 <= oam_din(15 downto 8); -- oam data is acutally inverted
        end if;
    end process;
    
    process(cota_n)
    begin
        if rising_edge(cota_n) then
            oam_a_data1 <= oam_a_data0;
        end if;
    end process;

    SPRITE_MATCH_GEN : for I in 0 to 9 generate
    begin    
        XPOS0_capture : process(sprite_data_rst_n(I), sprite_data_clk(I))
        begin
            if sprite_data_rst_n(I) = '0' then
                sprite_x(I) <= (others => '0');
            elsif rising_edge(sprite_data_clk(I)) then
                sprite_x(I) <= not oam_a_data1;
            end if;
        end process;  
         
        sprite_x_detect(I) <= '1' when not h_count_us = unsigned(sprite_x(I)) else '0';
    
    end generate SPRITE_MATCH_GEN;
    
    gomo <= oam_a_data1(4);
    baxo <= oam_a_data1(5);
    yzos <= oam_a_data1(6);
    depo <= oam_a_data1(7);
    
    -- page 32
    BG_BP1_capture : process(vypo, mofu)
    begin
        if vypo = '0' then
            bg_bp1 <= (others => '0');
        elsif falling_edge(mofu) then
            bg_bp1 <= vram_din;
        end if;
    end process;        
    
    bg_bp1_set_n <= bg_bp1 nand not (nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu);
    bg_bp1_reset_n <= not bg_bp1 nand not (nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu);
        
    BG_BP0_capture : process(vypo, nydy)
    begin
        if vypo = '0' then
            bg_bp0 <= (others => '0');
        elsif rising_edge(nydy) then
            bg_bp0 <= vram_din;
        end if;
    end process;
    
    bg_bp0_set_n <= bg_bp0 nand not (nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu);
    bg_bp0_reset_n <= not bg_bp0 nand not (nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu & nyxu);
    
    BG_BP1_SHIFTER_proc : process(clkpipe, bg_bp1_set_n, bg_bp1_reset_n)
    begin
        if bg_bp1_set_n /= X"FF" or bg_bp1_reset_n /= X"FF" then
            for I in 0 to 7 loop
                if bg_bp1_set_n(I) = '0' then
                    bg_bp1_shifter(I) <= '1';
                end if;
                if bg_bp1_reset_n(I) = '0' then
                    bg_bp1_shifter(I) <= '0';
                end if;
            end loop;
        elsif rising_edge(clkpipe) then
            bg_bp1_shifter <= shift_left(bg_bp1_shifter, 1);
        end if;
    end process;        
    
    BG_BP0_SHIFTER_proc : process(clkpipe, bg_bp0_set_n, bg_bp0_reset_n)
    begin
        if bg_bp0_set_n /= X"FF" or bg_bp0_reset_n /= X"FF" then
            for I in 0 to 7 loop
                if bg_bp0_set_n(I) = '0' then
                    bg_bp0_shifter(I) <= '1';
                end if;
                if bg_bp0_reset_n(I) = '0' then
                    bg_bp0_shifter(I) <= '0';
                end if;
            end loop;
        elsif rising_edge(clkpipe) then
            bg_bp0_shifter <= shift_left(bg_bp0_shifter, 1);
        end if;
    end process;  
    
    -- page 33
    SP_BP0_capture : process(xado)
    begin
        if rising_edge(xado) then
            if xono = '0' then
                sp_bp0 <= vram_din;
            else
                sp_bp0 <= vram_din(0) & vram_din(1) & vram_din(2) & vram_din(3) & vram_din(4) & vram_din(5) & vram_din(6) & vram_din(7);
            end if;
        end if;
    end process; 
    
    sp_bp0_set_n <= sp_bp0 nand sp_present_n;
    sp_bp0_reset_n <= not sp_bp0 nand sp_present_n;
           
    
    SP_BP1_capture : process(puco)
    begin
        if rising_edge(puco) then
            if xono = '0' then
                sp_bp1 <= vram_din;
            else
                sp_bp1 <= vram_din(0) & vram_din(1) & vram_din(2) & vram_din(3) & vram_din(4) & vram_din(5) & vram_din(6) & vram_din(7);
            end if;
        end if;
    end process;        
    
    sp_bp1_set_n <= sp_bp1 nand sp_present_n;
    sp_bp1_reset_n <= not sp_bp1 nand sp_present_n;
    
    SP_BP0_SHIFTER_proc : process(clkpipe, sp_bp0_set_n, sp_bp0_reset_n)
    begin
        if sp_bp0_set_n /= X"FF" or sp_bp0_reset_n /= X"FF" then
            for I in 0 to 7 loop
                if sp_bp0_set_n(I) = '0' then
                    sp_bp0_shifter(I) <= '1';
                end if;
                if sp_bp0_reset_n(I) = '0' then
                    sp_bp0_shifter(I) <= '0';
                end if;
            end loop;
        elsif rising_edge(clkpipe) then
            sp_bp0_shifter <= shift_left(sp_bp0_shifter, 1);
        end if;
    end process;
   
    SP_BP1_SHIFTER_proc : process(clkpipe, sp_bp1_set_n, sp_bp1_reset_n)
    begin
        if sp_bp1_set_n /= X"FF" or sp_bp1_reset_n /= X"FF" then
            for I in 0 to 7 loop
                if sp_bp1_set_n(I) = '0' then
                    sp_bp1_shifter(I) <= '1';
                end if;
                if sp_bp1_reset_n(I) = '0' then
                    sp_bp1_shifter(I) <= '0';
                end if;
            end loop;
        elsif rising_edge(clkpipe) then
            sp_bp1_shifter <= shift_left(sp_bp1_shifter, 1);
        end if;
    end process;      
        
    -- page 34
    
    sp_present_n <= std_logic_vector(sp_bp0_shifter nor sp_bp1_shifter) when xefy = '0' else X"00";
    sp_pal_set_n <= sp_present_n nand (sp_palette & sp_palette & sp_palette & sp_palette & sp_palette & sp_palette & sp_palette & sp_palette); 
    sp_pal_rst_n <= sp_present_n nand not (sp_palette & sp_palette & sp_palette & sp_palette & sp_palette & sp_palette & sp_palette & sp_palette); 
    
    SP_PAL_SHIFTER_proc : process(clkpipe, sp_pal_set_n, sp_pal_rst_n)
    begin
        if sp_pal_set_n /= X"FF" or sp_pal_rst_n /= X"FF" then
            for I in 0 to 7 loop
                if sp_pal_set_n(I) = '0' then
                    sp_pal_shifter(I) <= '1';
                end if;
                if sp_pal_rst_n(I) = '0' then
                    sp_pal_shifter(I) <= '0';
                end if;
            end loop;
        elsif rising_edge(clkpipe) then
            sp_pal_shifter <= shift_left(sp_pal_shifter, 1);
        end if;
    end process;
    
    obp0_px_n <= not sp_pal_shifter(7) nand bg_px_n;
    obp1_px_n <=     sp_pal_shifter(7) nand bg_px_n;
    
    -- page 35      
    
    tade <= ff40_din(0) and bg_bp1_shifter(7);
    rajy <= ff40_din(0) and bg_bp0_shifter(7);
    woxa <= ff40_din(1) and sp_bp1_shifter(7);
    xula <= ff40_din(1) and sp_bp0_shifter(7);
    
    bg_px_n <= not ((woxa nor xula) or (tade and obj_pri) or (rajy and obj_pri));
    
    nypo <= tade and rajy and not bg_px_n;
    nuxo <= not tade and rajy and not bg_px_n;
    numa <= tade and not rajy and not bg_px_n;
    pobu <= not tade and not rajy and not bg_px_n;
    
    bg_px <= ((ff47_din(1) and pobu) or (ff47_din(3) and nuxo) or (ff47_din(5) and numa) or (ff47_din(7) and nypo)) &
             ((ff47_din(0) and pobu) or (ff47_din(2) and nuxo) or (ff47_din(4) and numa) or (ff47_din(6) and nypo));
    
    vyro <= xula and woxa and not obp0_px_n;
    vata <= not xula and woxa and not obp0_px_n;
    volo <= xula and not woxa and not obp0_px_n;
    vugo <= not xula and not woxa and not obp0_px_n;
    
    sp0_px <= ((ff48_din(1) and vugo) or (ff48_din(3) and volo) or (ff48_din(5) and vata) or (ff48_din(7) and vyro)) &
              ((ff48_din(0) and vugo) or (ff48_din(2) and volo) or (ff48_din(4) and vata) or (ff48_din(6) and vyro));
             
    ledo <= xula and woxa and not obp1_px_n;
    laru <= not xula and woxa and not obp1_px_n;
    lyky <= xula and not woxa and not obp1_px_n;
    lopu <= not xula and not woxa and not obp1_px_n;
    
    sp1_px <= ((ff49_din(1) and lopu) or (ff49_din(3) and lyky) or (ff49_din(5) and laru) or (ff49_din(7) and ledo)) &
              ((ff49_din(0) and lopu) or (ff49_din(2) and lyky) or (ff49_din(4) and laru) or (ff49_din(6) and ledo));
                            
    pin_ld <= not (bg_px or sp0_px or sp1_px);
        
    -- tm addr, pore is bg or win
    ma_sel <= texy & pore & potu & neta;
    with ma_sel select
        ma <= 
            -- not pore and potu = bg tm
            "11" & bg_tm_alt & std_logic_vector(bgy(7 downto 3) & bgx(7 downto 3))                          when "0010",
            -- not pore and neta = bg bp 
            (tile_data_sel nor bg_bp1(7)) & bg_bp1 & std_logic_vector(bgy(2 downto 0)) & bg_count(2)        when "0001",
            -- pore and potu = win tm
            "11" & win_tm_alt & std_logic_vector(winh_count(7 downto 3) & winl_count)                       when "0110",
            -- pore and neta = win bp 
            (tile_data_sel nor bg_bp1(7)) & bg_bp1 & std_logic_vector(winh_count(2 downto 0)) & bg_count(2) when "0101",
            sp_ma                            when "1000" | "1001" | "1010" | "1011" | "1100" | "1101" | "1110" | "1111", 
            (others => 'Z') when others;
    
    vram_addr <= ma;
    

end Behavioral;
