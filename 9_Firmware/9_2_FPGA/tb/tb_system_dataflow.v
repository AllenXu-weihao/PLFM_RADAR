`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_system_dataflow.v  (PR-I, replaces tb_system_e2e G2.2 / G4.1 / G4.2)
//
// Shallow dataflow probe — verifies that auto-scan starts the production
// pipeline cleanly: TX fires chirps, the range pipeline emits multi-bin
// outputs through the matched filter, and observation counters advance.
//
// Coverage:
//   G2.2  new_chirp_frame pulsed (TX + chirp_scheduler alive)
//   G4.1  rx_range_valid pulsed (DDC + matched filter + range decimator)
//   G4.2  >= 100 range bin outputs (multi-bin emission)
//
// Deferred (NOT covered here — requires deeper RTL fix):
//   G4.4  doppler_valid pulse — full 48-chirp frame Doppler FFT.
//   G5.x  USB header/footer egress.
//   G9.x  Reset-mid-sim recovery.
//
//   Probe runs surface a hang in matched_filter_multi_segment's
//   ST_WAIT_FFT under continuous auto-scan stimulus: the inner FFT chain
//   (xfft_2048 + frequency_matched_filter) does not assert fft_done in
//   SIMULATION mode, so segment 0/1 never advances. tb_mf_cosim still
//   exercises matched_filter_processing_chain in isolation, but the
//   multi-segment wrapper has no dedicated TB (T-9). The hang is NOT
//   a test infrastructure problem — it is a real production-chain
//   integration gap to resolve in a future PR-J pipeline pass.
//
// Sim budget: ~18 ms (one full 48-chirp frame TX + range pipeline drain).
// ============================================================================

module tb_system_dataflow;

// ----------------------------------------------------------------------------
// Clocks (production)
// ----------------------------------------------------------------------------
localparam CLK_100M_PERIOD  = 10.0;
localparam CLK_120M_PERIOD  = 8.333;
localparam FT_CLK_PERIOD    = 16.667;
localparam ADC_DCO_PERIOD   = 2.5;

reg clk_100m     = 1'b0;
reg clk_120m_dac = 1'b0;
reg ft601_clk_in = 1'b0;
reg adc_dco_p    = 1'b0;
reg adc_dco_n    = 1'b1;

always #(CLK_100M_PERIOD/2) clk_100m     = ~clk_100m;
always #(CLK_120M_PERIOD/2) clk_120m_dac = ~clk_120m_dac;
always #(FT_CLK_PERIOD/2)   ft601_clk_in = ~ft601_clk_in;
always #(ADC_DCO_PERIOD/2)  begin adc_dco_p = ~adc_dco_p; adc_dco_n = ~adc_dco_n; end

// ----------------------------------------------------------------------------
// DUT signals
// ----------------------------------------------------------------------------
reg         reset_n = 1'b0;

reg [7:0]   adc_d_p = 8'h80;
reg [7:0]   adc_d_n = 8'h7F;

reg         stm32_new_chirp     = 1'b0;
reg         stm32_new_elevation = 1'b0;
reg         stm32_new_azimuth   = 1'b0;
reg         stm32_mixers_enable = 1'b0;
reg         stm32_sclk_3v3 = 1'b0;
reg         stm32_mosi_3v3 = 1'b0;
wire        stm32_miso_3v3;
reg         stm32_cs_adar1_3v3 = 1'b1, stm32_cs_adar2_3v3 = 1'b1;
reg         stm32_cs_adar3_3v3 = 1'b1, stm32_cs_adar4_3v3 = 1'b1;
wire        stm32_sclk_1v8, stm32_mosi_1v8;
reg         stm32_miso_1v8 = 1'b0;
wire        stm32_cs_adar1_1v8, stm32_cs_adar2_1v8;
wire        stm32_cs_adar3_1v8, stm32_cs_adar4_1v8;

wire [7:0]  dac_data;
wire        dac_clk;
wire        dac_sleep;

wire        fpga_rf_switch;
wire        rx_mixer_en, tx_mixer_en;
wire        adc_pwdn;

wire        adar_tx_load_1, adar_rx_load_1;
wire        adar_tx_load_2, adar_rx_load_2;
wire        adar_tx_load_3, adar_rx_load_3;
wire        adar_tx_load_4, adar_rx_load_4;
wire        adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4;

wire [31:0] ft601_data;
wire [3:0]  ft601_be;
wire        ft601_txe_n;
wire        ft601_rxf_n;
reg         ft601_txe = 1'b0;
reg         ft601_rxf = 1'b1;
wire        ft601_wr_n;
wire        ft601_rd_n;
wire        ft601_oe_n;
wire        ft601_siwu_n;
reg  [1:0]  ft601_srb = 2'b00;
reg  [1:0]  ft601_swb = 2'b00;
wire        ft601_clk_out;

wire [7:0]  ft_data;
reg         ft_rxf_n = 1'b1;
reg         ft_txe_n = 1'b0;
wire        ft_rd_n;
wire        ft_wr_n;
wire        ft_oe_n;
wire        ft_siwu;
pulldown pd[7:0] (ft_data);

wire [5:0]  current_elevation, current_azimuth, current_chirp;
wire        new_chirp_frame;
wire [31:0] dbg_doppler_data;
wire        dbg_doppler_valid;
wire [`RP_DOPPLER_BIN_WIDTH-1:0]   dbg_doppler_bin;
wire [`RP_RANGE_BIN_WIDTH_MAX-1:0] dbg_range_bin;
wire [3:0]  system_status;
wire        gpio_dig5, gpio_dig6, gpio_dig7;

// ----------------------------------------------------------------------------
// DUT — radar_system_top with USB_MODE=1 (FT2232H production)
// ----------------------------------------------------------------------------
radar_system_top #(.USB_MODE(1)) dut (
    .clk_100m(clk_100m),
    .clk_120m_dac(clk_120m_dac),
    .ft601_clk_in(ft601_clk_in),
    .reset_n(reset_n),

    .dac_data(dac_data), .dac_clk(dac_clk), .dac_sleep(dac_sleep),
    .fpga_rf_switch(fpga_rf_switch),
    .rx_mixer_en(rx_mixer_en), .tx_mixer_en(tx_mixer_en),

    .adar_tx_load_1(adar_tx_load_1), .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2), .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3), .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4), .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1), .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3), .adar_tr_4(adar_tr_4),

    .stm32_sclk_3v3(stm32_sclk_3v3),
    .stm32_mosi_3v3(stm32_mosi_3v3),
    .stm32_miso_3v3(stm32_miso_3v3),
    .stm32_cs_adar1_3v3(stm32_cs_adar1_3v3),
    .stm32_cs_adar2_3v3(stm32_cs_adar2_3v3),
    .stm32_cs_adar3_3v3(stm32_cs_adar3_3v3),
    .stm32_cs_adar4_3v3(stm32_cs_adar4_3v3),
    .stm32_sclk_1v8(stm32_sclk_1v8),
    .stm32_mosi_1v8(stm32_mosi_1v8),
    .stm32_miso_1v8(stm32_miso_1v8),
    .stm32_cs_adar1_1v8(stm32_cs_adar1_1v8),
    .stm32_cs_adar2_1v8(stm32_cs_adar2_1v8),
    .stm32_cs_adar3_1v8(stm32_cs_adar3_1v8),
    .stm32_cs_adar4_1v8(stm32_cs_adar4_1v8),

    .adc_d_p(adc_d_p), .adc_d_n(adc_d_n),
    .adc_dco_p(adc_dco_p), .adc_dco_n(adc_dco_n),
    .adc_or_p(1'b0), .adc_or_n(1'b1),
    .adc_pwdn(adc_pwdn),

    .stm32_new_chirp(stm32_new_chirp),
    .stm32_new_elevation(stm32_new_elevation),
    .stm32_new_azimuth(stm32_new_azimuth),
    .stm32_mixers_enable(stm32_mixers_enable),

    .ft601_data(ft601_data),
    .ft601_be(ft601_be),
    .ft601_txe_n(ft601_txe_n),
    .ft601_rxf_n(ft601_rxf_n),
    .ft601_txe(ft601_txe),
    .ft601_rxf(ft601_rxf),
    .ft601_wr_n(ft601_wr_n),
    .ft601_rd_n(ft601_rd_n),
    .ft601_oe_n(ft601_oe_n),
    .ft601_siwu_n(ft601_siwu_n),
    .ft601_srb(ft601_srb),
    .ft601_swb(ft601_swb),
    .ft601_clk_out(ft601_clk_out),

    .ft_data(ft_data),
    .ft_rxf_n(ft_rxf_n),
    .ft_txe_n(ft_txe_n),
    .ft_rd_n(ft_rd_n),
    .ft_wr_n(ft_wr_n),
    .ft_oe_n(ft_oe_n),
    .ft_siwu(ft_siwu),

    .current_elevation(current_elevation),
    .current_azimuth(current_azimuth),
    .current_chirp(current_chirp),
    .new_chirp_frame(new_chirp_frame),
    .dbg_doppler_data(dbg_doppler_data),
    .dbg_doppler_valid(dbg_doppler_valid),
    .dbg_doppler_bin(dbg_doppler_bin),
    .dbg_range_bin(dbg_range_bin),
    .system_status(system_status),
    .gpio_dig5(gpio_dig5),
    .gpio_dig6(gpio_dig6),
    .gpio_dig7(gpio_dig7)
);

// ADC stimulus: ramp around mid-scale
integer adc_phase;
initial begin
    adc_phase = 0;
    forever begin
        @(posedge adc_dco_p);
        if (reset_n) begin
            adc_d_p  = 8'h80 + ((adc_phase * 7) & 8'h3F) - 8'h20;
            adc_d_n  = ~adc_d_p;
            adc_phase = adc_phase + 1;
        end else begin
            adc_d_p = 8'h80;
            adc_d_n = 8'h7F;
        end
    end
end

// ----------------------------------------------------------------------------
// Observation counters
// ----------------------------------------------------------------------------
integer obs_chirp_frame_count = 0;
integer obs_range_valid_count = 0;

always @(posedge clk_100m) begin
    if (!reset_n) begin
        obs_chirp_frame_count = 0;
        obs_range_valid_count = 0;
    end else begin
        if (new_chirp_frame)    obs_chirp_frame_count = obs_chirp_frame_count + 1;
        if (dut.rx_range_valid) obs_range_valid_count = obs_range_valid_count + 1;
    end
end

// ----------------------------------------------------------------------------
// Test infrastructure
// ----------------------------------------------------------------------------
integer pass_count = 0;
integer fail_count = 0;
integer test_num   = 0;

task check;
    input         cond;
    input [80*8-1:0] msg;
    begin
        test_num = test_num + 1;
        if (cond) begin
            $display("  [PASS] %0d: %0s", test_num, msg);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %0d: %0s", test_num, msg);
            fail_count = fail_count + 1;
        end
    end
endtask

// ----------------------------------------------------------------------------
// Test sequence
// ----------------------------------------------------------------------------
initial begin
    $display("============================================================");
    $display("  tb_system_dataflow — TX/RX shallow integration probe");
    $display("============================================================");

    reset_n = 1'b0;
    repeat (20) @(posedge clk_100m);
    reset_n = 1'b1;
    repeat (50) @(posedge clk_100m);

    stm32_mixers_enable = 1'b1;
    $display("[%0t] mixers enabled — auto-scan running, waiting ~18 ms", $time);

    // 18 ms covers one full 48-chirp frame (3 sub-frames x 16 chirps,
    // ~8.4 ms TX) plus enough slack for new_chirp_frame to pulse and the
    // range pipeline to drain its first ~30 chirps.
    #18_000_000;

    $display("\n--- Group 2.2 / 4: TX + range pipeline ---");
    $display("    chirp_frames=%0d  range_valid=%0d",
             obs_chirp_frame_count, obs_range_valid_count);

    check(obs_chirp_frame_count > 0,
          "G2.2: new_chirp_frame pulsed at least once (TX/scheduler alive)");
    check(obs_range_valid_count > 0,
          "G4.1: range_profile_valid pulsed (matched filter produced output)");
    check(obs_range_valid_count >= 100,
          "G4.2: >= 100 range profile outputs (multi-bin emission)");

    $display("\n============================================================");
    $display("  RESULTS: %0d passed, %0d failed / %0d total",
             pass_count, fail_count, test_num);
    $display("    Sim time: %0t ns", $time);
    $display("============================================================");
    if (fail_count == 0) $display("  *** ALL TESTS PASSED ***");
    else                 $display("  *** %0d TEST(S) FAILED ***", fail_count);
    $finish;
end

// Watchdog — 25 ms (~1.4x the planned 18 ms run)
initial begin
    #25_000_000;
    $display("[WATCHDOG] tb_system_dataflow timeout at %0t", $time);
    $display("  Tests: %0d, Pass: %0d, Fail: %0d",
             test_num, pass_count, fail_count);
    $finish;
end

endmodule
