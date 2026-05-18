`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_usb_drivers_parity.v
//
// PR-AD (AD.2) cross-comparison parity TB. Instantiates BOTH
// usb_data_interface_ft2232h.v (8-bit, 60 MHz) and usb_data_interface.v
// (FT601, 32-bit+BE, 100 MHz) and feeds them identical stimulus.
//
// Each driver's egress byte stream is captured into a ring buffer (the
// FT601 stream is BE-reconstructed in lane order). The TB asserts:
//   - byte counts are equal
//   - per-index bytes are equal
//
// This is the byte-equality contract that makes the FT601 driver canonical
// with the FT2232H driver. Any future change to the FT2232H WR FSM that
// isn't mirrored in the FT601 driver will fail this TB at the diverging
// byte index.
//
// Scenarios:
//   A. Frame header only (stream_control = 0)             — 10 bytes
//   B. Status packet                                       — 34 bytes
//   C. Full frame with all 3 streams                       — 56330 bytes
// ============================================================================

module tb_usb_drivers_parity;
    // Common system clock (100 MHz radar domain)
    localparam CLK_PER       = 10.0;
    // Each driver has its own USB-side clock.
    localparam FT_CLK_PER    = 16.667;  // 60 MHz FT2232H ft_clk
    localparam FT601_CLK_PER = 10.0;    // 100 MHz FT601 ft601_clk_in

    reg clk           = 1'b0;
    reg ft_clk        = 1'b0;
    reg ft601_clk     = 1'b0;
    reg reset_n       = 1'b0;
    reg ft_reset_n    = 1'b0;
    reg ft601_reset_n = 1'b0;

    always #(CLK_PER/2)       clk       = ~clk;
    always #(FT_CLK_PER/2)    ft_clk    = ~ft_clk;
    always #(FT601_CLK_PER/2) ft601_clk = ~ft601_clk;

    // Shared radar stimulus
    reg [31:0] range_profile = 32'd0;
    reg        range_valid   = 1'b0;
    reg [15:0] doppler_real  = 16'd0;
    reg [15:0] doppler_imag  = 16'd0;
    reg        doppler_valid = 1'b0;
    reg [`RP_DETECT_CLASS_WIDTH-1:0] cfar_detect_class = `RP_DETECT_NONE;
    reg        cfar_valid    = 1'b0;
    reg [`RP_RANGE_BIN_WIDTH_MAX-1:0] range_bin_in   = 0;
    reg [`RP_DOPPLER_BIN_WIDTH-1:0]   doppler_bin_in = 0;
    reg                               frame_complete = 1'b0;

    // Shared control / status
    reg [5:0] stream_control     = 6'b000_111;
    reg [2:0] subframe_enable    = 3'b111;
    reg        status_request    = 1'b0;
    reg [15:0] status_cfar_threshold = 16'h1234;
    reg [5:0]  status_stream_ctrl    = 6'b000_111;
    reg [15:0] status_long_chirp   = 16'd0;
    reg [15:0] status_long_listen  = 16'd0;
    reg [15:0] status_guard        = 16'd0;
    reg [15:0] status_short_chirp  = 16'd0;
    reg [15:0] status_short_listen = 16'd0;
    reg [15:0] status_medium_chirp  = 16'd`RP_DEF_MEDIUM_CHIRP_CYCLES;
    reg [15:0] status_medium_listen = 16'd`RP_DEF_MEDIUM_LISTEN_CYCLES;
    reg [5:0]  status_chirps_per_elev = 6'd0;
    reg        status_chirps_mismatch = 1'b0;
    reg [4:0]  status_self_test_flags = 5'd0;
    reg [7:0]  status_self_test_detail = 8'd0;
    reg        status_self_test_busy = 1'b0;
    reg [3:0]  status_agc_current_gain = 4'd0;
    reg [7:0]  status_agc_peak_magnitude = 8'd0;
    reg [7:0]  status_agc_saturation_count = 8'd0;
    reg        status_agc_enable = 1'b0;
    reg        status_range_decim_watchdog = 1'b0;
    reg        status_ddc_cic_fir_overrun  = 1'b0;
    reg        status_beam_handshake_watchdog = 1'b0;
    reg [7:0]  status_cfar_alpha_soft       = `RP_DEF_CFAR_ALPHA_SOFT;
    reg [16:0] status_detect_threshold_soft = 17'h00ABC;
    reg [15:0] status_detect_count_cand     = 16'd42;

    // ---- FT2232H driver instance ----
    wire [7:0] ft_data;
    reg        ft_rxf_n = 1'b1;
    reg        ft_txe_n = 1'b0;
    wire       ft_rd_n, ft_wr_n, ft_oe_n, ft_siwu;
    pulldown pd_ft[7:0] (ft_data);

    usb_data_interface_ft2232h u_ft2232h (
        .clk(clk), .reset_n(reset_n), .ft_reset_n(ft_reset_n),
        .range_profile(range_profile), .range_valid(range_valid),
        .doppler_real(doppler_real), .doppler_imag(doppler_imag),
        .doppler_valid(doppler_valid),
        .cfar_detect_class(cfar_detect_class), .cfar_valid(cfar_valid),
        .range_bin_in(range_bin_in), .doppler_bin_in(doppler_bin_in),
        .frame_complete(frame_complete),
        .ft_data(ft_data), .ft_rxf_n(ft_rxf_n), .ft_txe_n(ft_txe_n),
        .ft_rd_n(ft_rd_n), .ft_wr_n(ft_wr_n), .ft_oe_n(ft_oe_n), .ft_siwu(ft_siwu),
        .ft_clk(ft_clk),
        .cmd_data(), .cmd_valid(), .cmd_opcode(), .cmd_addr(), .cmd_value(),
        .stream_control(stream_control),
        .subframe_enable(subframe_enable),
        .status_request(status_request),
        .status_cfar_threshold(status_cfar_threshold),
        .status_stream_ctrl(status_stream_ctrl),
        .status_long_chirp(status_long_chirp),
        .status_long_listen(status_long_listen),
        .status_guard(status_guard),
        .status_short_chirp(status_short_chirp),
        .status_short_listen(status_short_listen),
        .status_medium_chirp(status_medium_chirp),
        .status_medium_listen(status_medium_listen),
        .status_chirps_per_elev(status_chirps_per_elev),
        .status_chirps_mismatch(status_chirps_mismatch),
        .status_self_test_flags(status_self_test_flags),
        .status_self_test_detail(status_self_test_detail),
        .status_self_test_busy(status_self_test_busy),
        .status_agc_current_gain(status_agc_current_gain),
        .status_agc_peak_magnitude(status_agc_peak_magnitude),
        .status_agc_saturation_count(status_agc_saturation_count),
        .status_agc_enable(status_agc_enable),
        .status_range_decim_watchdog(status_range_decim_watchdog),
        .status_ddc_cic_fir_overrun(status_ddc_cic_fir_overrun),
        .status_beam_handshake_watchdog(status_beam_handshake_watchdog),
        .status_cfar_alpha_soft(status_cfar_alpha_soft),
        .status_detect_threshold_soft(status_detect_threshold_soft),
        .status_detect_count_cand(status_detect_count_cand)
    );

    // ---- FT601 driver instance ----
    wire [31:0] ft601_data;
    wire [3:0]  ft601_be;
    wire        ft601_txe_n_unused, ft601_rxf_n_unused;
    reg         ft601_txe = 1'b0;
    reg         ft601_rxf = 1'b1;
    wire        ft601_wr_n, ft601_rd_n, ft601_oe_n, ft601_siwu_n;
    wire        ft601_clk_out_unused;
    pulldown pd_ft601[31:0] (ft601_data);

    usb_data_interface u_ft601 (
        .clk(clk), .reset_n(reset_n), .ft601_reset_n(ft601_reset_n),
        .range_profile(range_profile), .range_valid(range_valid),
        .doppler_real(doppler_real), .doppler_imag(doppler_imag),
        .doppler_valid(doppler_valid),
        .cfar_detect_class(cfar_detect_class), .cfar_valid(cfar_valid),
        .range_bin_in(range_bin_in), .doppler_bin_in(doppler_bin_in),
        .frame_complete(frame_complete),
        .ft601_data(ft601_data), .ft601_be(ft601_be),
        .ft601_txe_n(ft601_txe_n_unused), .ft601_rxf_n(ft601_rxf_n_unused),
        .ft601_txe(ft601_txe), .ft601_rxf(ft601_rxf),
        .ft601_wr_n(ft601_wr_n), .ft601_rd_n(ft601_rd_n),
        .ft601_oe_n(ft601_oe_n), .ft601_siwu_n(ft601_siwu_n),
        .ft601_srb(2'd0), .ft601_swb(2'd0),
        .ft601_clk_out(ft601_clk_out_unused), .ft601_clk_in(ft601_clk),
        .cmd_data(), .cmd_valid(), .cmd_opcode(), .cmd_addr(), .cmd_value(),
        .stream_control(stream_control),
        .subframe_enable(subframe_enable),
        .status_request(status_request),
        .status_cfar_threshold(status_cfar_threshold),
        .status_stream_ctrl(status_stream_ctrl),
        .status_long_chirp(status_long_chirp),
        .status_long_listen(status_long_listen),
        .status_guard(status_guard),
        .status_short_chirp(status_short_chirp),
        .status_short_listen(status_short_listen),
        .status_medium_chirp(status_medium_chirp),
        .status_medium_listen(status_medium_listen),
        .status_chirps_per_elev(status_chirps_per_elev),
        .status_chirps_mismatch(status_chirps_mismatch),
        .status_self_test_flags(status_self_test_flags),
        .status_self_test_detail(status_self_test_detail),
        .status_self_test_busy(status_self_test_busy),
        .status_agc_current_gain(status_agc_current_gain),
        .status_agc_peak_magnitude(status_agc_peak_magnitude),
        .status_agc_saturation_count(status_agc_saturation_count),
        .status_agc_enable(status_agc_enable),
        .status_range_decim_watchdog(status_range_decim_watchdog),
        .status_ddc_cic_fir_overrun(status_ddc_cic_fir_overrun),
        .status_beam_handshake_watchdog(status_beam_handshake_watchdog),
        .status_cfar_alpha_soft(status_cfar_alpha_soft),
        .status_detect_threshold_soft(status_detect_threshold_soft),
        .status_detect_count_cand(status_detect_count_cand)
    );

    // ============================================================================
    // Byte capture rings — sized to fit a full frame (56330 bytes)
    // ============================================================================
    localparam integer RING_LEN = 65536;
    reg [7:0] a_bytes [0:RING_LEN-1];
    reg [7:0] b_bytes [0:RING_LEN-1];
    integer   a_count = 0;
    integer   b_count = 0;

    // FT2232H byte capture: 1 byte per ft_clk cycle when (!ft_wr_n && !ft_txe_n).
    always @(posedge ft_clk) begin
        if (!ft_wr_n && !ft_txe_n) begin
            if (a_count < RING_LEN)
                a_bytes[a_count] <= ft_data;
            a_count <= a_count + 1;
        end
    end

    // FT601 byte capture: up to 4 bytes per ft601_clk cycle (BE-masked).
    // Lane mapping: byte0 -> data[7:0] (BE[0]), byte1 -> data[15:8] (BE[1]),
    // byte2 -> data[23:16] (BE[2]), byte3 -> data[31:24] (BE[3]).
    integer b_idx;
    always @(posedge ft601_clk) begin
        if (!ft601_wr_n && !ft601_txe) begin
            b_idx = b_count;
            if (ft601_be[0]) begin
                if (b_idx < RING_LEN) b_bytes[b_idx] <= ft601_data[7:0];
                b_idx = b_idx + 1;
            end
            if (ft601_be[1]) begin
                if (b_idx < RING_LEN) b_bytes[b_idx] <= ft601_data[15:8];
                b_idx = b_idx + 1;
            end
            if (ft601_be[2]) begin
                if (b_idx < RING_LEN) b_bytes[b_idx] <= ft601_data[23:16];
                b_idx = b_idx + 1;
            end
            if (ft601_be[3]) begin
                if (b_idx < RING_LEN) b_bytes[b_idx] <= ft601_data[31:24];
                b_idx = b_idx + 1;
            end
            b_count <= b_idx;
        end
    end

    // ============================================================================
    // Bookkeeping
    // ============================================================================
    integer pass = 0;
    integer fail = 0;
    integer first_diff_idx;
    integer i;

    task check_b;
        input [127:0] tag;
        input         cond;
        begin
            if (cond) begin
                $display("[PASS] %0s", tag);
                pass = pass + 1;
            end else begin
                $display("[FAIL] %0s", tag);
                fail = fail + 1;
            end
        end
    endtask

    task assert_parity;
        input [127:0] scenario;
        input integer expected_count;
        begin
            $display("--- Parity check: %0s ---", scenario);
            $display("    FT2232H count = %0d", a_count);
            $display("    FT601 count   = %0d", b_count);
            check_b("count equal across drivers", a_count == b_count);
            check_b("count matches expected",    a_count == expected_count);

            // Find first byte difference (if any)
            first_diff_idx = -1;
            for (i = 0; i < a_count && i < b_count; i = i + 1) begin
                if (a_bytes[i] !== b_bytes[i] && first_diff_idx == -1)
                    first_diff_idx = i;
            end
            if (first_diff_idx != -1) begin
                $display("    [FAIL] first byte mismatch at index %0d: ft2232h=0x%02h ft601=0x%02h",
                         first_diff_idx, a_bytes[first_diff_idx], b_bytes[first_diff_idx]);
                fail = fail + 1;
            end else begin
                $display("    [PASS] byte streams identical");
                pass = pass + 1;
            end
        end
    endtask

    task wait_clk;
        input integer n;
        integer j;
        begin
            for (j = 0; j < n; j = j + 1) @(posedge clk);
        end
    endtask

    task reset_capture;
        begin
            a_count = 0;
            b_count = 0;
        end
    endtask

    initial begin
        $display("\n========== tb_usb_drivers_parity ==========");
        reset_n       = 1'b0;
        ft_reset_n    = 1'b0;
        ft601_reset_n = 1'b0;
        wait_clk(15);
        reset_n       = 1'b1;
        ft_reset_n    = 1'b1;
        ft601_reset_n = 1'b1;
        wait_clk(40);

        // --------------------------------------------------------------
        // SCENARIO A — frame header only (no stream bodies)
        // Expected total = 9 (header) + 1 (footer) = 10 bytes per driver.
        // --------------------------------------------------------------
        $display("\n[SCENARIO A] Frame header only");
        stream_control = 6'b000_000;
        wait_clk(50);
        reset_capture;
        @(posedge clk);
        frame_complete = 1'b1;
        @(posedge clk);
        frame_complete = 1'b0;
        wait_clk(500);  // both drivers drain
        assert_parity("scenario A (header+footer)", 10);

        // --------------------------------------------------------------
        // SCENARIO B — status packet
        // Expected total = 34 bytes per driver.
        // --------------------------------------------------------------
        $display("\n[SCENARIO B] Status packet");
        wait_clk(50);
        reset_capture;
        @(posedge clk);
        status_request = 1'b1;
        @(posedge clk);
        status_request = 1'b0;
        wait_clk(800);
        assert_parity("scenario B (status pkt)", 34);

        // --------------------------------------------------------------
        // SCENARIO C — full frame with all 3 streams enabled
        // Expected total = 9 + 1024 + 49152 + 6144 + 1 = 56330 bytes.
        // BRAMs zero-init in SIMULATION mode so content matches across
        // drivers (both emit 0x00 for every cell).
        // --------------------------------------------------------------
        $display("\n[SCENARIO C] Full frame (all 3 streams)");
        stream_control = 6'b000_111;
        wait_clk(50);
        reset_capture;
        @(posedge clk);
        frame_complete = 1'b1;
        @(posedge clk);
        frame_complete = 1'b0;
        // FT2232H @ 60 MHz needs ~56330 ft_clk cycles ≈ 940 µs.
        // FT601 @ 100 MHz needs ~56330 ft601_clk ≈ 564 µs.
        // Use clk-domain wait covering the slower driver: ~94000 clk + slack.
        wait_clk(150_000);
        assert_parity("scenario C (full frame)",
                      `RP_FRAME_HDR_BYTES
                      + `RP_NUM_RANGE_BINS * 2
                      + `RP_NUM_RANGE_BINS * `RP_NUM_DOPPLER_BINS * 2
                      + (`RP_NUM_RANGE_BINS * `RP_NUM_DOPPLER_BINS * 2) / 8
                      + 1);

        $display("\n-----------------------------------------------------------");
        $display("RESULTS: %0d PASS, %0d FAIL", pass, fail);
        $display("-----------------------------------------------------------");
        if (fail == 0) $display("[OVERALL PASS]"); else $display("[OVERALL FAIL]");
        $finish;
    end

    initial begin
        #50_000_000;
        $display("[TIMEOUT] tb_usb_drivers_parity watchdog");
        $finish;
    end

endmodule
