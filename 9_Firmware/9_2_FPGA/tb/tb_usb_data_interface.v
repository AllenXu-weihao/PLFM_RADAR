`timescale 1ns / 1ps
`include "radar_params.vh"

// ============================================================================
// tb_usb_data_interface.v
//
// PR-AD (AD.2) v2 bulk protocol unit TB for usb_data_interface.v (FT601).
// Mirrors tb_usb_protocol_v2.v's structure but adapted for the FT601 32-bit
// data bus + BE byte-enable. Byte stream reconstructed from BE lanes is
// asserted byte-equal to what the FT2232H driver would emit, by design.
//
//   1. Opcode 0x2D (host_cfar_alpha_soft) round-trip on the RX path.
//   2. Bulk frame header v2 — byte0=0xAA, byte1=0x02 (version), byte2 flags,
//      bytes3-8 = frame_num/range_bins/doppler_bins.
//   3. Status packet length 34 bytes (M-5), word[6] CFAR telemetry, word[7]
//      medium_chirp/medium_listen.
//   4. Full-frame length consistency with all 3 streams enabled (PR-G trim).
//   5. MEDIUM ladder opcodes 0x17 / 0x18 round-trip.
// ============================================================================

module tb_usb_data_interface;
    localparam CLK_PER       = 10.0;  // 100 MHz radar clk
    localparam FT_CLK_PER    = 10.0;  // 100 MHz ft601_clk_in (asynchronous)

    reg clk          = 1'b0;
    reg ft601_clk_in = 1'b0;
    reg reset_n      = 1'b0;
    reg ft601_reset_n = 1'b0;

    // Radar inputs (clk domain)
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

    // FT601 interface signals
    wire [31:0] ft601_data;
    wire [3:0]  ft601_be;
    wire        ft601_txe_n;     // VESTIGIAL output (tied to 1)
    wire        ft601_rxf_n;     // VESTIGIAL output (tied to 1)
    reg         ft601_txe = 1'b0;  // active-low: 0 = FIFO has space
    reg         ft601_rxf = 1'b1;  // active-low: 0 = host data available
    wire        ft601_wr_n;
    wire        ft601_rd_n;
    wire        ft601_oe_n;
    wire        ft601_siwu_n;
    reg  [1:0]  ft601_srb = 2'd0;
    reg  [1:0]  ft601_swb = 2'd0;
    wire        ft601_clk_out;

    pulldown pd[31:0] (ft601_data);

    // Host-to-FPGA bus driver for the RD path
    reg [31:0] host_data_drive   = 32'd0;
    reg        host_data_drive_en = 1'b0;
    assign ft601_data = host_data_drive_en ? host_data_drive : 32'hzzzz_zzzz;

    wire [31:0] cmd_data;
    wire        cmd_valid;
    wire [7:0]  cmd_opcode;
    wire [7:0]  cmd_addr;
    wire [15:0] cmd_value;

    // PR-G v2 stream control — enable all 3 streams (range|doppler|cfar).
    reg [5:0] stream_control     = 6'b000_111;
    reg [5:0] status_stream_ctrl = 6'b000_111;
    // PR-U / M-8: production 3-PRI ladder.
    reg [2:0] subframe_enable    = 3'b111;

    reg        status_request = 1'b0;
    reg [15:0] status_cfar_threshold = 16'h1234;
    reg [15:0] status_long_chirp = 16'd0;
    reg [15:0] status_long_listen = 16'd0;
    reg [15:0] status_guard = 16'd0;
    reg [15:0] status_short_chirp = 16'd0;
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
    reg [7:0]  status_cfar_alpha_soft       = `RP_DEF_CFAR_ALPHA_SOFT;  // 0x18
    reg [16:0] status_detect_threshold_soft = 17'h00ABC;
    reg [15:0] status_detect_count_cand     = 16'd42;

    integer pass = 0;
    integer fail = 0;

    always #(CLK_PER/2)    clk           = ~clk;
    always #(FT_CLK_PER/2) ft601_clk_in  = ~ft601_clk_in;

    usb_data_interface u_dut (
        .clk(clk),
        .reset_n(reset_n),
        .ft601_reset_n(ft601_reset_n),
        .range_profile(range_profile),
        .range_valid(range_valid),
        .doppler_real(doppler_real),
        .doppler_imag(doppler_imag),
        .doppler_valid(doppler_valid),
        .cfar_detect_class(cfar_detect_class),
        .cfar_valid(cfar_valid),
        .range_bin_in(range_bin_in),
        .doppler_bin_in(doppler_bin_in),
        .frame_complete(frame_complete),
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
        .ft601_clk_in(ft601_clk_in),
        .cmd_data(cmd_data),
        .cmd_valid(cmd_valid),
        .cmd_opcode(cmd_opcode),
        .cmd_addr(cmd_addr),
        .cmd_value(cmd_value),
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
    // BE-aware byte capture
    //
    // FT601 emits a 32-bit word + 4-bit BE per ft601_clk cycle when
    // (!ft601_wr_n && !ft601_txe). Each enabled BE lane carries one stream
    // byte. Convention: byte 0 of stream -> ft601_data[7:0] (BE[0]),
    // byte 1 -> [15:8] (BE[1]), byte 2 -> [23:16] (BE[2]), byte 3 -> [31:24]
    // (BE[3]). Reconstructed byte stream must match what FT2232H emits
    // byte-for-byte on the same stimulus (cross-comparison TB asserts this).
    // ============================================================================
    reg [7:0]  egress_bytes [0:35];
    integer    egress_count = 0;
    integer    capture_idx;
    always @(posedge ft601_clk_in) begin
        if (!ft601_wr_n && !ft601_txe) begin
            capture_idx = egress_count;
            if (ft601_be[0]) begin
                if (capture_idx < 36) egress_bytes[capture_idx] <= ft601_data[7:0];
                capture_idx = capture_idx + 1;
            end
            if (ft601_be[1]) begin
                if (capture_idx < 36) egress_bytes[capture_idx] <= ft601_data[15:8];
                capture_idx = capture_idx + 1;
            end
            if (ft601_be[2]) begin
                if (capture_idx < 36) egress_bytes[capture_idx] <= ft601_data[23:16];
                capture_idx = capture_idx + 1;
            end
            if (ft601_be[3]) begin
                if (capture_idx < 36) egress_bytes[capture_idx] <= ft601_data[31:24];
                capture_idx = capture_idx + 1;
            end
            egress_count <= capture_idx;
        end
    end

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

    task wait_clk;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) @(posedge clk);
        end
    endtask

    // 4-byte command bus driver (host -> FPGA, ft601_clk domain).
    // FT601 RD FSM reads one 32-bit word per transaction; cmd word is
    // packed per the FT601 RX layout: {opcode[31:24], addr[23:16], value[15:0]}.
    task send_cmd;
        input [7:0]  op;
        input [7:0]  addr;
        input [15:0] val;
        begin
            @(posedge ft601_clk_in); #1;
            ft601_rxf          = 1'b0;
            host_data_drive    = {op, addr, val};
            host_data_drive_en = 1'b1;
            @(posedge ft601_clk_in); #1;
            @(posedge ft601_clk_in); #1;
            @(posedge ft601_clk_in); #1;
            @(posedge ft601_clk_in); #1;
            ft601_rxf          = 1'b1;
            host_data_drive_en = 1'b0;
            wait_clk(20);
        end
    endtask

    initial begin
        $display("\n========== tb_usb_data_interface (FT601 v2 bulk) ==========");
        // Reset
        reset_n       = 1'b0;
        ft601_reset_n = 1'b0;
        wait_clk(10);
        reset_n       = 1'b1;
        ft601_reset_n = 1'b1;
        wait_clk(20);

        // -------------------------------------------------------------
        // TEST 1: Opcode 0x2D (host_cfar_alpha_soft) round trip
        // -------------------------------------------------------------
        $display("\n[TEST 1] Opcode 0x2D (cfar_alpha_soft) round trip");
        send_cmd(`RP_OP_CFAR_ALPHA_SOFT, 8'h00, 16'h0024);
        check_b("T1.1: cmd_opcode=0x2D",         cmd_opcode == 8'h2D);
        check_b("T1.2: cmd_value lower 8b=0x24", cmd_value[7:0] == 8'h24);

        // -------------------------------------------------------------
        // TEST 2: Frame header v2 — 9 bytes, byte1=0x02
        // -------------------------------------------------------------
        $display("\n[TEST 2] Frame header v2 emission");
        stream_control = 6'b000_000;  // skip data sections
        wait_clk(50);
        egress_count = 0;
        @(posedge clk);
        frame_complete = 1'b1;
        @(posedge clk);
        frame_complete = 1'b0;
        wait_clk(200);  // drain
        check_b("T2.1: byte0 = 0xAA",         egress_bytes[0] == 8'hAA);
        check_b("T2.2: byte1 = 0x02 (ver)",   egress_bytes[1] == `RP_USB_PROTOCOL_VERSION);
        check_b("T2.3: byte2 = {00, sf=111, stream=0} = 0x38",
                egress_bytes[2] == 8'h38);
        check_b("T2.4: byte3 = fn[15:8]=0",   egress_bytes[3] == 8'h00);
        check_b("T2.5: byte4 = fn[7:0]=0",    egress_bytes[4] == 8'h00);
        check_b("T2.6: byte5/6 = range_bins=512",
                {egress_bytes[5], egress_bytes[6]} == 16'd512);
        check_b("T2.7: byte7/8 = doppler_bins=48",
                {egress_bytes[7], egress_bytes[8]} == 16'd48);
        check_b("T2.8: byte9 = footer 0x55",  egress_bytes[9] == 8'h55);

        // -------------------------------------------------------------
        // TEST 3: Status packet length = 34 bytes (M-5)
        // -------------------------------------------------------------
        $display("\n[TEST 3] Status packet length 34B + word[6]/word[7]");
        egress_count = 0;
        @(posedge clk);
        status_request = 1'b1;
        @(posedge clk);
        status_request = 1'b0;
        wait_clk(400);
        check_b("T3.1: byte0 = 0xBB (status header)", egress_bytes[0] == 8'hBB);
        check_b("T3.2: byte33 = 0x55 (footer)",       egress_bytes[33] == 8'h55);
        check_b("T3.3: status_words[6] count_cand[15:8]=0",  egress_bytes[25] == 8'h00);
        check_b("T3.4: status_words[6] count_cand[7:0]=42",  egress_bytes[26] == 8'd42);
        check_b("T3.5: status_words[6] thr_soft[15:8]=0x0A", egress_bytes[27] == 8'h0A);
        check_b("T3.6: status_words[6] thr_soft[7:0]=0xBC",  egress_bytes[28] == 8'hBC);
        // alpha_soft (0x18) packed into word[4][9:2] -> byte at index 20.
        check_b("T3.7: status_words[4][7:0] = alpha_soft<<2 = 0x60 (alpha=0x18)",
                egress_bytes[20] == 8'h60);
        // M-5: status_words[7] = {medium_chirp (0x01F4), medium_listen (0x3CF0)}.
        check_b("T3.8: status_words[7] medium_chirp[15:8]=0x01",  egress_bytes[29] == 8'h01);
        check_b("T3.9: status_words[7] medium_chirp[7:0]=0xF4",   egress_bytes[30] == 8'hF4);
        check_b("T3.10: status_words[7] medium_listen[15:8]=0x3C", egress_bytes[31] == 8'h3C);
        check_b("T3.11: status_words[7] medium_listen[7:0]=0xF0",  egress_bytes[32] == 8'hF0);

        // -------------------------------------------------------------
        // TEST 4: full-frame length consistency (PR-G trim)
        // -------------------------------------------------------------
        $display("\n[TEST 4] Full-frame header/body length consistency");
        stream_control = 6'b000_111;
        wait_clk(50);
        egress_count = 0;
        @(posedge clk);
        frame_complete = 1'b1;
        @(posedge clk);
        frame_complete = 1'b0;
        // Wait for full drain: 9 + 1024 + 49152 + 6144 + 1 = 56330 bytes.
        // FT601 at 100 MHz produces 1 byte/cycle; budget ~70k ft601_clk + slack.
        wait_clk(100_000);
        check_b("T4.1: egress_count == expected total",
                egress_count == (`RP_FRAME_HDR_BYTES
                                 + `RP_NUM_RANGE_BINS * 2
                                 + `RP_NUM_RANGE_BINS * `RP_NUM_DOPPLER_BINS * 2
                                 + (`RP_NUM_RANGE_BINS * `RP_NUM_DOPPLER_BINS * 2) / 8
                                 + 1));
        check_b("T4.2: header byte0 = 0xAA",
                egress_bytes[0] == 8'hAA);
        check_b("T4.3: header byte1 = protocol version 0x02",
                egress_bytes[1] == `RP_USB_PROTOCOL_VERSION);
        check_b("T4.4: header byte5/6 = range_bins=512",
                {egress_bytes[5], egress_bytes[6]} == 16'd512);
        check_b("T4.5: header byte7/8 = doppler_bins=48",
                {egress_bytes[7], egress_bytes[8]} == 16'd48);
        check_b("T4.6: emitted bytes < pre-trim padded total (74762)",
                egress_count < 74762);
        $display("    egress_count = %0d (expected 56330)", egress_count);

        // -------------------------------------------------------------
        // TEST 5: MEDIUM ladder timing opcodes round-trip
        // -------------------------------------------------------------
        $display("\n[TEST 5] MEDIUM ladder timing opcodes (0x17, 0x18)");
        send_cmd(`RP_OP_MEDIUM_CHIRP_CYCLES, 8'h00, 16'd750);
        check_b("T5.1: cmd_opcode=0x17 (MEDIUM_CHIRP_CYCLES)", cmd_opcode == 8'h17);
        check_b("T5.2: cmd_value=750",                          cmd_value == 16'd750);

        send_cmd(`RP_OP_MEDIUM_LISTEN_CYCLES, 8'h00, 16'd16500);
        check_b("T5.3: cmd_opcode=0x18 (MEDIUM_LISTEN_CYCLES)", cmd_opcode == 8'h18);
        check_b("T5.4: cmd_value=16500",                         cmd_value == 16'd16500);

        $display("\n-----------------------------------------------------------");
        $display("RESULTS: %0d PASS, %0d FAIL", pass, fail);
        $display("-----------------------------------------------------------");
        if (fail == 0) $display("[OVERALL PASS]"); else $display("[OVERALL FAIL]");
        $finish;
    end

    initial begin
        #20_000_000;
        $display("[TIMEOUT] tb_usb_data_interface watchdog");
        $finish;
    end

endmodule
