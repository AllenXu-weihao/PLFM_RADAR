// ============================================================================
// tb_audit_s10_gpio_split.v
//
// AUDIT-S10: gpio_dig5 previously OR'd six unrelated flags — four signal-
// saturation classes (AGC, DDC overflow, DDC saturation, MTI saturation) and
// two control-fault classes (range-decimator watchdog, CIC->FIR CDC overrun)
// — into the single MCU-visible bit at PD13. The MCU outer-loop AGC reduces
// RF gain on PD13 assertion, which is the wrong response to a watchdog or
// CDC stall. gpio_dig7 (PD15) was tied 1'b0 (reserved).
//
// Fix: split the OR-network so gpio_dig5 carries only signal-saturation flags
// (AGC continues to react correctly) and gpio_dig7 carries control-fault
// flags (MCU follow-up will log + reset; until then host telemetry covers).
// Status words[5][6:5] expose the two control-fault classes so host-side
// can graph them regardless of MCU consumption.
//
// This TB mirrors the production fragments from radar_system_top.v and
// usb_data_interface[*_ft2232h].v and asserts:
//
//   GROUP A  GPIO split (combinational)
//     T1  All inputs 0          -> dig5=0, dig7=0
//     T2  Each signal-sat input -> dig5=1, dig7=0  (no cross-route to dig7)
//     T3  Each control-fault    -> dig5=0, dig7=1  (no cross-route to dig5)
//     T4  Mixed sat + fault     -> dig5=1, dig7=1  (independent)
//     T5  AGC count >0 (boundary) -> dig5=1
//     T6  DDC count =0 (boundary) -> dig5=0
//
//   GROUP B  Status-word CDC packing (sequential)
//     T7  Reset state           -> sync regs 0, status_word_bits 0
//     T8  Watchdog asserted in src -> after 2 ft_clk edges, status[5]=1
//     T9  CIC overrun asserted in src -> after 2 ft_clk edges, status[6]=1
//     T10 Both asserted, both cleared -> status[6:5] tracks (sticky in src;
//                                        TB drives explicit clears)
//     T11 status_words[5][7] stays 0  (reserved bit, not stomped by sync)
//     T12 status_words[5][4:0] (self_test_flags) stays = src input
// ============================================================================
`timescale 1ns/1ps

module tb_audit_s10_gpio_split;

    // ===== GROUP A: GPIO split inputs/outputs =====
    reg [7:0] agc_saturation_count;
    reg       ddc_overflow_any;
    reg [2:0] ddc_saturation_count;
    reg [7:0] mti_saturation_count;
    reg       range_decim_watchdog;
    reg       ddc_cic_fir_overrun;

    wire dig5;
    wire dig7;

    gpio_split_block gpio_dut (
        .agc_saturation_count (agc_saturation_count),
        .ddc_overflow_any     (ddc_overflow_any),
        .ddc_saturation_count (ddc_saturation_count),
        .mti_saturation_count (mti_saturation_count),
        .range_decim_watchdog (range_decim_watchdog),
        .ddc_cic_fir_overrun  (ddc_cic_fir_overrun),
        .gpio_dig5            (dig5),
        .gpio_dig7            (dig7)
    );

    // ===== GROUP B: status-word CDC packing =====
    reg        clk_src   = 1'b0;   // 100 MHz radar domain
    reg        ft_clk    = 1'b0;   // 60/100 MHz USB domain
    reg        reset_n   = 1'b0;
    reg        src_watchdog;
    reg        src_overrun;
    reg [4:0]  src_self_test_flags;
    reg        status_req_pulse;

    wire [31:0] status_word_5;

    status_packing_block status_dut (
        .clk                     (clk_src),
        .ft_clk                  (ft_clk),
        .reset_n                 (reset_n),
        .status_range_decim_watchdog (src_watchdog),
        .status_ddc_cic_fir_overrun  (src_overrun),
        .status_self_test_flags  (src_self_test_flags),
        .status_req_pulse_ft     (status_req_pulse),
        .status_word_5           (status_word_5)
    );

    // 100 MHz src clock
    always #5  clk_src = ~clk_src;
    // 60 MHz ft_clk (~16.67 ns)
    always #8  ft_clk  = ~ft_clk;

    // ----- bookkeeping -----
    integer pass = 0;
    integer fail = 0;

    task check_dig (input [127:0] label, input expected_dig5, input expected_dig7);
        begin
            #1;  // settle combinational
            if (dig5 === expected_dig5 && dig7 === expected_dig7) begin
                $display("  [PASS] %0s: dig5=%b dig7=%b", label, dig5, dig7);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: dig5=%b (exp %b)  dig7=%b (exp %b)",
                         label, dig5, expected_dig5, dig7, expected_dig7);
                fail = fail + 1;
            end
        end
    endtask

    task check_status (input [127:0] label, input [31:0] mask, input [31:0] expected);
        begin
            if ((status_word_5 & mask) === (expected & mask)) begin
                $display("  [PASS] %0s: word5=%h (masked %h)",
                         label, status_word_5, status_word_5 & mask);
                pass = pass + 1;
            end else begin
                $display("  [FAIL] %0s: word5=%h masked %h (exp %h)",
                         label, status_word_5, status_word_5 & mask, expected & mask);
                fail = fail + 1;
            end
        end
    endtask

    task pulse_status_req;
        begin
            @(posedge ft_clk); #1;
            status_req_pulse = 1'b1;
            @(posedge ft_clk); #1;
            status_req_pulse = 1'b0;
            // Allow the registered status_words update to land.
            @(posedge ft_clk); #1;
        end
    endtask

    initial begin
        $display("============================================================");
        $display("AUDIT-S10: gpio_dig split + status_words[5][6:5] visibility");
        $display("============================================================");

        // ---- GROUP A: GPIO split ----
        agc_saturation_count = 8'd0;
        ddc_overflow_any     = 1'b0;
        ddc_saturation_count = 3'd0;
        mti_saturation_count = 8'd0;
        range_decim_watchdog = 1'b0;
        ddc_cic_fir_overrun  = 1'b0;

        // T1
        check_dig("T1 all zero", 1'b0, 1'b0);

        // T2 each signal-sat individually
        agc_saturation_count = 8'd1;
        check_dig("T2a agc_sat>0", 1'b1, 1'b0);
        agc_saturation_count = 8'd0;

        ddc_overflow_any = 1'b1;
        check_dig("T2b ddc_overflow", 1'b1, 1'b0);
        ddc_overflow_any = 1'b0;

        ddc_saturation_count = 3'd1;
        check_dig("T2c ddc_sat>0", 1'b1, 1'b0);
        ddc_saturation_count = 3'd0;

        mti_saturation_count = 8'd1;
        check_dig("T2d mti_sat>0", 1'b1, 1'b0);
        mti_saturation_count = 8'd0;

        // T3 each control-fault individually
        range_decim_watchdog = 1'b1;
        check_dig("T3a watchdog", 1'b0, 1'b1);
        range_decim_watchdog = 1'b0;

        ddc_cic_fir_overrun = 1'b1;
        check_dig("T3b cic_fir_overrun", 1'b0, 1'b1);
        ddc_cic_fir_overrun = 1'b0;

        // T4 mixed
        agc_saturation_count = 8'd5;
        range_decim_watchdog = 1'b1;
        check_dig("T4 mixed sat+fault", 1'b1, 1'b1);
        agc_saturation_count = 8'd0;
        range_decim_watchdog = 1'b0;

        // T5 boundary: largest agc count
        agc_saturation_count = 8'hFF;
        check_dig("T5 agc_sat=FF", 1'b1, 1'b0);
        agc_saturation_count = 8'd0;

        // T6 boundary: ddc_sat=0 stays low
        ddc_saturation_count = 3'd0;
        check_dig("T6 ddc_sat=0", 1'b0, 1'b0);

        // ---- GROUP B: status-word CDC packing ----
        src_watchdog        = 1'b0;
        src_overrun         = 1'b0;
        src_self_test_flags = 5'b00000;
        status_req_pulse    = 1'b0;

        // Apply reset
        reset_n = 1'b0;
        repeat (5) @(posedge ft_clk);
        reset_n = 1'b1;
        repeat (3) @(posedge ft_clk);

        // T7 reset state
        pulse_status_req();
        check_status("T7 reset state",
                     32'h000000E0,    // [7:5]
                     32'h00000000);

        // T8 watchdog asserted only
        @(posedge clk_src); #1;
        src_watchdog = 1'b1;
        // give 4 ft_clk for sync chain to settle
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T8 watchdog asserted",
                     32'h00000060,    // [6:5]
                     32'h00000020);   // [5]=1

        // T9 cic_fir_overrun asserted only (clear watchdog first)
        @(posedge clk_src); #1;
        src_watchdog = 1'b0;
        src_overrun  = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T9 cic_fir_overrun asserted",
                     32'h00000060,
                     32'h00000040);   // [6]=1

        // T10 both, then both cleared
        @(posedge clk_src); #1;
        src_watchdog = 1'b1;
        src_overrun  = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T10a both asserted",
                     32'h00000060,
                     32'h00000060);   // [6:5]=11

        @(posedge clk_src); #1;
        src_watchdog = 1'b0;
        src_overrun  = 1'b0;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T10b both cleared",
                     32'h00000060,
                     32'h00000000);

        // T11 reserved bit [7] stays 0 even when neighbours are 1
        @(posedge clk_src); #1;
        src_watchdog = 1'b1;
        src_overrun  = 1'b1;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T11 [7] reserved stays 0",
                     32'h00000080,    // [7] only
                     32'h00000000);

        // T12 self_test_flags pass through unchanged
        @(posedge clk_src); #1;
        src_watchdog        = 1'b0;
        src_overrun         = 1'b0;
        src_self_test_flags = 5'b10110;
        repeat (5) @(posedge ft_clk);
        pulse_status_req();
        check_status("T12 self_test_flags untouched",
                     32'h0000001F,
                     32'h00000016);

        $display("============================================================");
        $display("AUDIT-S10 RESULTS: pass=%0d fail=%0d", pass, fail);
        $display("============================================================");
        if (fail == 0) $display("[OVERALL] PASS");
        else           $display("[OVERALL] FAIL");
        $finish;
    end

    initial begin
        #1_000_000;
        $display("[FATAL] timeout");
        $finish;
    end

endmodule

// ============================================================================
// gpio_split_block — mirrors the production fragment from radar_system_top.v
// post AUDIT-S10. Two combinational ORs:
//   gpio_dig5 = signal-saturation classes (AGC + DDC + MTI)
//   gpio_dig7 = control-fault classes (range-decimator watchdog + CIC->FIR
//               CDC overrun)
// ============================================================================
module gpio_split_block (
    input  wire [7:0] agc_saturation_count,
    input  wire       ddc_overflow_any,
    input  wire [2:0] ddc_saturation_count,
    input  wire [7:0] mti_saturation_count,
    input  wire       range_decim_watchdog,
    input  wire       ddc_cic_fir_overrun,
    output wire       gpio_dig5,
    output wire       gpio_dig7
);
    assign gpio_dig5 = (agc_saturation_count != 8'd0)
                     | ddc_overflow_any
                     | (ddc_saturation_count != 3'd0)
                     | (mti_saturation_count != 8'd0);
    assign gpio_dig7 = range_decim_watchdog
                     | ddc_cic_fir_overrun;
endmodule

// ============================================================================
// status_packing_block — mirrors the production CDC fragment from
// usb_data_interface.v (and usb_data_interface_ft2232h.v) for the AUDIT-S10
// telemetry path. Source-domain inputs cross to ft_clk via 2-FF level sync,
// then pack into status_words[5][6:5]. Self-test flags pass through into
// status_words[5][4:0] for a sanity check that the packing keeps the
// neighbouring fields untouched. Bit [7] is intentionally reserved.
// ============================================================================
module status_packing_block (
    input  wire        clk,        // 100 MHz radar domain (unused but mirrors prod port list)
    input  wire        ft_clk,
    input  wire        reset_n,
    input  wire        status_range_decim_watchdog,
    input  wire        status_ddc_cic_fir_overrun,
    input  wire [4:0]  status_self_test_flags,
    input  wire        status_req_pulse_ft,
    output reg  [31:0] status_word_5
);
    (* ASYNC_REG = "TRUE" *) reg range_decim_watchdog_sync_0;
    reg                          range_decim_watchdog_sync_1;
    (* ASYNC_REG = "TRUE" *) reg ddc_cic_fir_overrun_sync_0;
    reg                          ddc_cic_fir_overrun_sync_1;

    always @(posedge ft_clk or negedge reset_n) begin
        if (!reset_n) begin
            range_decim_watchdog_sync_0 <= 1'b0;
            range_decim_watchdog_sync_1 <= 1'b0;
            ddc_cic_fir_overrun_sync_0  <= 1'b0;
            ddc_cic_fir_overrun_sync_1  <= 1'b0;
            status_word_5               <= 32'd0;
        end else begin
            range_decim_watchdog_sync_0 <= status_range_decim_watchdog;
            range_decim_watchdog_sync_1 <= range_decim_watchdog_sync_0;
            ddc_cic_fir_overrun_sync_0  <= status_ddc_cic_fir_overrun;
            ddc_cic_fir_overrun_sync_1  <= ddc_cic_fir_overrun_sync_0;

            if (status_req_pulse_ft) begin
                status_word_5 <= {7'd0, 1'b0,                  // [31:24] busy slot tied 0 in TB
                                  8'd0,                        // [23:16] reserved
                                  8'd0,                        // [15:8]  detail tied 0 in TB
                                  1'd0,                        // [7]     reserved
                                  ddc_cic_fir_overrun_sync_1,  // [6]
                                  range_decim_watchdog_sync_1, // [5]
                                  status_self_test_flags};     // [4:0]
            end
        end
    end
endmodule
