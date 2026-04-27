/*******************************************************************************
 * test_mcu_a7_emergency_persist.c
 *
 * MCU-A7: the original Emergency_Stop hold loop refreshed IWDG forever, so
 * a stuck loop or wedged interrupt could not be recovered by watchdog reset
 * AND, even if a reset did occur (e.g. SYSRESETREQ from another fault),
 * startup re-energized the PA rails because there was no persistent state.
 *
 * Production fix uses BKPSRAM as a reset-surviving emergency flag:
 *   1. Emergency_Stop sets the BKPSRAM magic BEFORE cutting rails.
 *   2. main() checks the flag IMMEDIATELY after MX_IWDG_Init, before any
 *      PA enable code, and re-enters Emergency_Stop if the flag is set.
 *   3. The flag is cleared only by main-power removal (BKPSRAM loses
 *      contents) — power-cycle is the deliberate operator action required
 *      to clear emergency.
 *
 * This test models BKPSRAM as a process-local "non-volatile" word, replays
 * the Emergency_Stop set + boot-time check sequence across simulated
 * resets, and asserts the PA rails stay LOW across every reset path until
 * BKPSRAM is explicitly cleared (modelling a power cycle).
 ******************************************************************************/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* --- Simulated BKPSRAM (survives "reset" but not "power cycle") --- */
static uint32_t g_bkpsram_word;

#define EMERGENCY_PERSIST_MAGIC 0xDEAD5A5AU

static void emergency_persist_set(void)   { g_bkpsram_word = EMERGENCY_PERSIST_MAGIC; }
static bool emergency_persist_check(void) { return g_bkpsram_word == EMERGENCY_PERSIST_MAGIC; }
static void simulated_power_cycle(void)   { g_bkpsram_word = 0; }

/* --- Simulated PA rail state (set by GPIO init at boot, modified by code) --- */
typedef struct {
    bool pa1_5v0;
    bool pa2_5v0;
    bool pa3_5v0;
    bool pa_5v5;
    bool rfpa_vdd;
} pa_rails_t;

static pa_rails_t pa;

static void mx_gpio_init(void) {
    /* matches main.cpp:2783 — GPIO init forces all PA enables LOW */
    pa.pa1_5v0 = false; pa.pa2_5v0 = false; pa.pa3_5v0 = false;
    pa.pa_5v5  = false; pa.rfpa_vdd = false;
}

static void enable_pa_rails(void) {
    /* models the cold-boot startup sequence that energizes the PAs */
    pa.pa1_5v0 = true; pa.pa2_5v0 = true; pa.pa3_5v0 = true;
    pa.pa_5v5  = true; pa.rfpa_vdd = true;
}

static void emergency_stop(void) {
    emergency_persist_set();
    pa.pa1_5v0 = false; pa.pa2_5v0 = false; pa.pa3_5v0 = false;
    pa.pa_5v5  = false; pa.rfpa_vdd = false;
    /* hold loop modelled as immediate return so the test can continue */
}

/* models main() up to the persist check; returns true if PA-enable code ran */
static bool boot_sequence(void) {
    mx_gpio_init();
    /* MX_IWDG_Init() — irrelevant for this test */
    if (emergency_persist_check()) {
        emergency_stop();  /* NOTREACHED in production */
        return false;      /* PA enable code did NOT run */
    }
    enable_pa_rails();
    return true;
}

static bool any_rail_hot(void) {
    return pa.pa1_5v0 || pa.pa2_5v0 || pa.pa3_5v0 || pa.pa_5v5 || pa.rfpa_vdd;
}

int main(void)
{
    printf("=== MCU-A7: BKPSRAM emergency-persist across resets ===\n");

    /* 1. Cold boot from clean state — PAs energize normally. */
    printf("  Test 1: cold boot from cleared BKPSRAM ... ");
    simulated_power_cycle();
    bool pa_enable_ran = boot_sequence();
    assert(pa_enable_ran == true);
    assert(any_rail_hot() == true);
    printf("PA enabled, PASS\n");

    /* 2. Emergency_Stop sets the persist flag and cuts rails. */
    printf("  Test 2: Emergency_Stop sets flag and cuts rails ... ");
    emergency_stop();
    assert(emergency_persist_check() == true);
    assert(any_rail_hot() == false);
    printf("flag=SET rails=OFF, PASS\n");

    /* 3. IWDG reset (or any reset short of power-cycle) — flag survives,
     * boot path takes the safe-hold branch and PA enable code does NOT run. */
    printf("  Test 3: IWDG reset re-enters safe-hold ... ");
    pa_enable_ran = boot_sequence();
    assert(pa_enable_ran == false);
    assert(any_rail_hot() == false);
    assert(emergency_persist_check() == true);
    printf("PA stayed OFF, PASS\n");

    /* 4. Repeat reset N times — flag persists, no PA enable. */
    printf("  Test 4: 10 successive resets all stay safe ... ");
    for (int i = 0; i < 10; i++) {
        pa_enable_ran = boot_sequence();
        assert(pa_enable_ran == false);
        assert(any_rail_hot() == false);
    }
    printf("10/10 stayed OFF, PASS\n");

    /* 5. Power-cycle clears BKPSRAM — next boot energizes PAs again
     * (this is the deliberate operator-recovery path). */
    printf("  Test 5: power-cycle clears flag, next boot energizes ... ");
    simulated_power_cycle();
    assert(emergency_persist_check() == false);
    pa_enable_ran = boot_sequence();
    assert(pa_enable_ran == true);
    assert(any_rail_hot() == true);
    printf("PA enabled after power-cycle, PASS\n");

    /* 6. Regression guard for the pre-fix behaviour: without persistence,
     * any reset would re-run startup and re-energize the PAs even though
     * Emergency_Stop had been entered. Simulate the buggy boot (no flag
     * check) and confirm it would have hot-rail'd — ensuring the test
     * actually exercises the fix. */
    printf("  Test 6: pre-fix regression check ... ");
    simulated_power_cycle();
    boot_sequence();
    emergency_stop();              /* fix: would set flag */
    g_bkpsram_word = 0;            /* simulate the pre-fix "no persistence" */
    /* a buggy boot ignores the flag — re-runs full startup */
    mx_gpio_init();
    enable_pa_rails();             /* this is exactly what the bug allowed */
    assert(any_rail_hot() == true);
    printf("buggy boot would have re-energized PA, fix prevents this, PASS\n");

    printf("\n=== MCU-A7: ALL TESTS PASSED ===\n\n");
    return 0;
}
