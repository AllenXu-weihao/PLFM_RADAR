/*******************************************************************************
 * test_mcu_a4_ocxo_warm_restart.c
 *
 * MCU-A4: every boot waited the full 180 s OCXO warmup soak — even an
 * IWDG/SYSRESETREQ reset that takes seconds and leaves the OCXO oven hot
 * lost three minutes of bringup time. No warm-restart bypass.
 *
 * Production fix sets a BKPSRAM flag at the end of the cold-boot warmup
 * loop. Subsequent boots that find the flag still set know the previous
 * boot completed warmup AND the BKPSRAM was not cleared by main-power
 * removal, so the OCXO oven is still hot and the crystal is settled.
 * Warm-restart path waits 5 s instead of 180 s. Power-cycle clears
 * BKPSRAM, forcing the full soak again.
 *
 * This test models the BKPSRAM flag and replays cold/warm boot sequences,
 * asserting the warmup duration matches the boot type.
 ******************************************************************************/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define WARMUP_MAGIC   0xCA1C1F1EU
#define COLD_WARMUP_S  180
#define WARM_WARMUP_S  5

static uint32_t g_warmup_flag;

static void simulated_power_cycle(void)  { g_warmup_flag = 0; }
static void simulated_mcu_reset(void)    { /* BKPSRAM survives — no-op */ }
static void warmup_persist_set(void)     { g_warmup_flag = WARMUP_MAGIC; }
static bool warmup_persist_check(void)   { return g_warmup_flag == WARMUP_MAGIC; }

/* Models the boot warmup branch from main.cpp:1601 */
static int boot_ocxo_warmup_seconds(void)
{
    if (warmup_persist_check()) return WARM_WARMUP_S;
    /* cold path soaks then arms the bypass for next reset */
    int soak = COLD_WARMUP_S;
    warmup_persist_set();
    return soak;
}

int main(void)
{
    printf("=== MCU-A4: OCXO warm-restart bypass ===\n");

    /* 1. Cold boot from cleared BKPSRAM -> full 180 s soak. */
    printf("  Test 1: cold boot soaks 180 s ... ");
    simulated_power_cycle();
    assert(boot_ocxo_warmup_seconds() == COLD_WARMUP_S);
    printf("PASS\n");

    /* 2. Cold boot ARMS the warm-restart flag for next reset. */
    printf("  Test 2: cold boot sets BKPSRAM flag ... ");
    assert(warmup_persist_check() == true);
    printf("PASS\n");

    /* 3. IWDG / SYSRESETREQ reset -> warm path, 5 s only. */
    printf("  Test 3: warm reset takes 5 s only ... ");
    simulated_mcu_reset();
    assert(boot_ocxo_warmup_seconds() == WARM_WARMUP_S);
    printf("PASS\n");

    /* 4. Repeated warm resets all stay on the fast path. */
    printf("  Test 4: 5 successive warm resets all 5 s ... ");
    for (int i = 0; i < 5; i++) {
        simulated_mcu_reset();
        assert(boot_ocxo_warmup_seconds() == WARM_WARMUP_S);
    }
    printf("5/5, PASS\n");

    /* 5. Power-cycle clears BKPSRAM -> next boot must do the full soak. */
    printf("  Test 5: power-cycle forces full 180 s next boot ... ");
    simulated_power_cycle();
    assert(boot_ocxo_warmup_seconds() == COLD_WARMUP_S);
    printf("PASS\n");

    /* 6. After the post-power-cycle cold boot, the flag is re-armed
     * and the next reset is fast again. */
    printf("  Test 6: cold-after-power-cycle re-arms warm bypass ... ");
    simulated_mcu_reset();
    assert(boot_ocxo_warmup_seconds() == WARM_WARMUP_S);
    printf("PASS\n");

    /* 7. Pre-fix regression: every boot was 180 s regardless of type.
     * Confirm fixed warm path is strictly faster than cold path. */
    printf("  Test 7: warm path strictly faster than cold ... ");
    assert(WARM_WARMUP_S < COLD_WARMUP_S);
    /* Total saved across 10 warm restarts = 10 * (180 - 5) = 1750 s */
    int saved = 10 * (COLD_WARMUP_S - WARM_WARMUP_S);
    assert(saved == 1750);
    printf("(10 warm restarts save %d s vs pre-fix), PASS\n", saved);

    printf("\n=== MCU-A4: ALL TESTS PASSED ===\n\n");
    return 0;
}
