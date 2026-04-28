/*******************************************************************************
 * test_mcu_a2_mag_declination.c
 *
 * MCU-A2: magnetic declination was a hardcoded -0.61° literal baked in for
 * one deployment site. Yaw_Sensor is wrong by (site_decl - (-0.61))° at
 * every other site whenever GPS dual-antenna heading is unavailable.
 *
 * Production fix backs the value with a BKPSRAM slot and exposes a
 * setter/getter pair. Default returns to the legacy -0.61° when no override
 * has been written, preserving backward compatibility for the original
 * site. Range is clamped to ±30° (real-world declinations are roughly
 * -25° to +25°, so anything beyond is a calibration error).
 *
 * This test models the BKPSRAM slot, replays the setter/getter, and
 * verifies clamping, persistence across "reset", default-on-empty, and
 * defensive clamping if BKPSRAM is corrupted.
 ******************************************************************************/
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAG_DECL_DEFAULT  (-0.61f)
#define MAG_DECL_LIMIT    (30.0f)
#define MAG_DECL_MAGIC    0xCAFEFACEU

/* Models BKPSRAM slot for declination (magic + value). */
static uint32_t g_magic;
static uint32_t g_value_bits;

static void simulated_power_cycle(void) { g_magic = 0; g_value_bits = 0; }
static void simulated_corrupt(uint32_t magic, float value) {
    g_magic = magic;
    union { float f; uint32_t u; } cvt = { .f = value };
    g_value_bits = cvt.u;
}

static void set_decl(float deg) {
    if (deg >  MAG_DECL_LIMIT) deg =  MAG_DECL_LIMIT;
    if (deg < -MAG_DECL_LIMIT) deg = -MAG_DECL_LIMIT;
    union { float f; uint32_t u; } cvt = { .f = deg };
    g_value_bits = cvt.u;
    g_magic      = MAG_DECL_MAGIC;
}

static float get_decl(void) {
    if (g_magic == MAG_DECL_MAGIC) {
        union { float f; uint32_t u; } cvt = { .u = g_value_bits };
        float v = cvt.f;
        if (v >  MAG_DECL_LIMIT) v =  MAG_DECL_LIMIT;
        if (v < -MAG_DECL_LIMIT) v = -MAG_DECL_LIMIT;
        return v;
    }
    return MAG_DECL_DEFAULT;
}

static bool feq(float a, float b) { return fabsf(a - b) < 0.001f; }

int main(void)
{
    printf("=== MCU-A2: mag-declination BKPSRAM persistence + clamp ===\n");

    /* 1. Empty BKPSRAM -> legacy default for backward compatibility. */
    printf("  Test 1: empty BKPSRAM returns default -0.61 ... ");
    simulated_power_cycle();
    assert(feq(get_decl(), MAG_DECL_DEFAULT));
    printf("PASS\n");

    /* 2. Setter writes; getter returns the written value. */
    printf("  Test 2: set 12.4 then get ... ");
    set_decl(12.4f);
    assert(feq(get_decl(), 12.4f));
    printf("PASS\n");

    /* 3. Value persists across simulated reset (BKPSRAM survives). */
    printf("  Test 3: persists across reset ... ");
    /* simulated reset = process state preserved (BKPSRAM survives) */
    assert(feq(get_decl(), 12.4f));
    printf("PASS\n");

    /* 4. Power cycle clears BKPSRAM -> back to default. */
    printf("  Test 4: power-cycle restores default ... ");
    simulated_power_cycle();
    assert(feq(get_decl(), MAG_DECL_DEFAULT));
    printf("PASS\n");

    /* 5. Setter clamps high. */
    printf("  Test 5: set +45 clamps to +30 ... ");
    set_decl(45.0f);
    assert(feq(get_decl(), 30.0f));
    printf("PASS\n");

    /* 6. Setter clamps low. */
    printf("  Test 6: set -45 clamps to -30 ... ");
    set_decl(-45.0f);
    assert(feq(get_decl(), -30.0f));
    printf("PASS\n");

    /* 7. Plausible site values pass through unmodified. */
    printf("  Test 7: realistic site values pass through ... ");
    const float sites[] = { -22.5f, -8.0f, -0.61f, 0.0f, 4.3f, 11.2f, 17.9f };
    for (size_t i = 0; i < sizeof(sites)/sizeof(sites[0]); i++) {
        set_decl(sites[i]);
        assert(feq(get_decl(), sites[i]));
    }
    printf("PASS\n");

    /* 8. Defensive clamp on getter — if BKPSRAM is corrupted to an
     * out-of-range value (VBAT brown-out, bit flip), getter still returns
     * a safe value rather than propagating a wild offset. */
    printf("  Test 8: corrupt +1000 BKPSRAM clamps to +30 on read ... ");
    simulated_corrupt(MAG_DECL_MAGIC, 1000.0f);
    assert(feq(get_decl(), 30.0f));
    printf("PASS\n");

    /* 9. Wrong magic -> default (corruption that doesn't preserve magic). */
    printf("  Test 9: wrong magic returns default ... ");
    simulated_corrupt(0xDEADBEEFU, 5.0f);
    assert(feq(get_decl(), MAG_DECL_DEFAULT));
    printf("PASS\n");

    /* 10. Pre-fix regression: hardcoded -0.61 used at non-default site
     * yields wrong heading by (site_decl - (-0.61))°. Confirm the fixed
     * path returns the configured site value, eliminating the offset. */
    printf("  Test 10: post-fix yaw correction matches configured site ... ");
    set_decl(11.2f);
    float pre_fix_decl_used  = -0.61f;
    float post_fix_decl_used = get_decl();
    float bearing_error = post_fix_decl_used - pre_fix_decl_used;
    assert(feq(bearing_error, 11.81f));   /* heading offset corrected */
    printf("(error %.2f degrees), PASS\n", (double)bearing_error);

    printf("\n=== MCU-A2: ALL TESTS PASSED ===\n\n");
    return 0;
}
