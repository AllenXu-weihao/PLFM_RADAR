/*******************************************************************************
 * test_audit_c17_bmp180_sentinel_and_cast.c
 *
 * AUDIT-C17: BMP180 driver had two latent bugs in its temperature path:
 *
 *   (a) BMP180_ERROR=255 was an in-band sentinel returned by uint16_t
 *       read16()/readRawTemperature() on I2C error. 255 is also a valid
 *       uint16_t register reading (0x00FF appears across the calibration
 *       coefficient block and is reachable as a raw temperature/pressure
 *       sample). Sensor failure was indistinguishable from a real reading.
 *
 *   (b) getTemperature() narrowed the uint16_t raw value to int16_t before
 *       calling computeB5(), which takes int32_t. Bit-patterns ≥ 0x8000
 *       (reachable across the BMP180 -40..+85 °C operating window) flipped
 *       to negative int16_t and sign-extended into computeB5(), producing
 *       temperature errors of order 100s of °C.
 *
 * Production fix:
 *   - I/O helpers (read8/read16/readRawTemperature/readRawPressure) now
 *     return bool and pass the value through an out-param. getTemperature
 *     returns NaN on error; getPressure/getSeaLevelPressure return
 *     INT32_MIN. None of these sentinels collide with valid sensor output.
 *   - getTemperature() keeps raw as uint16_t and widens to int32_t
 *     value-preservingly: `(int32_t)raw_uint16` instead of `(int16_t)raw`.
 *
 * This test models the corrected math (computeB5 + getTemperature) plus the
 * casting choices and asserts:
 *   T1: bool out-param signaling is distinguishable from any valid uint16
 *       (incl. 0x00FF, 0x8000, 0xFFFF — all of which collided with the old
 *       BMP180_ERROR=255 OR-with-narrowing scheme).
 *   T2: corrected widen-cast yields the Bosch-reference result for a
 *       calibrated sample (datasheet example UT=27898 -> 15.0 °C).
 *   T3: the buggy narrowing cast produces catastrophically wrong output
 *       for raw UT = 0x8000 (regression guard — flipping the rawTemperature
 *       declaration back to int16_t would re-trigger it).
 *   T4: full-range sweep — no raw uint16 in [0, 65535] should produce
 *       NaN/error from the corrected pipeline; under the buggy pipeline the
 *       upper half of the range collapses to negative output.
 ******************************************************************************/
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------
 * Bosch BMP180 datasheet example calibration (Table 6, datasheet rev 2.5)
 * ------------------------------------------------------------------------- */
typedef struct {
    int16_t  AC1;
    int16_t  AC2;
    int16_t  AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t  B1;
    int16_t  B2;
    int16_t  MB;
    int16_t  MC;
    int16_t  MD;
} BMP180_CAL;

static const BMP180_CAL DATASHEET_CAL = {
    .AC1 =   408, .AC2 =   -72, .AC3 = -14383,
    .AC4 = 32741, .AC5 = 32757, .AC6 =  23153,
    .B1  =  6190, .B2  =     4,
    .MB  = -32768, .MC = -8711, .MD =   2868,
};

/* -------------------------------------------------------------------------
 * Mirrors BMP180::computeB5(int32_t UT) at BMP180.cpp:393-399.
 * Identical math; the only thing this test varies is the caller's choice of
 * cast on the way in.
 * ------------------------------------------------------------------------- */
static int32_t computeB5(const BMP180_CAL *cal, int32_t UT)
{
    int32_t X1 = ((UT - (int32_t)cal->AC6) * (int32_t)cal->AC5) >> 15;
    int32_t X2 = ((int32_t)cal->MC << 11) / (X1 + (int32_t)cal->MD);
    return X1 + X2;
}

/* CORRECTED path: keep raw as uint16_t, widen to int32_t value-preservingly.
 * Mirrors the patched BMP180::getTemperature() at BMP180.cpp:136-148. */
static float getTemperature_fixed(const BMP180_CAL *cal, uint16_t raw)
{
    return (float)((computeB5(cal, (int32_t)raw) + 8) >> 4) / 10.0f;
}

/* BUGGY path: narrow uint16_t → int16_t before widening to int32_t.
 * Mirrors the original BMP180::getTemperature() at HEAD ea2615e. */
static float getTemperature_buggy(const BMP180_CAL *cal, uint16_t raw)
{
    int16_t narrowed = (int16_t)raw;
    return (float)((computeB5(cal, (int32_t)narrowed) + 8) >> 4) / 10.0f;
}

/* -------------------------------------------------------------------------
 * Mock I/O helper modeling the new bool-out-param contract.
 * ------------------------------------------------------------------------- */
typedef struct {
    bool     i2c_will_fail;
    uint16_t programmed_value;
} MockI2C;

static bool mock_readRawTemperature(MockI2C *m, uint16_t *out)
{
    if (m->i2c_will_fail) return false;
    *out = m->programmed_value;
    return true;
}

/* OLD contract (regression model): in-band BMP180_ERROR=255 sentinel,
 * uint16_t return. 255 is a valid reading; we cannot distinguish a real
 * raw=255 reading from a sensor failure. */
#define OLD_BMP180_ERROR 255
static uint16_t mock_readRawTemperature_old(MockI2C *m)
{
    if (m->i2c_will_fail) return OLD_BMP180_ERROR;
    return m->programmed_value;
}

/* -------------------------------------------------------------------------
 * T1: sentinel separability under the new contract.
 * ------------------------------------------------------------------------- */
static void test_t1_sentinel_separability(void)
{
    printf("  T1: bool-out-param sentinel separability ... ");

    MockI2C m;
    uint16_t value;

    /* The exact set called out in the audit memo: every reading that
     * collides with BMP180_ERROR=255 under the old in-band scheme. */
    const uint16_t collision_cases[] = { 0, 1, 254, 255, 256, 32767, 32768, 65535 };
    const size_t   N = sizeof(collision_cases) / sizeof(collision_cases[0]);

    for (size_t i = 0; i < N; i++) {
        m.i2c_will_fail    = false;
        m.programmed_value = collision_cases[i];
        value              = 0xDEAD;
        bool ok = mock_readRawTemperature(&m, &value);
        assert(ok == true);
        assert(value == collision_cases[i]);
    }

    /* I2C-error path: bool=false, out untouched. */
    m.i2c_will_fail = true;
    value           = 0xDEAD;
    bool ok = mock_readRawTemperature(&m, &value);
    assert(ok == false);
    assert(value == 0xDEAD);   /* out-param NOT clobbered on error */

    /* Regression demonstration: under the OLD contract, raw=255 and an I2C
     * fault produce the same return value, so the caller cannot tell them
     * apart. This is the bug the new contract eliminates. */
    m.i2c_will_fail    = false;
    m.programmed_value = 255;
    uint16_t v_real    = mock_readRawTemperature_old(&m);
    m.i2c_will_fail    = true;
    uint16_t v_fault   = mock_readRawTemperature_old(&m);
    assert(v_real == v_fault);   /* old contract: indistinguishable */

    printf("PASS\n");
}

/* -------------------------------------------------------------------------
 * T2: datasheet reference value reproduces under the corrected cast.
 *
 * Bosch BMP180 datasheet (Section 3.5, "Calculating pressure and
 * temperature") worked example: with the calibration above and
 * UT=27898, the expected temperature is 15.0 °C.
 * ------------------------------------------------------------------------- */
static void test_t2_datasheet_reference(void)
{
    printf("  T2: datasheet UT=27898 -> 15.0 °C (fixed cast) ... ");
    float t = getTemperature_fixed(&DATASHEET_CAL, 27898);
    assert(fabsf(t - 15.0f) < 0.05f);
    printf("PASS (got %.2f °C)\n", (double)t);
}

/* -------------------------------------------------------------------------
 * T3: regression guard for the narrowing bug.
 *
 * For raw UT = 0x8000 (32768), the corrected cast yields ~+51 °C; the
 * buggy narrow-cast yields ~-347 °C. The two paths must diverge by
 * hundreds of °C — that is exactly the operational hazard.
 * ------------------------------------------------------------------------- */
static void test_t3_narrowing_regression(void)
{
    printf("  T3: raw UT=0x8000 fixed vs buggy diverge by >100 °C ... ");

    float t_fixed = getTemperature_fixed(&DATASHEET_CAL, 0x8000);
    float t_buggy = getTemperature_buggy(&DATASHEET_CAL, 0x8000);

    /* Fixed path lands in a plausible (if hot) range. */
    assert(t_fixed > 30.0f && t_fixed < 80.0f);

    /* Buggy path is wildly negative — far outside any real sensor range. */
    assert(t_buggy < -100.0f);

    /* The catastrophic divergence is the actual regression signal. */
    assert(fabsf(t_fixed - t_buggy) > 100.0f);

    printf("PASS (fixed=%.1f, buggy=%.1f, delta=%.1f °C)\n",
           (double)t_fixed, (double)t_buggy,
           (double)fabsf(t_fixed - t_buggy));
}

/* -------------------------------------------------------------------------
 * T4: full uint16 range sweep — fixed path stays finite + monotonic-ish;
 * buggy path collapses across the 0x8000 boundary.
 * ------------------------------------------------------------------------- */
static void test_t4_full_range_sweep(void)
{
    printf("  T4: full uint16 sweep — fixed path finite, buggy collapses ... ");

    int total_samples       = 0;
    int upper_half_samples  = 0;
    int buggy_collapses     = 0;
    int fixed_finite        = 0;

    /* Sample raw values across the full uint16 range every 1024 LSB —
     * enough to exercise the 0x8000 boundary without spamming the log. */
    for (uint32_t raw32 = 0; raw32 <= 0xFFFF; raw32 += 1024) {
        uint16_t raw    = (uint16_t)raw32;
        float    t_fix  = getTemperature_fixed(&DATASHEET_CAL, raw);
        float    t_bug  = getTemperature_buggy(&DATASHEET_CAL, raw);

        total_samples++;
        if (isfinite(t_fix)) fixed_finite++;

        /* Boundary crossing: at raw>=0x8000, the buggy path goes negative
         * (wildly) while the fixed path keeps climbing. */
        if (raw >= 0x8000) {
            upper_half_samples++;
            if (t_bug < -50.0f) buggy_collapses++;
        }
    }

    assert(fixed_finite    == total_samples);      /* every sample finite under fixed path */
    assert(buggy_collapses == upper_half_samples); /* every upper-half sample collapsed under buggy path */

    printf("PASS (fixed_finite=%d/%d, buggy_collapses=%d/%d upper-half)\n",
           fixed_finite, total_samples, buggy_collapses, upper_half_samples);
}

int main(void)
{
    printf("=== AUDIT-C17: BMP180 sentinel separability + signed-cast fix ===\n");

    test_t1_sentinel_separability();
    test_t2_datasheet_reference();
    test_t3_narrowing_regression();
    test_t4_full_range_sweep();

    printf("=== ALL PASS ===\n");
    return 0;
}
