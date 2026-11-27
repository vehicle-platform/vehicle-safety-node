/* L0 tests for hal/fault_record: store, integrity (magic + check word), cold-boot garbage. */
#include "hal/fault_record.h"

#include <stdint.h>
#include <unity.h>

void setUp(void) {}
void tearDown(void) {}

static void test_store_is_valid(void)
{
    fault_record_t r = {0u, 0u, 0u, 0u, 0u, 0u};
    fault_record_store(&r, FAULT_TAG_HARDFAULT, 0x00000200u, 0x40000000u, 0x00000100u);
    TEST_ASSERT_TRUE(fault_record_valid(&r));
    TEST_ASSERT_EQUAL_UINT32(FAULT_TAG_HARDFAULT, r.tag);
    TEST_ASSERT_EQUAL_UINT32(0x00000200u, r.cfsr);
    TEST_ASSERT_EQUAL_UINT32(0x40000000u, r.hfsr);
    TEST_ASSERT_EQUAL_UINT32(0x00000100u, r.cifr); /* CSSF */
}

static void test_every_tag_valid_and_range(void)
{
    fault_record_t r;
    for (uint32_t t = FAULT_TAG_NMI; t <= FAULT_TAG_ERROR_HANDLER; t++) {
        fault_record_store(&r, t, 0u, 0u, 0u);
        TEST_ASSERT_TRUE(fault_record_valid(&r));
    }
    fault_record_store(&r, FAULT_TAG_NONE, 0u, 0u, 0u);
    TEST_ASSERT_FALSE(fault_record_valid(&r));
    fault_record_store(&r, FAULT_TAG_ERROR_HANDLER + 1u, 0u, 0u, 0u);
    TEST_ASSERT_FALSE(fault_record_valid(&r));
}

static void test_asm_literals_match(void)
{
    TEST_ASSERT_EQUAL_UINT32(FAULT_TAG_HARDFAULT, (uint32_t)FAULT_TAG_HARDFAULT_N);
    TEST_ASSERT_EQUAL_UINT32(FAULT_TAG_ERROR_HANDLER, (uint32_t)FAULT_TAG_ERROR_HANDLER_N);
}

static void test_garbage_and_corruption(void)
{
    const fault_record_t z = {0u, 0u, 0u, 0u, 0u, 0u};
    const fault_record_t ones = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
    fault_record_t r;
    TEST_ASSERT_FALSE(fault_record_valid(&z));
    TEST_ASSERT_FALSE(fault_record_valid(&ones));
    fault_record_store(&r, FAULT_TAG_BUSFAULT, 0x8000u, 0u, 0u);
    r.cfsr ^= 1u;
    TEST_ASSERT_FALSE(fault_record_valid(&r));
    fault_record_store(&r, FAULT_TAG_BUSFAULT, 0x8000u, 0u, 0u);
    r.cifr ^= 0x100u;
    TEST_ASSERT_FALSE(fault_record_valid(&r));
    fault_record_store(&r, FAULT_TAG_BUSFAULT, 0x8000u, 0u, 0u);
    r.magic = 0u; /* boot_fault_record_clear() */
    TEST_ASSERT_FALSE(fault_record_valid(&r));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_store_is_valid);
    RUN_TEST(test_every_tag_valid_and_range);
    RUN_TEST(test_asm_literals_match);
    RUN_TEST(test_garbage_and_corruption);
    return UNITY_END();
}
