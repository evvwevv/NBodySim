#include <math.h>

#define ASSERT_DBL_ALMOST_EQUAL(d1, d2, thresh) \
    do { \
        if (!(fabs(d1 - d2) < thresh)) { \
            printf("%f is not almost equal to %f", d1, d2); \
            return 1; \
        } \
     } while (0)

#define ASSERT_TRUE(test, msg) \
    do { \
        if (!(test)) { \
            printf(msg); \
            return 1; \
        } \
    } while (0)

#define ASSERT_INT_EQUAL(i1, i2) \
    do { \
        if (!(i1 == i2)) { \
            printf("%d is not equal to %d", i1, i2); \
            return 1; \
        } \
     } while (0)

#define ASSERT_INT_LT(i1, i2) \
    do { \
        if (!(i1 < i2)) { \
            printf("%d is not less than %d", i1, i2); \
            return 1; \
        } \
    } while (0)

#define ASSERT_INT_GT(i1, i2) \
    do { \
        if (!(i1 > i2)) { \
            printf("%d is not greater than %d", i1, i2); \
            return 1; \
        } \
    } while (0)

#define RUN_TEST(test) \
    do { \
        printf(#test ": "); \
        if (test()) { \
            culite_tests_failed++; \
            printf(" -> FAILED\n"); \
        } else \
            printf("PASSED\n"); \
        culite_tests_run++; \
    } while (0)

#define TEST_CASE(name) int name () {
#define TEST_CASE_END return 0; }

extern int culite_tests_run;
extern int culite_tests_failed;
