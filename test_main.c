#include <stdio.h>
#include "culite.h"
#include "vector.h"
#include "sim.h"

/* Insert declarations here to expose functions from source files
 * for testing.
 *
 * Ex: int func_to_test(int, int);
 */

/* Tests start here */
TEST_CASE(test_VECTOR_LEN_SQRD)
{
    Vector3 v = {2, 3, 4};

    double len = VECTOR_LEN_SQRD(v);

    ASSERT_TRUE(len == 29, "");
}
TEST_CASE_END

TEST_CASE(test_normalize_vector)
{
    Vector3 v = {2, 3, 4};
    Vector3 norm = normalize_vector(v);

    ASSERT_DBL_ALMOST_EQUAL(norm.x, .3714, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(norm.y, .5571, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(norm.z, .7428, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(VECTOR_LEN_SQRD(norm), 1.0, 0.0001);
}
TEST_CASE_END

TEST_CASE(test_scale_vector)
{
    Vector3 v = {1.5, 2, 3};
    Vector3 scaled = scale_vector(v, 3);

    ASSERT_DBL_ALMOST_EQUAL(scaled.x, 4.5, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(scaled.y, 6.0, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(scaled.z, 9.0, 0.0001);
}
TEST_CASE_END

TEST_CASE(test_normalize_then_scale) {
    Vector3 v = {2, 3, 4};
    Vector3 norm = normalize_vector(v);
    Vector3 scaled = scale_vector(norm, 3);
    double len_squared = VECTOR_LEN_SQRD(scaled);
    double len = sqrt(len_squared);

    ASSERT_DBL_ALMOST_EQUAL(len, 3.0, 0.0001);
}
TEST_CASE_END

TEST_CASE(test_ADD_TO_VECTOR)
{
    Vector3 v = {1.5, 2, 3};
    Vector3 w = {2, 4, 6};

    ADD_TO_VECTOR(v, w);

    ASSERT_DBL_ALMOST_EQUAL(v.x, 3.5, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(v.y, 6.0, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(v.z, 9.0, 0.0001);

    ASSERT_DBL_ALMOST_EQUAL(w.x, 2.0, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(w.y, 4.0, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(w.z, 6.0, 0.0001);
}
TEST_CASE_END

TEST_CASE(test_vector_from_to)
{
    Vector3 from = {1, 1, 1};
    Vector3 to = {5, 6, 7};
    Vector3 from_to = vector_from_to(from, to);

    ASSERT_DBL_ALMOST_EQUAL(from_to.x, 4.0, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(from_to.y, 5.0, 0.0001);
    ASSERT_DBL_ALMOST_EQUAL(from_to.z, 6.0, 0.0001);
}
TEST_CASE_END

TEST_CASE(test_get_acc)
{
    int n = 3;
    double masses[3] = {2e20, 2e5, 3e2};
    Vector3 prevs[n*2];
    prevs[0].x = 0;
    prevs[0].y = 0;
    prevs[0].z = 0;
    /* Don't care about velocity vector */
    prevs[2].x = 997330.368;
    prevs[2].y = 50;
    prevs[2].z = 0;
    prevs[4].x = 1000000000;
    prevs[4].y = 20;
    prevs[4].z = 0;

    Vector3 acc = get_acc(prevs, masses, 1, n);

    ASSERT_DBL_ALMOST_EQUAL(acc.x, -0.0134197157, 0.00001);
    ASSERT_DBL_ALMOST_EQUAL(acc.y, -6.72781871e-7, 0.00001);
    ASSERT_DBL_ALMOST_EQUAL(acc.z, 0.0, 0.00001);
}
TEST_CASE_END

void culite_run_all_tests() {
    RUN_TEST(test_VECTOR_LEN_SQRD);
    RUN_TEST(test_normalize_vector);
    RUN_TEST(test_scale_vector);
    RUN_TEST(test_normalize_then_scale);
    RUN_TEST(test_ADD_TO_VECTOR);
    RUN_TEST(test_vector_from_to);
    RUN_TEST(test_get_acc);
}
