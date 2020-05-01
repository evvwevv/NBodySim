#include <stdio.h>

#include "culite.h"

/* This must be defined by users in the test case files. */
void culite_run_all_tests();

int culite_tests_run = 0;
int culite_tests_failed = 0;

int main(int argc, char *argv[]) {
    printf("Test cases:\n--------------------\n");
    culite_run_all_tests();

    printf("--------------------\n");
    if (culite_tests_failed == 0)
        printf("ALL TESTS PASSED\n");

    printf("%d/%d tests passed.\n",
           (culite_tests_run - culite_tests_failed),
           culite_tests_run);

    return culite_tests_failed != 0;
}
