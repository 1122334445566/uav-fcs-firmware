#include <stdio.h>

int test_math(void);
int test_attitude(void);
int test_control(void);
int test_mixer(void);
int test_safety(void);
int test_integration(void);

typedef struct {
    const char *name;
    int (*fn)(void);
} test_case_t;

int main(void)
{
    const test_case_t cases[] = {
        { "math/quaternion/vector",  test_math },
        { "attitude estimator",      test_attitude },
        { "cascaded controller",     test_control },
        { "mixer / allocation",      test_mixer },
        { "failsafe / FSM / tlm",    test_safety },
        { "top-level integration",   test_integration }
    };
    const int count = (int)(sizeof(cases) / sizeof(cases[0]));
    int failed = 0;

    printf("Running %d test suites\n", count);
    for (int i = 0; i < count; ++i) {
        printf("[ RUN      ] %s\n", cases[i].name);
        if (cases[i].fn() != 0) {
            printf("[  FAILED  ] %s\n", cases[i].name);
            failed++;
        } else {
            printf("[       OK ] %s\n", cases[i].name);
        }
    }
    printf("\n%d/%d suites passed\n", count - failed, count);
    return failed ? 1 : 0;
}
