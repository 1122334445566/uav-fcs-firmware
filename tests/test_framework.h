/** Minimal dependency-free test harness (no unity/CMocka needed). */
#ifndef FCS_TEST_FRAMEWORK_H
#define FCS_TEST_FRAMEWORK_H

#include <math.h>
#include <stdio.h>

#define CHECK(cond) do {                                                    \
    if (!(cond)) {                                                          \
        fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
        return 1;                                                           \
    }                                                                       \
} while (0)

#define CHECK_NEAR(a, b, tol) do {                                          \
    float _va = (float)(a), _vb = (float)(b), _vt = (float)(tol);           \
    if (!isfinite((double)_va) || fabsf(_va - _vb) > _vt) {                 \
        fprintf(stderr, "  FAIL %s:%d: %s (%.6f) != %s (%.6f) tol %.6f\n",  \
                __FILE__, __LINE__, #a, (double)_va, #b, (double)_vb,        \
                (double)_vt);                                                \
        return 1;                                                           \
    }                                                                       \
} while (0)

#define CHECK_TRUE(v)  CHECK((v) ? 1 : 0)
#define CHECK_FALSE(v) CHECK((v) ? 0 : 1)

#endif /* FCS_TEST_FRAMEWORK_H */
