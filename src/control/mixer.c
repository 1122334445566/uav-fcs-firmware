#include "fcs/mixer.h"
#include "fcs/fcs_math.h"

#include <string.h>

/*             roll   pitch   yaw
 *  0 FR CCW  | -1     +1     +1
 *  1 RR CW   | -1     -1     -1
 *  2 RL CCW  | +1     -1     +1
 *  3 FL CW   | +1     +1     -1
 *
 * Positive roll command lowers the right side -> lift LEFT motors (2,3).
 * Positive pitch command lowers the nose    -> lift FRONT motors (0,3).
 * Positive yaw (NED clockwise seen from above) -> spin up the CCW rotors.
 */
const float fcs_xquad_matrix[FCS_MIXER_XQUAD_MOTORS][3] = {
    { -1.0f,  1.0f,  1.0f },
    { -1.0f, -1.0f, -1.0f },
    {  1.0f, -1.0f,  1.0f },
    {  1.0f,  1.0f, -1.0f }
};

static const float MIX_GAIN = 0.5f;

void fcs_mixer_mix(const fcs_control_t *control,
                   const fcs_params_t *p,
                   float *outputs,
                   bool *saturated)
{
    float min_v = 1e9f, max_v = -1e9f;
    if (saturated) *saturated = false;

    for (uint8_t i = 0; i < FCS_MIXER_XQUAD_MOTORS; ++i) {
        float v = control->thrust
                + MIX_GAIN * (control->torque.x * fcs_xquad_matrix[i][0]
                            + control->torque.y * fcs_xquad_matrix[i][1]
                            + control->torque.z * fcs_xquad_matrix[i][2]);
        outputs[i] = v;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
    }

    /* Attitude-priority saturation: shift the collective, keep motor
     * differences (and therefore roll/pitch/yaw authority) intact. */
    if (max_v > p->throttle_max) {
        float shift = max_v - p->throttle_max;
        for (uint8_t i = 0; i < FCS_MIXER_XQUAD_MOTORS; ++i) outputs[i] -= shift;
        min_v -= shift;
        if (saturated) *saturated = true;
    }
    if (min_v < p->throttle_min) {
        float shift = p->throttle_min - min_v;
        for (uint8_t i = 0; i < FCS_MIXER_XQUAD_MOTORS; ++i) outputs[i] += shift;
        if (saturated) *saturated = true;
    }
    for (uint8_t i = 0; i < FCS_MIXER_XQUAD_MOTORS; ++i) {
        float v = fcs_clampf(outputs[i], p->throttle_min, p->throttle_max);
        if (saturated && v != outputs[i]) *saturated = true;
        outputs[i] = v;
    }
}

void fcs_mixer_disarm(float *outputs, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) outputs[i] = 0.0f;
}

fcs_vec3_t fcs_mixer_torque_from_outputs(const float *outputs)
{
    fcs_vec3_t t = fcs_vec3(0.0f, 0.0f, 0.0f);
    for (uint8_t i = 0; i < FCS_MIXER_XQUAD_MOTORS; ++i) {
        t.x += outputs[i] * fcs_xquad_matrix[i][0];
        t.y += outputs[i] * fcs_xquad_matrix[i][1];
        t.z += outputs[i] * fcs_xquad_matrix[i][2];
    }
    /* Inverse of the MIX_GAIN * sum(M^2)=2 relation used in fcs_mixer_mix. */
    return fcs_vec3_scale(t, 0.5f);
}
