/**
 * @file mixer.h
 * @brief Control allocation for an X-quadrotor (and generic matrix mixer).
 *
 * Motor numbering (viewed from above, nose = +x):
 *
 *        3 (FL, CW)        0 (FR, CCW)
 *              \          /
 *               \   +x   /
 *                [ FCS ]
 *               /        \
 *        2 (RL, CCW)      1 (RR, CW)
 *
 * Rotor spin direction follows the PX4 "X" convention. Changing the frame
 * only requires replacing the mixing matrix in mixer.c; the controller is
 * completely frame agnostic.
 */
#ifndef FCS_MIXER_H
#define FCS_MIXER_H

#include <stdint.h>
#include "fcs/fcs_types.h"
#include "fcs/params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FCS_MIXER_XQUAD_MOTORS 4u

/** roll/pitch/yaw coefficients for motors 0..3. */
extern const float fcs_xquad_matrix[FCS_MIXER_XQUAD_MOTORS][3];

/**
 * Map (thrust, torques) to per-motor commands in [0,1].
 *
 * Saturation policy: preserve the *difference* between motors (i.e. attitude
 * authority) and shift the collective until all outputs fit, because losing
 * attitude control is far worse than losing a little lift.
 *
 * @param outputs  destination array of at least 4 floats
 * @param saturated optional out flag, true when the mixer had to clip
 */
void fcs_mixer_mix(const fcs_control_t *control,
                   const fcs_params_t *p,
                   float *outputs,
                   bool *saturated);

/** Zero all motor outputs (disarmed / failsafe kill). */
void fcs_mixer_disarm(float *outputs, uint8_t count);

/** Compute the normalised body torque produced by motor commands (for SIL/tests). */
fcs_vec3_t fcs_mixer_torque_from_outputs(const float *outputs);

#ifdef __cplusplus
}
#endif

#endif /* FCS_MIXER_H */
