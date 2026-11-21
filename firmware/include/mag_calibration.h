/* Fixed experimental candidate, NOT physical mounting/heading acceptance.
 * Generated from config/bmm150-calibration-candidate.json; tests lock its hash.
 * Chip correction precedes independent mounting rotation. Raw output unchanged. */
#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H
#define MAG_CALIBRATION_CONFIG_SHA256 "4704802c8e345239c8809bd78c4b4b79dc00761667a048ac36189fc84ebffc30"
static const float mag_bias_ut[3] = {-20.51479881f, -14.69928811f, -4.728956551f};
static const float mag_correction[3][3] = {
    {1.007948187f, -0.005386536907f, 0.002291617838f},
    {-0.005386536907f, 0.9825200371f, -0.01062539415f},
    {0.002291617838f, -0.01062539415f, 1.009914588f}
};
#define MAG_REFERENCE_NORM_UT 47.88960059f
#endif
