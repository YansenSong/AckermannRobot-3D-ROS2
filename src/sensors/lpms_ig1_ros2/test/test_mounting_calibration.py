import math
from lpms_ig1_ros2.mounting_calibration import G0, estimate_mounting_rp, rotate_vector, rpy_matrix, transpose

def gravity_in_imu(r, p, y=0.0):
    return rotate_vector(transpose(rpy_matrix(r, p, y)), (0.0, 0.0, G0))

def test_level_and_single_axis_tilts():
    for r, p in ((0, 0), (math.radians(5), 0), (math.radians(-5), 0),
                 (0, math.radians(5)), (0, math.radians(-5))):
        actual = estimate_mounting_rp(gravity_in_imu(r, p))
        assert math.isclose(actual[0], r, abs_tol=1e-12)
        assert math.isclose(actual[1], p, abs_tol=1e-12)

def test_tf_direction_combined_rotation():
    r, p, y = map(math.radians, (5, -3, 10))
    measured = gravity_in_imu(r, p, y)
    estimated = estimate_mounting_rp(measured)
    assert math.isclose(estimated[0], r, abs_tol=1e-12)
    assert math.isclose(estimated[1], p, abs_tol=1e-12)
    corrected = rotate_vector(rpy_matrix(*estimated, y), measured)
    assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(corrected, (0, 0, G0)))
