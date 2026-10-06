from imu.ros_param_yaml import format_double


def test_whole_numbers_keep_a_decimal_point():
    # A bare "0"/"1" is typed INTEGER by rcl_yaml_param_parser and rejected for
    # double parameters, which is what broke lpms_ig1_node startup.
    assert format_double(0.0) == "0.0"
    assert format_double(1.0) == "1.0"
    assert format_double(-1.0) == "-1.0"
    assert format_double(-0.0) == "-0.0"


def test_fractional_values_unchanged():
    assert format_double(-0.032090619572) == "-0.032090619572"
    assert format_double(0.5) == "0.5"
    assert format_double(1.00405671301) == "1.00405671301"


def test_exponent_form_is_preserved_only_when_g_uses_it():
    # %.12g only switches to exponent form outside (1e-5, 1e12), so a value like
    # 1e-05 keeps its exponent while 1e5 comes back as plain digits.
    assert format_double(1e-05) == "1e-05"
    assert format_double(1e5) == "100000.0"


def test_output_always_carries_a_decimal_point_or_exponent():
    for value in (0.0, 1.0, -1.0, 1e-05, 1e5, -0.032090619572, 1234567890.0):
        text = format_double(value)
        assert "." in text or "e" in text or "E" in text
