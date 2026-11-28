import math
import pytest
from whill_wheel_odometry.kinematics import DriveModel, WheelOdometry, covariance6


def test_straight_and_reverse():
    m = DriveModel()
    e = WheelOdometry(m)
    assert m.wheel_speeds(3.6, 3.6) == pytest.approx((1.0, 1.0))
    e.integrate(1.0, 1.0, 1.0)
    assert e.base_pose() == pytest.approx((1.0, 0.0, 0.0))
    e.integrate(-1.0, -1.0, 1.0)
    assert e.base_pose() == pytest.approx((0.0, 0.0, 0.0))


def test_spin():
    m = DriveModel(wheel_tread=0.5)
    e = WheelOdometry(m)
    e.integrate(-0.25, 0.25, math.pi/2)
    assert e.base_pose() == pytest.approx((0.0, 0.0, math.pi/2))
    assert m.base_twist(-0.25, 0.25) == pytest.approx((0, 0, 1))


def test_arc():
    m = DriveModel(wheel_tread=0.5)
    e = WheelOdometry(m)
    e.integrate(0.25, 0.75, math.pi/2)
    assert e.base_pose() == pytest.approx((0.5, 0.5, math.pi/2))


def test_offset():
    m = DriveModel(wheel_tread=0.5, axle_to_base_x=0.3, axle_to_base_y=0.1)
    e = WheelOdometry(m)
    assert e.base_pose() == pytest.approx((0, 0, 0))
    assert m.base_twist(-0.25, 0.25) == pytest.approx((-0.1, 0.3, 1.0))
    e.integrate(-0.25, 0.25, math.pi/2)
    assert e.base_pose() == pytest.approx((-0.4, 0.2, math.pi/2))


def test_sign_scale_and_invalid_speed():
    m = DriveModel(left_sign=-1, right_scale=1.2)
    assert m.wheel_speeds(-3.6, 3.6) == pytest.approx((1.0, 1.2))
    for value in (math.nan, math.inf, 100.0):
        with pytest.raises(ValueError):
            m.wheel_speeds(value, 0.0)
    with pytest.raises(ValueError):
        DriveModel(wheel_tread=0.0)


def test_invalid_dt_and_gap():
    e = WheelOdometry(DriveModel())
    for dt in (0.0, -1.0, math.nan):
        with pytest.raises(ValueError):
            e.integrate(0.0, 0.0, dt)
    e.invalidate_pose()
    e.integrate(0.1, 0.1, 0.1)
    assert not e.pose_valid
    assert e.pose_covariance()[0][0] == 1e6


def test_covariance():
    m = DriveModel(wheel_tread=0.5, axle_to_base_x=0.3)
    e = WheelOdometry(m)
    for _ in range(10):
        e.integrate(0.1, 0.2, 0.05)
    for p in (e.pose_covariance(), m.twist_covariance()):
        for i in range(3):
            assert p[i][i] > 0.0
            for j in range(3):
                assert p[i][j] == pytest.approx(p[j][i])
        c = covariance6(p)
        assert len(c) == 36
        assert c[35] == p[2][2]
        assert c[14] == c[21] == c[28] == 1e6
