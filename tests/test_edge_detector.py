"""Testes do detector de borda, das funções geodésicas e do simulador (sem broker)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import config  # noqa: E402
from drone_sim import DroneSimulator, parse_spoof_command  # noqa: E402
from edge_detector import ALL_CHECKS, EdgeSpoofDetector  # noqa: E402
from geo import EARTH_RADIUS_M, bearing_deg, haversine_m, offset_latlon  # noqa: E402

SEEDS = range(5)
ATTACK_START_S = 100


def run_scenario(mode="none", seed=0, drift_mps=None, checks=ALL_CHECKS, duration_s=300):
    """Roda uma execução offline de 1 Hz e devolve a lista (amostra, resultado)."""
    sim = DroneSimulator(seed=seed)
    det = EdgeSpoofDetector(checks=checks)
    history = []
    for k in range(duration_s):
        if mode != "none" and k == ATTACK_START_S:
            sim.set_attack(mode, drift_mps)
        s = sim.step()
        history.append((s, det.update(s.t, s.lat, s.lon, s.imu_vn, s.imu_ve)))
    return history


def first_alert(history):
    return next(((s, r) for s, r in history if r.alert), (None, None))


# --- geo ------------------------------------------------------------------------


def test_haversine_one_degree_of_latitude():
    one_degree = EARTH_RADIUS_M * math.pi / 180.0  # ~111,2 km
    assert haversine_m(0.0, 0.0, 1.0, 0.0) == pytest.approx(one_degree, rel=1e-9)
    assert haversine_m(-23.0, -46.0, -24.0, -46.0) == pytest.approx(one_degree, rel=1e-9)


def test_haversine_quarter_circle_symmetry_and_zero():
    quarter = EARTH_RADIUS_M * math.pi / 2.0
    assert haversine_m(0.0, 0.0, 90.0, 0.0) == pytest.approx(quarter, rel=1e-9)
    assert haversine_m(0.0, 0.0, 0.0, 90.0) == pytest.approx(quarter, rel=1e-9)
    sao_paulo = (config.HOME_LAT, config.HOME_LON)
    rio = (-22.9068, -43.1729)
    assert haversine_m(*sao_paulo, *rio) == pytest.approx(haversine_m(*rio, *sao_paulo))
    assert haversine_m(*sao_paulo, *sao_paulo) == 0.0


def test_offset_latlon_matches_haversine_and_bearing():
    lat, lon = offset_latlon(config.HOME_LAT, config.HOME_LON, north_m=300.0, east_m=400.0)
    assert haversine_m(config.HOME_LAT, config.HOME_LON, lat, lon) == pytest.approx(500.0, rel=1e-3)
    expected_bearing = math.degrees(math.atan2(400.0, 300.0))  # ~53,13 graus
    assert bearing_deg(config.HOME_LAT, config.HOME_LON, lat, lon) == pytest.approx(expected_bearing, abs=0.1)


def test_bearing_cardinal_directions():
    assert bearing_deg(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert bearing_deg(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0, abs=1e-9)
    assert bearing_deg(0.0, 0.0, -1.0, 0.0) == pytest.approx(180.0, abs=1e-9)
    assert bearing_deg(0.0, 0.0, 0.0, -1.0) == pytest.approx(270.0, abs=1e-9)


# --- detector + simulador -----------------------------------------------------------


@pytest.mark.parametrize("seed", SEEDS)
def test_no_attack_no_alert_in_300s(seed):
    history = run_scenario("none", seed=seed)
    assert len(history) == 300
    assert not any(r.alert for _, r in history)


@pytest.mark.parametrize("seed", SEEDS)
def test_jump_alert_within_3s(seed):
    history = run_scenario("jump", seed=seed)
    sample, _ = first_alert(history)
    assert sample is not None and sample.spoof_active
    assert sample.t - ATTACK_START_S <= 3.0
    # Com a âncora do dead reckoning congelada, o alerta se mantém durante o ataque.
    assert all(r.alert for s, r in history if s.t >= sample.t)


@pytest.mark.parametrize("seed", SEEDS)
def test_drift_2mps_caught_by_dr_divergence_but_not_by_speed_rule(seed):
    sample, result = first_alert(run_scenario("drift", seed=seed, drift_mps=2.0))
    assert sample is not None and sample.spoof_active
    assert result.flags["dr_divergence"]
    assert "dr_divergence" in result.alert_type

    speed_only = run_scenario("drift", seed=seed, drift_mps=2.0, checks=("speed",))
    assert not any(r.alert for _, r in speed_only)
    assert not any(r.flags["speed"] for _, r in speed_only)  # nem chega a marcar suspeita


def test_single_accel_spike_needs_k_consecutive_samples():
    # Trajetória reta para o norte a 8 m/s com um desvio de 5 m para leste numa
    # única amostra: a aceleração implícita fica 5, 10 e 5 m/s² nas amostras
    # seguintes, então só uma ultrapassa A_MAX = 8 m/s².
    def run(k_consecutive):
        det = EdgeSpoofDetector(k_consecutive=k_consecutive)
        results = []
        for k in range(20):
            east = 5.0 if k == 10 else 0.0
            lat, lon = offset_latlon(config.HOME_LAT, config.HOME_LON, north_m=8.0 * k, east_m=east)
            results.append(det.update(float(k), lat, lon, imu_vn=8.0, imu_ve=0.0))
        return results

    with_k2 = run(2)
    assert sum(r.flags["accel"] for r in with_k2) == 1
    assert not any(r.alert for r in with_k2)
    assert any(r.alert for r in run(1))


def test_attack_changes_gps_but_not_imu():
    clean = DroneSimulator(seed=3)
    attacked = DroneSimulator(seed=3)
    attacked.set_attack("jump")
    for _ in range(50):
        a, b = clean.step(), attacked.step()
        assert (a.imu_vn, a.imu_ve) == (b.imu_vn, b.imu_ve)
        assert (a.true_lat, a.true_lon) == (b.true_lat, b.true_lon)
        offset = haversine_m(a.lat, a.lon, b.lat, b.lon)
        assert config.JUMP_MIN_M - 1.0 <= offset <= config.JUMP_MAX_M + 1.0


def test_parse_spoof_command():
    assert parse_spoof_command(b'{"mode":"drift","drift_mps":2.0}') == ("drift", 2.0)
    assert parse_spoof_command(b'{"mode":"drift"}') == ("drift", config.DRIFT_DEFAULT_MPS)
    assert parse_spoof_command(b'{"mode":"none"}') == ("none", None)
    assert parse_spoof_command(b'{"mode":"JUMP"}') == ("jump", None)
    invalid = (
        b"nao e json",
        b"[1, 2]",
        b'{"mode":"teleport"}',
        b'{"mode":"drift","drift_mps":-1}',
        b'{"mode":"drift","drift_mps":"rapido"}',
    )
    for payload in invalid:
        with pytest.raises(ValueError):
            parse_spoof_command(payload)
