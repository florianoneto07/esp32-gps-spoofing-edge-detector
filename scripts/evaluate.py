#!/usr/bin/env python3
"""Avaliação offline do detector de borda (sem broker MQTT).

Importa o simulador e o detector de src/ e mede, por cenário:
  - TPR: fração das execuções com ataque em que houve alerta durante o ataque;
  - FPR: amostras em alerta sem ataque ativo / amostras sem ataque ativo;
  - atraso de detecção: do início do ataque até o primeiro alerta (s);
  - cobertura: fração das amostras sob ataque que ficaram em alerta;
  - custo do detector por amostra (µs, CPython nesta máquina).

As duas variantes ("speed+accel" e "speed+accel+dr_divergence") processam
exatamente as mesmas amostras (comparação pareada). Em cada seed, a fase da
janela de re-ancoragem é sorteada para que o atraso não dependa de onde o
início do ataque cai dentro da janela.

A varredura de sensibilidade estima a menor taxa de deriva detectável em
função de RESYNC_S e do viés da velocidade inercial (IMU).

Saídas: results/metrics.csv, results/detection_delay.png,
        results/sensitivity.csv e results/drift_floor.png
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import config  # noqa: E402
from drone_sim import DroneSimulator  # noqa: E402
from edge_detector import ALL_CHECKS, EdgeSpoofDetector  # noqa: E402

NAN = float("nan")

SCENARIOS = (("none", None), ("jump", None), ("drift", 0.5), ("drift", 1.0), ("drift", 2.0), ("drift", 4.0))
VARIANTS = (("speed+accel", ("speed", "accel")), ("speed+accel+dr_divergence", ALL_CHECKS))

SWEEP_RESYNC_S = (10.0, 20.0, 30.0, 60.0)
SWEEP_IMU_BIAS_MPS = (0.0, 0.1, 0.25, 0.5)
SWEEP_DRIFT_MPS = (0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0, 2.8, 4.0)
TPR_TARGET = 0.9  # taxa de detecção exigida para considerar uma deriva detectável
FPR_LIMIT = 0.01  # acima disso a configuração é considerada inviável

# Paleta dos gráficos (fundo claro), validada para daltonismo e contraste.
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SERIES_COLORS = ("#2a78d6", "#eb6834")  # ordem fixa: variante 1, variante 2
BIAS_RAMP = ("#86b6ef", "#3987e5", "#256abf", "#184f95")  # ordinal: viés menor -> maior


@dataclass
class RunStats:
    """Contagens de uma execução para um detector."""

    clean_samples: int = 0  # amostras sem ataque ativo
    false_alert_samples: int = 0  # ... e que estavam em alerta
    attack_samples: int = 0  # amostras com ataque ativo
    attack_alert_samples: int = 0  # ... e que estavam em alerta
    delay_s: Optional[float] = None  # do início do ataque ao 1o alerta


def scenario_label(mode: str, drift_mps: Optional[float]) -> str:
    return mode if drift_mps is None else "%s_%g" % (mode, drift_mps)


def phase_fraction(seed: int) -> float:
    """Fase da janela de re-ancoragem, como fração de RESYNC_S, sorteada por seed."""
    return random.Random("resync-phase-%d" % seed).random()


def nearest_rank(sorted_values: Sequence[float], q: float) -> float:
    if not sorted_values:
        return NAN
    return sorted_values[max(0, math.ceil(q * len(sorted_values)) - 1)]


def simulate(mode, drift_mps, seed, detectors, duration_s, attack_start_s, imu_bias_mps=0.0, timings=None):
    """Roda uma execução e devolve um RunStats por detector (mesmas amostras)."""
    sim = DroneSimulator(seed=seed, imu_bias_sigma=imu_bias_mps)
    n_samples = int(round(duration_s / sim.dt))
    attack_k = int(round(attack_start_s / sim.dt))
    stats = {name: RunStats() for name in detectors}
    attack_t0 = None
    for k in range(n_samples):
        if mode != "none" and k == attack_k:
            sim.set_attack(mode, drift_mps)
        s = sim.step()
        if s.spoof_active and attack_t0 is None:
            attack_t0 = s.t
        for name, det in detectors.items():
            t0 = time.perf_counter_ns()
            r = det.update(s.t, s.lat, s.lon, s.imu_vn, s.imu_ve)
            if timings is not None:
                timings.setdefault(name, []).append((time.perf_counter_ns() - t0) / 1000.0)
            st = stats[name]
            if s.spoof_active:
                st.attack_samples += 1
                if r.alert:
                    st.attack_alert_samples += 1
                    if st.delay_s is None:
                        st.delay_s = s.t - attack_t0
            else:
                st.clean_samples += 1
                if r.alert:
                    st.false_alert_samples += 1
    return stats


def aggregate(runs: Sequence[RunStats], attacked: bool) -> Dict:
    clean = sum(r.clean_samples for r in runs)
    out = {
        "runs": len(runs),
        "tpr": NAN,
        "fpr": sum(r.false_alert_samples for r in runs) / clean if clean else NAN,
        "false_alarm_runs": sum(1 for r in runs if r.false_alert_samples),
        "coverage": NAN,
        "delay_mean_s": NAN,
        "delay_median_s": NAN,
        "delay_p95_s": NAN,
        "delay_max_s": NAN,
        "delays": [],
    }
    if attacked:
        delays = sorted(r.delay_s for r in runs if r.delay_s is not None)
        attack_samples = sum(r.attack_samples for r in runs)
        out["tpr"] = len(delays) / len(runs)
        out["coverage"] = sum(r.attack_alert_samples for r in runs) / attack_samples if attack_samples else NAN
        out["delays"] = delays
        if delays:
            out["delay_mean_s"] = statistics.mean(delays)
            out["delay_median_s"] = statistics.median(delays)
            out["delay_p95_s"] = nearest_rank(delays, 0.95)
            out["delay_max_s"] = delays[-1]
    return out


# ---------------------------------------------------------------------------
# Avaliação principal
# ---------------------------------------------------------------------------


def run_main(seeds, duration_s, attack_start_s):
    results = []
    timings_by_variant: Dict[str, List[float]] = {name: [] for name, _ in VARIANTS}
    for mode, rate in SCENARIOS:
        per_variant: Dict[str, List[RunStats]] = {name: [] for name, _ in VARIANTS}
        timings: Dict[str, List[float]] = {}
        for seed in seeds:
            frac = phase_fraction(seed)
            detectors = {
                name: EdgeSpoofDetector(checks=checks, resync_phase_s=frac * config.RESYNC_S) for name, checks in VARIANTS
            }
            stats = simulate(mode, rate, seed, detectors, duration_s, attack_start_s, timings=timings)
            for name, st in stats.items():
                per_variant[name].append(st)
        for name, _ in VARIANTS:
            agg = aggregate(per_variant[name], attacked=mode != "none")
            us = sorted(timings[name])
            agg.update(
                scenario=scenario_label(mode, rate),
                attack_mode=mode,
                drift_mps=rate,
                variant=name,
                det_us_mean=statistics.mean(us),
                det_us_p99=nearest_rank(us, 0.99),
            )
            results.append(agg)
            timings_by_variant[name].extend(us)
    return results, timings_by_variant


def _num(value, fmt="%.4f"):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return ""
    return fmt % value if isinstance(value, float) else value


def write_metrics_csv(results, path: Path) -> None:
    fields = [
        "scenario",
        "attack_mode",
        "drift_mps",
        "variant",
        "runs",
        "tpr",
        "fpr",
        "false_alarm_runs",
        "coverage",
        "delay_mean_s",
        "delay_median_s",
        "delay_p95_s",
        "delay_max_s",
        "det_us_mean",
        "det_us_p99",
    ]
    with open(str(path), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for r in results:
            writer.writerow({key: _num(r.get(key)) for key in fields})


def _pct(x, decimals=0):
    return "-" if math.isnan(x) else "%.*f%%" % (decimals, 100.0 * x)


def _sec(x):
    return "-" if math.isnan(x) else "%.1f" % x


def print_main_table(results, seeds_n, duration_s, attack_start_s) -> None:
    print()
    print(
        "Avaliação principal: %d seeds x %d s por cenário, ataque em t = %d s "
        "(V_MAX=%g m/s, A_MAX=%g m/s², D_MAX=%g m, RESYNC_S=%g s, K=%d)"
        % (seeds_n, duration_s, attack_start_s, config.V_MAX_MPS, config.A_MAX_MPS2, config.D_MAX_M, config.RESYNC_S, config.K_CONSECUTIVE)
    )
    header = "%-10s %-26s %5s %7s %7s %7s %8s %8s %8s %8s" % (
        "cenário",
        "variante",
        "TPR",
        "FPR",
        "FA/exec",
        "cobert.",
        "atr.méd",
        "atr.med",
        "atr.p95",
        "atr.máx",
    )
    print(header)
    print("-" * len(header))
    for r in results:
        print(
            "%-10s %-26s %5s %7s %7s %7s %8s %8s %8s %8s"
            % (
                r["scenario"],
                r["variant"],
                _pct(r["tpr"]),
                _pct(r["fpr"], 2),
                "%d/%d" % (r["false_alarm_runs"], r["runs"]),
                _pct(r["coverage"], 1),
                _sec(r["delay_mean_s"]),
                _sec(r["delay_median_s"]),
                _sec(r["delay_p95_s"]),
                _sec(r["delay_max_s"]),
            )
        )
    print("atrasos em segundos; FA/exec = execuções com ao menos um alerta falso")


# ---------------------------------------------------------------------------
# Varredura de sensibilidade: RESYNC_S x viés da IMU x taxa de deriva
# ---------------------------------------------------------------------------


def run_sweep(seeds, duration_s, attack_start_s):
    rows = []
    scenarios = [("none", None)] + [("drift", rate) for rate in SWEEP_DRIFT_MPS]
    for bias in SWEEP_IMU_BIAS_MPS:
        for mode, rate in scenarios:
            per_resync: Dict[float, List[RunStats]] = {resync: [] for resync in SWEEP_RESYNC_S}
            for seed in seeds:
                frac = phase_fraction(seed)
                # Os quatro detectores (um por RESYNC_S) veem as mesmas amostras.
                detectors = {
                    resync: EdgeSpoofDetector(resync_s=resync, resync_phase_s=frac * resync) for resync in SWEEP_RESYNC_S
                }
                stats = simulate(mode, rate, seed, detectors, duration_s, attack_start_s, imu_bias_mps=bias)
                for resync, st in stats.items():
                    per_resync[resync].append(st)
            for resync, runs in per_resync.items():
                agg = aggregate(runs, attacked=mode != "none")
                rows.append(
                    {
                        "imu_bias_mps": bias,
                        "resync_s": resync,
                        "scenario": scenario_label(mode, rate),
                        "drift_mps": rate if rate is not None else 0.0,
                        "runs": agg["runs"],
                        "tpr": agg["tpr"],
                        "fpr": agg["fpr"],
                        "false_alarm_runs": agg["false_alarm_runs"],
                        "delay_median_s": agg["delay_median_s"],
                    }
                )
    return rows


def summarize_sweep(rows):
    """Por (viés, RESYNC_S): FPR sem ataque, menor deriva detectável e piso teórico."""
    summary = []
    for bias in SWEEP_IMU_BIAS_MPS:
        for resync in SWEEP_RESYNC_S:
            sel = [r for r in rows if r["imu_bias_mps"] == bias and r["resync_s"] == resync]
            fpr = next(r["fpr"] for r in sel if r["scenario"] == "none")
            tpr_by_rate = {r["drift_mps"]: r["tpr"] for r in sel if r["scenario"] != "none"}
            # Menor taxa a partir da qual TODAS as taxas testadas atingem a meta.
            v_min = None
            for rate in sorted(tpr_by_rate, reverse=True):
                if tpr_by_rate[rate] < TPR_TARGET:
                    break
                v_min = rate
            summary.append(
                {
                    "imu_bias_mps": bias,
                    "resync_s": resync,
                    "fpr": fpr,
                    "feasible": fpr <= FPR_LIMIT,
                    "v_min_mps": v_min,
                    "v_theory_mps": config.D_MAX_M / resync,
                    "tpr_by_rate": tpr_by_rate,
                }
            )
    return summary


def write_sensitivity_csv(rows, path: Path) -> None:
    fields = ["imu_bias_mps", "resync_s", "scenario", "drift_mps", "runs", "tpr", "fpr", "false_alarm_runs", "delay_median_s"]
    with open(str(path), "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow({key: _num(r.get(key)) for key in fields})


def print_sweep_table(summary, seeds_n) -> None:
    print()
    print(
        "Varredura de sensibilidade (%d seeds por ponto, D_MAX=%g m): menor deriva detectável com TPR >= %d%%"
        % (seeds_n, config.D_MAX_M, round(100 * TPR_TARGET))
    )
    print("colunas à direita da barra: TPR por taxa de deriva testada (m/s)")
    rates = " ".join("%5g" % r for r in SWEEP_DRIFT_MPS)
    header = "%-9s %-9s %7s %-12s %-9s | %s" % ("viés IMU", "RESYNC_S", "FPR", "v_min (m/s)", "D/RESYNC", rates)
    print(header)
    print("-" * len(header))
    for s in summary:
        if not s["feasible"]:
            v_min = "inviável"
        elif s["v_min_mps"] is None:
            v_min = "> %g" % SWEEP_DRIFT_MPS[-1]
        else:
            v_min = "%g" % s["v_min_mps"]
        tprs = " ".join("%5s" % _pct(s["tpr_by_rate"][r]) for r in SWEEP_DRIFT_MPS)
        print(
            "%-9s %-9s %7s %-12s %-9s | %s"
            % ("%g m/s" % s["imu_bias_mps"], "%g s" % s["resync_s"], _pct(s["fpr"], 2), v_min, "%.2f" % s["v_theory_mps"], tprs)
        )
    print("inviável = FPR sem ataque > %g%%; D/RESYNC = piso aproximado D_MAX/RESYNC_S" % (100 * FPR_LIMIT))


# ---------------------------------------------------------------------------
# Gráficos
# ---------------------------------------------------------------------------


def _decimal_comma(value) -> str:
    return ("%g" % value).replace(".", ",")


def _style_axes(ax) -> None:
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(colors=MUTED, labelcolor=INK_2, labelsize=8, length=3, width=0.8)
    ax.yaxis.grid(True, color=GRID, linewidth=0.8, linestyle="-")
    ax.set_axisbelow(True)


def _scenario_tick(label: str) -> str:
    if label == "jump":
        return "salto\n150-300 m"
    return "deriva\n%s m/s" % _decimal_comma(float(label.split("_")[1]))


def plot_detection(results, path: Path, seeds_n, duration_s, attack_start_s) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FuncFormatter

    attack_labels = [scenario_label(m, r) for m, r in SCENARIOS if m != "none"]
    by_key = {(r["scenario"], r["variant"]): r for r in results}
    names = [name for name, _ in VARIANTS]
    width, offsets = 0.26, (-0.145, 0.145)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax1, ax2):
        _style_axes(ax)
        ax.set_xticks(range(len(attack_labels)))
        ax.set_xticklabels([_scenario_tick(lbl) for lbl in attack_labels])

    # (a) taxa de detecção por cenário
    for j, name in enumerate(names):
        for i, lbl in enumerate(attack_labels):
            tpr = 100.0 * by_key[(lbl, name)]["tpr"]
            x = i + offsets[j]
            ax1.bar(x, tpr, width=width, color=SERIES_COLORS[j], zorder=2)
            if tpr == 0.0:  # barra de altura zero ficaria invisível
                ax1.text(x, 2.0, "0%", ha="center", va="bottom", fontsize=7, color=MUTED)
    ax1.set_ylim(0, 105)
    ax1.set_yticks([0, 25, 50, 75, 100])
    ax1.yaxis.set_major_formatter(FuncFormatter(lambda v, _: "%d%%" % v))
    ax1.set_title("Taxa de detecção (TPR)", loc="left", fontsize=10, color=INK)

    # (b) atraso de detecção: um ponto por execução detectada + mediana
    jitter = random.Random(0)
    y_max = 1.0
    for j, name in enumerate(names):
        for i, lbl in enumerate(attack_labels):
            delays = by_key[(lbl, name)]["delays"]
            x = i + offsets[j]
            if not delays:
                ax2.text(x, 0.5, "não\ndetec.", ha="center", va="bottom", fontsize=6.5, color=MUTED, linespacing=1.0)
                continue
            xs = [x + jitter.uniform(-0.07, 0.07) for _ in delays]
            ax2.scatter(xs, delays, s=30, color=SERIES_COLORS[j], edgecolors=SURFACE, linewidths=1.0, zorder=3)
            med = statistics.median(delays)
            ax2.hlines(med, x - 0.11, x + 0.11, color=INK, linewidth=2, zorder=4)
            y_max = max(y_max, max(delays))
    ax2.set_ylim(0, y_max * 1.12)
    ax2.set_ylabel("atraso até o alerta (s)", fontsize=8, color=INK_2)
    ax2.set_title("Atraso de detecção (pontos = execuções; traço = mediana)", loc="left", fontsize=10, color=INK)

    fpr_txt = ", ".join(
        "%s: %s" % (name, _pct(by_key[("none", name)]["fpr"], 2).replace(".", ",")) for name in names
    )
    fig.suptitle("Detecção de GPS spoofing na borda: só regras de salto × com dead reckoning", x=0.01, y=0.985, ha="left", fontsize=12, color=INK)
    fig.text(
        0.01,
        0.9,
        "%d seeds × %d s por cenário, ataque em t = %d s. FPR sem ataque (%s)." % (seeds_n, duration_s, attack_start_s, fpr_txt),
        ha="left",
        fontsize=8.5,
        color=INK_2,
    )
    handles = [Line2D([0], [0], marker="s", linestyle="", markersize=8, color=SERIES_COLORS[j], label=name) for j, name in enumerate(names)]
    fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.995, 0.995), ncol=2, frameon=False, fontsize=8.5, labelcolor=INK_2)
    fig.subplots_adjust(left=0.055, right=0.995, bottom=0.15, top=0.8, wspace=0.16)
    fig.savefig(str(path), facecolor=SURFACE)
    plt.close(fig)


def plot_drift_floor(summary, path: Path, seeds_n) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    from matplotlib.ticker import FuncFormatter

    fig, ax = plt.subplots(figsize=(8.0, 4.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    _style_axes(ax)

    n_bias = len(SWEEP_IMU_BIAS_MPS)
    width = 0.17
    for g, resync in enumerate(SWEEP_RESYNC_S):
        for b, bias in enumerate(SWEEP_IMU_BIAS_MPS):
            s = next(x for x in summary if x["resync_s"] == resync and x["imu_bias_mps"] == bias)
            x = g + (b - (n_bias - 1) / 2.0) * (width + 0.02)
            if not s["feasible"]:
                ax.text(x, 0.05, "FPR\n>1%", ha="center", va="bottom", fontsize=6.5, color=MUTED, linespacing=1.0)
            elif s["v_min_mps"] is None:
                ax.text(x, 0.05, "n/d", ha="center", va="bottom", fontsize=6.5, color=MUTED)
            else:
                ax.bar(x, s["v_min_mps"], width=width, color=BIAS_RAMP[b], zorder=2)
        theory = config.D_MAX_M / resync
        half = n_bias / 2.0 * (width + 0.02)
        ax.hlines(theory, g - half, g + half, color=INK, linewidth=1.5, linestyles=(0, (4, 2)), zorder=4)

    ax.set_xticks(range(len(SWEEP_RESYNC_S)))
    ax.set_xticklabels(["RESYNC_S = %g s" % r for r in SWEEP_RESYNC_S])
    top = max(SWEEP_DRIFT_MPS[-1], config.D_MAX_M / SWEEP_RESYNC_S[0]) * 1.08
    ax.set_ylim(0, top)
    ax.set_yticks([r for r in SWEEP_DRIFT_MPS if r >= 0.5])
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: _decimal_comma(v)))
    ax.set_ylabel("menor deriva detectada (m/s)", fontsize=8, color=INK_2)

    fig.suptitle("Piso de detecção de deriva lenta (TPR ≥ %d%%)" % round(100 * TPR_TARGET), x=0.01, y=0.985, ha="left", fontsize=12, color=INK)
    fig.text(
        0.01,
        0.9,
        "%d seeds × 300 s por ponto; taxas testadas de %s a %s m/s; sem barra: FPR sem ataque > 1%% (inviável) ou n/d."
        % (seeds_n, _decimal_comma(SWEEP_DRIFT_MPS[0]), _decimal_comma(SWEEP_DRIFT_MPS[-1])),
        ha="left",
        fontsize=8.5,
        color=INK_2,
    )
    handles = [Patch(color=BIAS_RAMP[b], label="viés IMU σ = %s m/s" % _decimal_comma(bias)) for b, bias in enumerate(SWEEP_IMU_BIAS_MPS)]
    handles.append(Line2D([0], [0], color=INK, linewidth=1.5, linestyle=(0, (4, 2)), label="piso teórico D_MAX/RESYNC_S"))
    ax.legend(handles=handles, loc="upper right", frameon=False, fontsize=7.5, labelcolor=INK_2)
    fig.subplots_adjust(left=0.085, right=0.99, bottom=0.09, top=0.84)
    fig.savefig(str(path), facecolor=SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------------------


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Avaliação offline do detector de GPS spoofing na borda.")
    parser.add_argument("--seeds", type=int, default=20, help="seeds por cenário (padrão: %(default)s)")
    parser.add_argument("--duration", type=int, default=300, help="duração de cada execução em s (padrão: %(default)s)")
    parser.add_argument("--attack-start", type=int, default=100, help="início do ataque em s (padrão: %(default)s)")
    parser.add_argument("--sweep-seeds", type=int, default=10, help="seeds por ponto da varredura (padrão: %(default)s)")
    parser.add_argument("--no-sweep", action="store_true", help="pula a varredura de sensibilidade")
    parser.add_argument("--out", default=str(ROOT / "results"), help="pasta de saída (padrão: results/)")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = list(range(args.seeds))

    t0 = time.time()
    results, timings = run_main(seeds, args.duration, args.attack_start)
    write_metrics_csv(results, out / "metrics.csv")
    print_main_table(results, len(seeds), args.duration, args.attack_start)
    full = sorted(timings["speed+accel+dr_divergence"])
    print(
        "\nCusto do detector completo por amostra (CPython nesta máquina, %d amostras): "
        "média %.1f µs | p50 %.1f | p99 %.1f | máx %.1f µs"
        % (len(full), statistics.mean(full), nearest_rank(full, 0.5), nearest_rank(full, 0.99), full[-1])
    )
    plot_detection(results, out / "detection_delay.png", len(seeds), args.duration, args.attack_start)

    if not args.no_sweep:
        sweep_seeds = list(range(args.sweep_seeds))
        rows = run_sweep(sweep_seeds, args.duration, args.attack_start)
        summary = summarize_sweep(rows)
        write_sensitivity_csv(rows, out / "sensitivity.csv")
        print_sweep_table(summary, len(sweep_seeds))
        plot_drift_floor(summary, out / "drift_floor.png", len(sweep_seeds))

    print("\nArquivos gerados em %s (%.1f s)" % (out, time.time() - t0))
    for name in ("metrics.csv", "detection_delay.png", "sensitivity.csv", "drift_floor.png"):
        if (out / name).exists():
            print("  - " + name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
