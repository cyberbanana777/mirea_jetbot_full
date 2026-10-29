# Copyright (c) 2026 Alice Zenina, Alexander Grachev and Ilya Monakhov RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
"""Графики для steptest.py и mtcal.py. Нужен matplotlib (pip install matplotlib); без него скрипты работают, просто без графиков."""
import ui
import os

import numpy as np

C_L, C_R = "#1f77b4", "#e07b00"   # левое / правое колесо


def _plt(show):
    try:
        import matplotlib
        if not show or not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        ui.warn("Графики не построены: установите matplotlib (pip install matplotlib).")
        return None


def _finish(plt, fig, path, show):
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=130)
        ui.ok("График сохранён: %s" % path)
    if show:
        plt.show()
    plt.close(fig)


def plot_steps(log, levels, rows, path="step_plot.png", show=False):
    """log: (уровень, фаза, t, wL, wR, uL, uR); rows: результат analyze() steptest.py"""
    plt = _plt(show)
    if plt is None:
        return
    n = len(levels)
    fig, axs = plt.subplots(2, n, figsize=(5.2 * n, 6.4), squeeze=False, sharex="col",
                            gridspec_kw={"height_ratios": [3, 1.6]})
    for k, lev in enumerate(levels):
        pre = np.array([[r[2], r[3], r[4], r[5], r[6]] for r in log if r[0] == lev and r[1] == "pre"])
        stp = np.array([[r[2], r[3], r[4], r[5], r[6]] for r in log if r[0] == lev and r[1] == "step"])
        if not len(stp):
            continue
        t0 = stp[0, 0]
        if len(pre):
            pre = pre[pre[:, 0] > t0 - 0.3]
        a = np.vstack([pre, stp]) if len(pre) else stp
        t = a[:, 0] - t0
        ax, au = axs[0, k], axs[1, k]
        ax.axhspan(0.9 * lev, 1.1 * lev, color="0.88", label="±10 %")
        ax.plot([0, t[-1]], [lev, lev], "k--", lw=1.2, label="уставка")
        ax.plot(t, a[:, 1], color=C_L, lw=1.4, label="L")
        ax.plot(t, a[:, 2], color=C_R, lw=1.4, label="R")
        ax.axvline(0, color="0.6", lw=0.8)
        title = "уставка %.2f об/с" % lev
        for r in rows:
            if r["level"] == lev:
                t90 = "—" if r["t90"] is None else "%.0f мс" % (r["t90"] * 1000)
                title += "\n%s: t90 %s, перерег. %.0f %%" % (r["wheel"], t90, r["over"])
        ax.set_title(title, fontsize=10)
        ax.set_ylabel("скорость, об/с")
        ax.grid(alpha=0.3)
        au.plot(t, a[:, 3], color=C_L, lw=1.2)
        au.plot(t, a[:, 4], color=C_R, lw=1.2)
        au.axvline(0, color="0.6", lw=0.8)
        au.set_ylabel("ШИМ")
        au.set_xlabel("время от ступеньки, с")
        au.grid(alpha=0.3)
    axs[0, 0].legend(loc="lower right", fontsize=8)
    _finish(plt, fig, path, show)


def plot_cal(log, st, br, sp, path="cal_plot.png", show=False):
    """st, br, sp: результаты fit_static, fit_breakaway, fit_step из mtcal.py"""
    plt = _plt(show)
    if plt is None:
        return
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.8))

    ax = axs[0]
    for name, wi, ui, c in (("L", 1, 3, C_L), ("R", 2, 4, C_R)):
        a = np.array([[r[1], r[2], r[3], r[4], r[5]] for r in log if r[0] == "ramp"])
        if len(a):
            ax.plot(a[:, ui], a[:, wi], color=c, lw=1.2, label=name)
            if br.get(name) is not None:
                ax.axvline(br[name], color=c, ls=":", lw=1)
    ax.set_title("A. Плавный подъём ШИМ: страгивание (пунктир)")
    ax.set_xlabel("ШИМ")
    ax.set_ylabel("скорость, об/с")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = axs[1]
    for name, c in (("L", C_L), ("R", C_R)):
        kv, dz, r2, pts, top = st[name]
        d = np.array([p[0] for p in pts]); w = np.array([p[1] for p in pts])
        ax.plot(d, w, "o", color=c, label="%s: точки" % name)
        dd = np.linspace(dz, max(d.max(), 1.0) * 0.6, 20)
        ax.plot(dd, kv * (dd - dz), "-", color=c, lw=1.2, label="%s: Kv=%.2f, dz=%.3f (R²=%.3f)" % (name, kv, dz, r2))
    ax.set_title("B. Статика: скорость от ШИМ и прямая подгонки")
    ax.set_xlabel("ШИМ")
    ax.set_ylabel("установившаяся скорость, об/с")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = axs[2]
    a = np.array([[r[1], r[2], r[3], r[4], r[5]] for r in log if r[0] == "step"])
    if len(a):
        t = a[:, 0] - a[0, 0]
        ax.plot(t, a[:, 1], color=C_L, lw=1.2, label="L: Tm=%.0f мс, L=%.0f мс" % (sp["L"][0] * 1000, sp["L"][1] * 1000))
        ax.plot(t, a[:, 2], color=C_R, lw=1.2, label="R: Tm=%.0f мс, L=%.0f мс" % (sp["R"][0] * 1000, sp["R"][1] * 1000))
        ax2 = ax.twinx()
        ax2.plot(t, a[:, 3], color=C_L, lw=0.8, alpha=0.5, drawstyle="steps-post")
        ax2.plot(t, a[:, 4], color=C_R, lw=0.8, alpha=0.5, drawstyle="steps-post")
        ax2.set_ylabel("ШИМ (бледные линии)")
    ax.set_title("C. Ступенька ШИМ: постоянная времени")
    ax.set_xlabel("время, с")
    ax.set_ylabel("скорость, об/с")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")
    _finish(plt, fig, path, show)


def plot_compare(logs, names, levels, wheel_r, base, path="step_compare.png", show=False):
    """Наложение ступенек нескольких логов (например, ПИ и LADRC): скорости L/R и набег курса"""
    plt = _plt(show)
    if plt is None:
        return
    two_pi = 2.0 * np.pi
    styles = ["-", "--", ":", "-."]
    fig, axs = plt.subplots(2, len(levels), figsize=(5.2 * len(levels), 7), squeeze=False,
                            gridspec_kw={"height_ratios": [2, 1]})
    for j, lev in enumerate(levels):
        ax, ay = axs[0, j], axs[1, j]
        ax.axhline(lev, color="0.6", lw=0.8, ls=":")
        for k, (log, name) in enumerate(zip(logs, names)):
            a = np.array([[r[2], r[3], r[4]] for r in log if r[0] == lev and r[1] == "step"])
            if len(a) < 2:
                continue
            t = a[:, 0] - a[0, 0]
            ls = styles[k % len(styles)]
            ax.plot(t, a[:, 1], color=C_L, lw=1.3, ls=ls, label="%s L" % name)
            ax.plot(t, a[:, 2], color=C_R, lw=1.3, ls=ls, label="%s R" % name)
            dt = np.diff(a[:, 0], prepend=a[0, 0])
            yaw = np.cumsum((a[:, 2] - a[:, 1]) * two_pi * wheel_r / base * dt) * 180 / np.pi
            ay.plot(t, yaw, color="0.2", lw=1.3, ls=ls, label=name)
        ax.set_title("уставка %.2f об/с" % lev, fontsize=10)
        ax.set_ylabel("скорость, об/с")
        ax.grid(alpha=0.3)
        ay.axhline(0, color="0.6", lw=0.8)
        ay.set_ylabel("набег курса, °")
        ay.set_xlabel("время от ступеньки, с")
        ay.grid(alpha=0.3)
    axs[0, 0].legend(fontsize=8, loc="lower right")
    axs[1, 0].legend(fontsize=8)
    _finish(plt, fig, path, show)
