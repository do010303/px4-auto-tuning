#!/usr/bin/env python3
"""Figures for the in-flight tuners (online_tuner.py, bayes_tuner.py).

    python3 -m px4_auto_tuning.tuner_plots               # results/tuner_*.png
    python3 -m px4_auto_tuning.tuner_plots --lang vi --out-dir results/raw/deck

  * tuner_convergence: one panel per tuner; tracking error of every lap it
    flew and of the best gains it had found so far, against flight time
  * tuner_search_map: where each tuner looked in the MPC_XY_P x
    MPC_XY_VEL_P_ACC plane, on top of the offline grid sweep
  * tuner_before_after: tracking error before and after tuning for every run
"""

import argparse
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

BLUE = 'tab:blue'
ORANGE = 'tab:orange'
GREEN = 'tab:green'
RED = 'tab:red'
GREY = '#9AA0A6'
SIZE = (16.64, 5.4)     # wide: fills a 16:9 slide below its title

GAIN_X, GAIN_Y = 'MPC_XY_P', 'MPC_XY_VEL_P_ACC'
UNSTABLE = 0.3          # m, grid cells above this are marked unstable

COORDINATE_RUNS = ['online_tuner_x500', 'online_tuner_x500_rep2', 'online_tuner_x500_heavy', 'online_tuner_x500_long',
                   'online_tuner_x500_weak', 'online_tuner_noisy_pos', 'online_tuner_noisy_vel', 'online_tuner_with_i']
BAYES_RUNS = ['bayes_tuner_x500', 'bayes_tuner_x500_seed1', 'bayes_tuner_x500_3gains']

TEXT = {
    'en': {
        'time': 'Flight time [s]',
        'rmse': 'Tracking error, RMSE [m]',
        'coordinate': 'Coordinate search, 2 gains',
        'bayes2': 'Bayes, 2 gains',
        'bayes3': 'Bayes, 3 gains',
        'lap': 'one lap with trial gains',
        'best': 'best gains so far',
        'reverted': 'reverted by the watchdog',
        'after_s': '{value:.3f} m\nafter {time:.0f} s',
        'default': 'default gains',
        'final': 'final gains',
        'accepted': 'accepted step',
        'rejected': 'rejected step',
        'seed': 'Bayes trial, seed {n}',
        'grid': 'Offline grid sweep: tracking RMSE [m]',
        'unstable': 'unstable in the grid sweep',
        'map_coordinate': 'Coordinate search',
        'map_bayes': 'Bayes, 2 seeds of 25 trials',
        'before': 'before (default gains)',
        'after_coordinate': 'after, coordinate search',
        'after_bayes': 'after, Bayes',
        'runs': {
            'online_tuner_x500': 'x500', 'online_tuner_x500_rep2': 'x500\nrepeat',
            'online_tuner_x500_heavy': 'heavy', 'online_tuner_x500_long': 'long\narms',
            'online_tuner_x500_weak': 'weak\nmotors', 'online_tuner_noisy_pos': 'GPS pos.\nnoise',
            'online_tuner_noisy_vel': 'GPS vel.\nnoise', 'online_tuner_with_i': 'x500\n3 gains',
            'bayes_tuner_x500': 'x500\n2 gains', 'bayes_tuner_x500_seed1': '2 gains\nseed 1',
            'bayes_tuner_x500_3gains': 'x500\n3 gains',
        },
    },
    'vi': {
        'time': 'Thời gian bay [s]',
        'rmse': 'Sai số tracking, RMSE [m]',
        'coordinate': 'Tìm theo toạ độ, 2 gain',
        'bayes2': 'Bayes, 2 gain',
        'bayes3': 'Bayes, 3 gain',
        'lap': 'một vòng bay với gain thử',
        'best': 'gain tốt nhất tới lúc đó',
        'reverted': 'watchdog trả gain cũ',
        'after_s': '{value:.3f} m\nsau {time:.0f} s',
        'default': 'gain mặc định',
        'final': 'gain cuối',
        'accepted': 'bước được nhận',
        'rejected': 'bước bị từ chối',
        'seed': 'lần thử của Bayes, seed {n}',
        'grid': 'Lưới quét offline: tracking RMSE [m]',
        'unstable': 'mất ổn định trong lưới quét',
        'map_coordinate': 'Tìm theo toạ độ',
        'map_bayes': 'Bayes, 2 seed × 25 lần thử',
        'before': 'trước (gain mặc định)',
        'after_coordinate': 'sau, tìm theo toạ độ',
        'after_bayes': 'sau, Bayes',
        'runs': {
            'online_tuner_x500': 'x500', 'online_tuner_x500_rep2': 'x500\nlặp lại',
            'online_tuner_x500_heavy': 'nặng', 'online_tuner_x500_long': 'tay dài',
            'online_tuner_x500_weak': 'motor\nyếu', 'online_tuner_noisy_pos': 'GPS nhiễu\nvị trí',
            'online_tuner_noisy_vel': 'GPS nhiễu\nvận tốc', 'online_tuner_with_i': 'x500\n3 gain',
            'bayes_tuner_x500': 'x500\n2 gain', 'bayes_tuner_x500_seed1': '2 gain\nseed 1',
            'bayes_tuner_x500_3gains': 'x500\n3 gain',
        },
    },
}


def load(results, name):
    """Log of one tuner run, with the time counted from its baseline lap."""
    log = pd.read_csv(os.path.join(results, f'{name}.csv'))
    log['t'] = log['px4_time_s'] - log['px4_time_s'].iloc[0]
    return log


def summary(log):
    """Tracking RMSE before and after, and the flight time of the whole run."""
    return log['rmse'].iloc[0], log[log['action'] == 'validate']['rmse'].mean(), log['t'].iloc[-1]


def save(fig, out_dir, name):
    path = os.path.join(out_dir, f'{name}.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(path)


def convergence(results, out_dir, text):
    fig, axes = plt.subplots(1, 3, figsize=SIZE, sharex=True, sharey=True)
    top = 0.30
    runs = [('online_tuner_x500', text['coordinate'], BLUE), ('bayes_tuner_x500', text['bayes2'], ORANGE),
            ('bayes_tuner_x500_3gains', text['bayes3'], GREEN)]

    for ax, (name, title, color) in zip(axes, runs):
        log = load(results, name)
        laps = log[log['action'].isin(['baseline', 'accept', 'reject', 'trial'])]
        flown = laps.dropna(subset=['rmse'])
        ax.scatter(flown['t'], flown['rmse'].clip(upper=top), s=110, color=GREY, zorder=2)

        # Reverted by the watchdog before the lap was over: no error value
        reverted = laps[laps['rmse'].isna()]
        ax.scatter(reverted['t'], [top] * len(reverted), s=260, marker='x', color=RED, linewidths=4, zorder=3)

        best = laps[laps['action'].isin(['baseline', 'accept'])]
        before, after, end = summary(log)
        ax.step(list(best['t']) + [end], list(best['rmse']) + [after], where='post', color=color, linewidth=4.5, zorder=4)
        ax.scatter([end], [after], s=260, color=color, edgecolors='black', linewidths=1.5, zorder=5)
        ax.annotate(text['after_s'].format(value=after, time=end), (end, after), textcoords='offset points',
                    xytext=(0, 26), ha='right' if end > 600 else 'left', va='bottom', fontsize=20, fontweight='bold',
                    bbox=dict(facecolor='white', edgecolor='none', alpha=0.85, pad=3), zorder=6)
        ax.annotate(f'{before:.3f} m', (0, before), textcoords='offset points', xytext=(16, 4), fontsize=20, va='center')
        ax.set_title(title)
        ax.set_xlabel(text['time'])

    axes[0].set_xlim(-40, 1180)
    axes[0].set_ylim(0.07, top + 0.015)
    axes[0].set_yticks([0.10, 0.15, 0.20, 0.25, 0.30])
    axes[0].set_ylabel(text['rmse'])
    handles = [Line2D([], [], color='black', linewidth=4.5, label=text['best']),
               Line2D([], [], marker='o', linestyle='', color=GREY, markersize=11, label=text['lap']),
               Line2D([], [], marker='x', linestyle='', color=RED, markersize=14, markeredgewidth=4, label=text['reverted'])]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    save(fig, out_dir, 'tuner_convergence')


def search_map(results, out_dir, text):
    fig, axes = plt.subplots(1, 2, figsize=SIZE, sharey=True)
    grid = pd.read_csv(os.path.join(results, f'sweep_{GAIN_X}_x_{GAIN_Y}.csv'))
    grid = grid[grid['valid'].astype(str) == 'True'].groupby([GAIN_X, GAIN_Y])['tracking_rmse_xy'].mean()
    xs = sorted({x for x, _ in grid.index})
    ys = sorted({y for _, y in grid.index})

    def edges(values):
        mid = [(a + b) / 2.0 for a, b in zip(values, values[1:])]
        return [values[0] - (mid[0] - values[0])] + mid + [values[-1] + (values[-1] - mid[-1])]

    ex, ey = edges(xs), edges(ys)
    values = np.array([[grid[(x, y)] for x in xs] for y in ys])
    stable = np.ma.masked_greater(values, UNSTABLE)
    # Light end of a green scale, so that markers and numbers stay readable on top
    shades = matplotlib.colors.LinearSegmentedColormap.from_list('error', plt.get_cmap('YlGn')(np.linspace(0.45, 0.0, 64)))

    for ax in axes:
        mesh = ax.pcolormesh(ex, ey, stable, cmap=shades, vmin=0.04, vmax=0.22, edgecolors='white', linewidth=2)

        for i, x in enumerate(xs):
            for j, y in enumerate(ys):
                bad = values[j, i] > UNSTABLE

                if bad:
                    ax.fill_between([ex[i], ex[i + 1]], ey[j], ey[j + 1], facecolor='#F4C7C3', edgecolor=RED, hatch='//',
                                    linewidth=0)

                ax.text(ex[i + 1] - 0.06, ey[j + 1] - 0.18, f'{values[j, i]:.2f}', ha='right', va='top', fontsize=15,
                        color='#8B0000' if bad else 'black', zorder=3,
                        bbox=dict(facecolor='white', edgecolor='none', alpha=0.75, pad=1.5))

        ax.set_xlim(ex[0], ex[-1])
        ax.set_ylim(ey[0], ey[-1])
        ax.set_xticks(xs)
        ax.set_xlabel(GAIN_X)
        ax.grid(False)

    axes[0].set_ylabel(GAIN_Y)
    axes[0].set_yticks(ys)
    star = dict(marker='*', s=750, edgecolors='black', linewidths=1.5, zorder=7)

    # Coordinate search: a path of accepted steps, rejected ones around its end
    ax = axes[0]
    log = load(results, 'online_tuner_x500')
    path = log[log['action'].isin(['baseline', 'accept'])]
    ax.plot(path[GAIN_X], path[GAIN_Y], color='black', linewidth=4, marker='o', markersize=13, zorder=4)
    rejected = log[log['action'] == 'reject']
    ax.scatter(rejected[GAIN_X], rejected[GAIN_Y], s=220, facecolors='white', edgecolors='black', linewidths=3, zorder=5)
    final = log[log['action'] == 'final'].iloc[0]
    ax.scatter([final[GAIN_X]], [final[GAIN_Y]], color=BLUE, **star)
    ax.scatter([path[GAIN_X].iloc[0]], [path[GAIN_Y].iloc[0]], s=330, marker='D', color=RED, edgecolors='black',
               linewidths=1.5, zorder=7)
    ax.set_title(text['map_coordinate'])

    # Bayes: trials spread over the allowed range
    ax = axes[1]

    for n, (name, color, marker) in enumerate([('bayes_tuner_x500', ORANGE, 'o'), ('bayes_tuner_x500_seed1', 'tab:purple', '^')]):
        log = load(results, name)
        tried = log[log['action'].isin(['accept', 'reject', 'trial'])]
        flown = tried.dropna(subset=['rmse'])
        ax.scatter(flown[GAIN_X], flown[GAIN_Y], s=150, marker=marker, color=color, edgecolors='black', linewidths=1.2,
                   zorder=4)
        reverted = tried[tried['rmse'].isna()]
        ax.scatter(reverted[GAIN_X], reverted[GAIN_Y], s=320, marker='X', color=RED, edgecolors='black', linewidths=1.2,
                   zorder=6)
        final = log[log['action'] == 'final'].iloc[0]
        ax.scatter([final[GAIN_X]], [final[GAIN_Y]], color=color, **star)

    ax.scatter([0.95], [1.8], s=330, marker='D', color=RED, edgecolors='black', linewidths=1.5, zorder=7)
    ax.set_title(text['map_bayes'])

    edge = dict(markeredgecolor='black', linestyle='')
    handles = [
        Line2D([], [], marker='D', color=RED, markersize=15, label=text['default'], **edge),
        Line2D([], [], marker='o', color='black', linewidth=4, markersize=12, label=text['accepted']),
        Line2D([], [], marker='o', color='white', markersize=13, markeredgewidth=3, label=text['rejected'], **edge),
        Line2D([], [], marker='o', color=ORANGE, markersize=13, label=text['seed'].format(n=0), **edge),
        Line2D([], [], marker='^', color='tab:purple', markersize=13, label=text['seed'].format(n=1), **edge),
        Line2D([], [], marker='X', color=RED, markersize=16, label=text['reverted'], **edge),
        Line2D([], [], marker='*', color=GREY, markersize=26, label=text['final'], **edge),
        Patch(facecolor='#F4C7C3', edgecolor=RED, hatch='//', label=text['unstable']),
    ]
    fig.legend(handles=handles, loc='center left', bbox_to_anchor=(0.715, 0.53), frameon=False, labelspacing=0.9)
    fig.subplots_adjust(left=0.065, right=0.6, bottom=0.17, top=0.9, wspace=0.06)
    bar = fig.colorbar(mesh, cax=fig.add_axes([0.612, 0.17, 0.013, 0.73]))
    bar.set_label(text['grid'], fontsize=17)
    bar.ax.tick_params(labelsize=17)
    save(fig, out_dir, 'tuner_search_map')


def before_after(results, out_dir, text):
    fig, ax = plt.subplots(figsize=SIZE)
    width = 0.42
    names = COORDINATE_RUNS + BAYES_RUNS
    labels = []

    for i, name in enumerate(names):
        before, after, duration = summary(load(results, name))
        color = BLUE if name in COORDINATE_RUNS else ORANGE
        ax.bar(i - width / 2, before, width, color=GREY, edgecolor='black', linewidth=1.2, zorder=3)
        ax.bar(i + width / 2, after, width, color=color, edgecolor='black', linewidth=1.2, zorder=3)
        ax.text(i - width / 2, before + 0.006, f'{before:.3f}', ha='center', va='bottom', fontsize=15)
        ax.text(i + width / 2, after + 0.006, f'{after:.3f}', ha='center', va='bottom', fontsize=15, fontweight='bold')
        ax.text(i + width / 2, after - 0.012, f'−{100.0 * (1.0 - after / before):.0f}%', ha='center', va='top',
                fontsize=13.5, fontweight='bold', color='white' if color == BLUE else 'black')
        labels.append(f'{text["runs"][name]}\n{duration:.0f} s')

    ax.axvline(len(COORDINATE_RUNS) - 0.5, color='black', linewidth=1.5, linestyle='--')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(labels, fontsize=17, linespacing=1.25)
    ax.set_xlim(-0.6, len(names) - 0.4)
    ax.set_ylim(0.0, 0.41)
    ax.set_yticks([0.0, 0.1, 0.2, 0.3])
    ax.set_ylabel(text['rmse'])
    ax.grid(False, axis='x')
    handles = [Patch(facecolor=GREY, edgecolor='black', label=text['before']),
               Patch(facecolor=BLUE, edgecolor='black', label=text['after_coordinate']),
               Patch(facecolor=ORANGE, edgecolor='black', label=text['after_bayes'])]
    ax.legend(handles=handles, loc='upper left', ncol=3, frameon=False)
    fig.tight_layout()
    save(fig, out_dir, 'tuner_before_after')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--results', default='results', help='directory with the tuner logs')
    parser.add_argument('--out-dir', default='results')
    parser.add_argument('--lang', choices=sorted(TEXT), default='en')
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 19, 'axes.titlesize': 22, 'axes.titleweight': 'bold',
        'axes.labelsize': 20, 'xtick.labelsize': 19, 'ytick.labelsize': 19, 'legend.fontsize': 18,
        'axes.grid': True, 'grid.color': '#CFCFCF', 'grid.linewidth': 1.2, 'axes.axisbelow': True,
        'axes.linewidth': 1.5, 'figure.facecolor': 'white', 'axes.facecolor': 'white', 'hatch.linewidth': 1.5,
    })
    text = TEXT[args.lang]
    convergence(args.results, args.out_dir, text)
    search_map(args.results, args.out_dir, text)
    before_after(args.results, args.out_dir, text)


if __name__ == '__main__':
    main()
