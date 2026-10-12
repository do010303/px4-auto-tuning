#!/usr/bin/env python3
"""Plot a one-parameter sweep: one small panel per metric, shared gain axis.

    python3 -m px4_auto_tuning.plots results/sweep_MPC_XY_P.csv --baseline results/baseline_figure8.csv

Dots are single measurements, the line joins the per-value means. The grey
band is the baseline mean +/- 2 standard deviations: differences inside it
are within measurement noise. Crosses on the bottom edge mark failed runs.
"""

import argparse
import os

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

SERIES = '#2a78d6'
CRITICAL = '#d03b3b'
SURFACE = '#fcfcfb'
TEXT = '#0b0b0b'
MUTED = '#52514e'
GRID = '#e4e3df'

PANELS = [
    ('true_rmse_xy', 'True error, horizontal RMSE [m]'),
    ('tracking_rmse_xy', 'Tracking error, horizontal RMSE [m]'),
    ('true_crosstrack_rmse', 'Cross-track error RMSE [m]'),
    ('rate_hf_rms', 'Oscillation, high-frequency body rate RMS [rad/s]'),
]


def plot_sweep(table, baseline=None, out=None):
    data = pd.read_csv(table)
    data = data[data['valid'].astype(str) == 'True']
    failed = data[data['failed'].astype(str) == 'True']
    ok = data[data['failed'].astype(str) != 'True']
    param = data['param'].iloc[0]
    default = data['default'].iloc[0]
    base = pd.read_csv(baseline) if baseline else None

    if base is not None:
        base = base[base['valid'].astype(str) == 'True']

    fig, axes = plt.subplots(2, 2, figsize=(10, 6.5), sharex=True, facecolor=SURFACE)

    for ax, (column, title) in zip(axes.flat, PANELS):
        ax.set_facecolor(SURFACE)

        if base is not None and column in base:
            mean, std = base[column].mean(), base[column].std()
            ax.axhspan(mean - 2 * std, mean + 2 * std, color=GRID, zorder=0, linewidth=0)

        ax.axvline(default, color=MUTED, linewidth=1, linestyle=(0, (2, 3)), zorder=1)
        ax.scatter(ok['value'], ok[column], s=22, color=SERIES, alpha=0.45, linewidths=0, zorder=2)
        means = ok.groupby('value')[column].mean()
        ax.plot(means.index, means.values, color=SERIES, linewidth=2, zorder=3)

        ax.set_ylim(bottom=0)

        if len(failed):
            ax.scatter(failed['value'], [0] * len(failed), marker='x', s=45, color=CRITICAL,
                       clip_on=False, zorder=4)

        ax.set_title(title, loc='left', fontsize=10, color=TEXT)
        ax.grid(axis='y', color=GRID, linewidth=0.8)
        ax.tick_params(colors=MUTED, labelsize=9, length=0)

        for side in ('top', 'right', 'left'):
            ax.spines[side].set_visible(False)

        ax.spines['bottom'].set_color(GRID)

    for ax in axes[1]:
        ax.set_xlabel(param, color=MUTED, fontsize=9)

    note = f'dotted line: default {default:g}'

    if base is not None:
        note += '   ·   grey band: baseline mean ± 2 SD'

    if len(failed):
        note += '   ·   red cross: flight diverged'

    fig.suptitle(f'{param} sweep', x=0.06, ha='left', fontsize=13, color=TEXT)
    fig.text(0.06, 0.925, note, fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0.03, 0, 1, 0.92))

    out = out or os.path.splitext(table)[0] + '.png'
    fig.savefig(out, dpi=140, facecolor=SURFACE)
    plt.close(fig)
    return out


def plot_grid(table, out=None, column='true_rmse_xy', osc_limit=0.15):
    """Heat map of a two-parameter sweep. Cells whose oscillation exceeds `osc_limit` are outlined."""
    from matplotlib.colors import LinearSegmentedColormap

    data = pd.read_csv(table)
    data = data[(data['valid'].astype(str) == 'True') & (data['failed'].astype(str) != 'True')]
    row_name, col_name = data['param'].iloc[0].split(',')
    grid = data.pivot_table(index=row_name, columns=col_name, values=column, aggfunc='mean')
    osc = data.pivot_table(index=row_name, columns=col_name, values='rate_hf_rms', aggfunc='mean')

    # one hue, light to dark; everything above 0.5 m is "bad" and shares the darkest step
    cmap = LinearSegmentedColormap.from_list('blue', ['#cde2fb', '#6da7ec', '#256abf', '#0d366b'])
    fig, ax = plt.subplots(figsize=(1.6 + 1.3 * grid.shape[1], 1.6 + 0.8 * grid.shape[0]), facecolor=SURFACE)
    ax.imshow(grid.values, cmap=cmap, vmin=0.05, vmax=0.5, aspect='auto', origin='lower')

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            value = grid.values[i, j]
            ax.text(j, i, f'{value:.2f}', ha='center', va='center', fontsize=10,
                    color='#ffffff' if value > 0.25 else TEXT)

            if osc.values[i, j] > osc_limit:
                ax.add_patch(plt.Rectangle((j - 0.46, i - 0.46), 0.92, 0.92, fill=False,
                                           edgecolor=CRITICAL, linewidth=2))

    # 2 px surface gap between cells
    ax.set_xticks([x - 0.5 for x in range(1, grid.shape[1])], minor=True)
    ax.set_yticks([y - 0.5 for y in range(1, grid.shape[0])], minor=True)
    ax.grid(which='minor', color=SURFACE, linewidth=2)
    ax.tick_params(which='both', length=0, colors=MUTED, labelsize=9)
    ax.set_xticks(range(grid.shape[1]), [f'{v:g}' for v in grid.columns])
    ax.set_yticks(range(grid.shape[0]), [f'{v:g}' for v in grid.index])
    ax.set_xlabel(col_name, color=MUTED, fontsize=9)
    ax.set_ylabel(row_name, color=MUTED, fontsize=9)

    for side in ax.spines.values():
        side.set_visible(False)

    fig.suptitle('True error, horizontal RMSE [m]', x=0.02, ha='left', fontsize=12, color=TEXT)
    fig.text(0.02, 0.9, f'red outline: oscillation above {osc_limit:g} rad/s', fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    out = out or os.path.splitext(table)[0] + '.png'
    fig.savefig(out, dpi=140, facecolor=SURFACE)
    plt.close(fig)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('table')
    parser.add_argument('--baseline')
    parser.add_argument('--out')
    args = parser.parse_args()
    if ',' in str(pd.read_csv(args.table)['param'].iloc[0]):
        print(plot_grid(args.table, args.out))

    else:
        print(plot_sweep(args.table, args.baseline, args.out))


if __name__ == '__main__':
    main()
