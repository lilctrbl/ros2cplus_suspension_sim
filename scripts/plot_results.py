#!/usr/bin/env python3
# Copyright 2026
# Apache-2.0
"""
Plot suspension simulation results from the CSV recorded by data_recorder_node.

CSV 由 data_recorder_node 生成，表头：
    t, road_height, xs, xus, xs_dot, xus_dot, body_accel,
    control_force, xs_est, xus_est

用法：
    python3 plot_results.py [csv_file]             # 默认 logs/suspension_log.csv
    python3 plot_results.py logs/x.csv --out a.png # 存为图片（不弹窗）
    python3 plot_results.py logs/x.csv --xlim 0 5  # 只画前 5 秒
"""

import argparse
import os
import sys

import matplotlib
from matplotlib import font_manager
import numpy as np

# 常见中文字体候选（按优先级）——找不到可用中文字体时回退英文标签
_ZH_CANDIDATES = [
    'Noto Sans CJK SC', 'Noto Serif CJK SC', 'WenQuanYi Zen Hei',
    'WenQuanYi Micro Hei', 'AR PL UMing CN', 'SimHei', 'Microsoft YaHei',
]


def setup_chinese_font():
    """
    Configure a CJK-capable font when available.

    成功返回 True，否则返回 False（脚本改用英文标签）。
    """
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in _ZH_CANDIDATES:
        if name in available:
            matplotlib.rcParams['font.sans-serif'] = [name] + \
                matplotlib.rcParams['font.sans-serif']
            matplotlib.rcParams['axes.unicode_minus'] = False
            return True
    return False


HAS_ZH = setup_chinese_font()


def parse_args():
    """
    Build the command line argument parser.

    返回已解析的命令行参数。
    """
    parser = argparse.ArgumentParser(
        description='绘制悬架仿真 CSV 结果（data_recorder_node 记录）')
    parser.add_argument(
        'csv', nargs='?', default='logs/suspension_log.csv',
        help='CSV 文件路径（默认 logs/suspension_log.csv）')
    parser.add_argument(
        '--out', default=None,
        help='输出图片路径（如 figure.png），不指定则弹窗显示')
    parser.add_argument(
        '--no-show', action='store_true',
        help='不调用 plt.show()（无显示环境时配合 --out 使用）')
    parser.add_argument(
        '--xlim', nargs=2, type=float, default=None, metavar=('T0', 'T1'),
        help='只绘制 t 在 [T0, T1] 内的数据（默认全部）')
    return parser.parse_args()


def load_csv(path):
    """
    Read the CSV into a dict of named columns.

    读取 CSV，返回 {列名: numpy 数组}（缺失话题对应位置为 NaN）。
    """
    data = np.genfromtxt(path, delimiter=',', names=True, dtype=None,
                         encoding='utf-8')
    names = data.dtype.names
    if names is None:
        sys.exit(f'无法解析 CSV：{path}')
    return {n: np.asarray(data[n], dtype=float) for n in names}


def panels():
    """
    Return the list of plotting panels.

    返回绘图面板列表：(标题[中文,英文], [(列名, 图例), ...])。
    """
    return [
        (('位移 (m)', 'displacement (m)'),
         [('xs', 'xs（簧上）'), ('xus', 'xus（簧下）')]),
        (('速度 (m/s)', 'velocity (m/s)'),
         [('xs_dot', 'xs_dot'), ('xus_dot', 'xus_dot')]),
        (('车身加速度 (m/s^2)', 'body accel (m/s^2)'),
         [('body_accel', 'body_accel')]),
        (('控制力 (N)', 'control force (N)'),
         [('control_force', 'control_force')]),
        (('路面高度 (m)', 'road height (m)'),
         [('road_height', 'road_height')]),
    ]


def main():
    """
    Entry point: parse args, plot the CSV panels, then show or save.

    解析参数 → 读取 CSV → 绘制各面板 → 弹窗显示或保存图片。
    """
    args = parse_args()

    # 指定 --out 时使用 Agg 后端（无需显示）；否则用默认交互后端
    if args.out:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt  # noqa: E402
    # pyplot 仅在绘制时导入（--out 时后端已切为 Agg）

    if not os.path.exists(args.csv):
        sys.exit(
            f'找不到 CSV：{args.csv}\n'
            '请先运行 data_recorder_node 生成日志（见 RUNNING.md 第 3.10 节）')

    cols = load_csv(args.csv)
    t = cols['t']

    # 时间区间裁剪
    if args.xlim is not None:
        mask = (t >= args.xlim[0]) & (t <= args.xlim[1])
        t = t[mask]
        cols = {n: v[mask] for n, v in cols.items()}
    if t.size < 2:
        sys.exit(f'CSV 有效数据过少（{t.size} 行），无法绘图')

    # 估计列存在时才追加“估计 vs 真值”面板
    p = panels()
    if any(k in cols for k in ('xs_est', 'xus_est')):
        p.append(
            (('估计 vs 真值 (m)', 'estimate vs truth (m)'),
             [('xs', 'xs（真值）'), ('xus', 'xus（真值）'),
              ('xs_est', 'xs_est（估计）'), ('xus_est', 'xus_est（估计）')]))

    n_rows = len(p)
    fig, axes = plt.subplots(n_rows, 1, figsize=(11, 2.4 * n_rows),
                             sharex=True, squeeze=False)
    axes = axes[:, 0]

    for ax, ((zh_title, en_title), curves) in zip(axes, p):
        for col, label in curves:
            if col not in cols:
                continue
            y = cols[col]
            ok = ~np.isnan(y)
            if not ok.any():
                continue
            dashed = col.endswith('_est')
            ax.plot(t[ok], y[ok], label=label, lw=1.1,
                    ls=('--' if dashed else '-'))
        ax.grid(True, alpha=0.3)
        ax.set_ylabel(zh_title if HAS_ZH else en_title)
        ax.legend(loc='upper right', fontsize=8, ncol=len(curves))

    axes[-1].set_xlabel('时间 t (s)' if HAS_ZH else 'time t (s)')

    fig.suptitle(
        os.path.basename(args.csv) + ('（中文字体未找到）' if not HAS_ZH else ''),
        fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.98))

    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f'已保存图片：{args.out}')
        return
    if args.no_show:
        print('已生成图表（--no-show，未弹窗）')
        return
    plt.show()


if __name__ == '__main__':
    main()
