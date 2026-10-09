#!/usr/bin/env python3
"""
render_map.py —— 把 map_saver 导出的 .pgm/.yaml 渲染成便于检查的地图图片

用法:
  python3 render_map.py tilepool_final                    # 读 tilepool_final.{pgm,yaml}
  python3 render_map.py tilepool_final --traj slam.tum    # 叠加 TUM 轨迹
  python3 render_map.py tilepool_final --style dark       # 深色风格
  python3 render_map.py tilepool_final --crop 2           # 裁掉四周 2m 空白外的区域

输出: <name>_render.png (300 dpi) 与 <name>_render.pdf (矢量预览)

配色方案 (--style):
  clean (默认): 白色自由区 / 深藏青墙 / 浅灰未知 —— 印刷友好
  dark        : 深底 / 白墙 / 青色轨迹 —— 演示PPT用
"""
import argparse, pathlib, sys
import numpy as np


def read_pgm(path):
    data = open(path, 'rb').read()
    # 头部: P5 <w> <h> <maxval>\n, 可能夹注释行
    tokens, i = [], 0
    while len(tokens) < 4:
        j = data.find(b'\n', i)
        line = data[i:j]
        i = j + 1
        if line.startswith(b'#'):
            continue
        tokens += line.split()
    magic, w, h, maxv = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b'P5':
        img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    elif magic == b'P2':
        img = np.array(data[i:].split()[:w * h], dtype=np.uint8).reshape(h, w)
    else:
        sys.exit(f'不支持的PGM格式: {magic}')
    return img


def read_yaml_meta(path):
    res, origin = 0.05, (0.0, 0.0)
    for line in open(path):
        line = line.strip()
        if line.startswith('resolution:'):
            res = float(line.split(':', 1)[1])
        elif line.startswith('origin:'):
            v = line.split(':', 1)[1].strip().strip('[]').split(',')
            origin = (float(v[0]), float(v[1]))
    return res, origin


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', help='地图名(不含扩展名), 读取 name.pgm + name.yaml')
    ap.add_argument('--traj', help='TUM 轨迹文件(t x y z qx qy qz qw), map 系')
    ap.add_argument('--style', default='clean', choices=['clean', 'dark'])
    ap.add_argument('--crop', type=float, default=1.0,
                    help='裁剪: 保留占据区域外多少米的边距 (默认1.0, 0=不裁)')
    args = ap.parse_args()

    img = read_pgm(args.name + '.pgm')
    res, origin = read_yaml_meta(args.name + '.yaml')
    h, w = img.shape

    occ = img < 100                    # 占据
    free = img > 250                   # 自由
    unk = ~occ & ~free                 # 未知

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    if args.style == 'clean':
        c_free, c_unk, c_occ = (1, 1, 1), (0.92, 0.92, 0.92), (0.10, 0.12, 0.25)
        c_traj, c_bar = '#c62828', '#333333'
    else:
        c_free, c_unk, c_occ = (0.13, 0.15, 0.18), (0.20, 0.22, 0.25), (0.95, 0.95, 0.95)
        c_traj, c_bar = '#00bcd4', '#eeeeee'

    rgb = np.zeros((h, w, 3))
    rgb[unk] = c_unk
    rgb[free] = c_free
    rgb[occ] = c_occ
    rgb = rgb[::-1]                    # pgm 第一行是地图顶端, 翻转成 y 向上

    # 物理坐标范围 (map 系)
    x0, y0 = origin
    extent = [x0, x0 + w * res, y0, y0 + h * res]

    fig, ax = plt.subplots(figsize=(10, 10 * h / w))
    ax.imshow(rgb, extent=extent, origin='lower', interpolation='nearest')

    if args.traj:
        T = np.loadtxt(args.traj)
        if T.ndim == 1:
            T = T.reshape(1, -1)
        ax.plot(T[:, 1], T[:, 2], color=c_traj, lw=1.2, alpha=0.95,
                label='trajectory')
        ax.plot(T[0, 1], T[0, 2], 'o', color=c_traj, ms=6)
        ax.plot(T[-1, 1], T[-1, 2], 's', color=c_traj, ms=6)
        ax.legend(loc='upper right', framealpha=0.8)

    # 按占据区域裁剪
    if args.crop > 0 and occ.any():
        ys, xs = np.where(occ[::-1])
        m = args.crop
        ax.set_xlim(x0 + xs.min() * res - m, x0 + xs.max() * res + m)
        ax.set_ylim(y0 + ys.min() * res - m, y0 + ys.max() * res + m)

    # 比例尺 (1 m)
    xl, yl = ax.get_xlim(), ax.get_ylim()
    bx = xl[0] + 0.06 * (xl[1] - xl[0])
    by = yl[0] + 0.06 * (yl[1] - yl[0])
    ax.add_patch(Rectangle((bx, by), 1.0, 0.03 * (yl[1] - yl[0]),
                           color=c_bar, ec='none'))
    ax.text(bx + 0.5, by + 0.05 * (yl[1] - yl[0]), '1 m',
            ha='center', color=c_bar, fontsize=11)

    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    plt.tight_layout(pad=0.3)
    for ext in ('png', 'pdf'):
        out = f'{args.name}_render.{ext}'
        plt.savefig(out, dpi=300, bbox_inches='tight',
                    facecolor=rgb[0, 0] if args.style == 'dark' else 'white')
        print('已生成', out)


if __name__ == '__main__':
    main()
