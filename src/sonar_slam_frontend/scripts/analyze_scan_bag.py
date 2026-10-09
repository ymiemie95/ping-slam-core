#!/usr/bin/env python3
"""
analyze_scan_bag.py —— 离线分析 sonarToLaserscan 输出的 /scan 质量

用法:
  python3 analyze_scan_bag.py <ros2_bag目录>            # 例: scan_check/
  python3 analyze_scan_bag.py scan_check --tf           # 同时用 /tf 把点投到 odom 系

输出:
  1) 终端统计: 帧数/帧率, validBins 分布, 段数分布, 帧间点重叠率
  2) scan_report.png 四联图:
     a. validBins 时间序列          b. 每帧段数直方图
     c. 极坐标叠加(传感器系)        d. odom 系全程叠加(需 --tf)

评估要点(对照长包离线复现值):
  validBins 中位 ~19, 丢帧后无 <12 点的帧;
  段数 1-2 为主; 帧间重叠率中位 >0.5 (顺序匹配的物质基础)。
"""
import sys, argparse, sqlite3, pathlib
import numpy as np

def read_ros2_bag(bagdir, topics):
    """直接读 rosbag2 的 sqlite3 (.db3) 文件, 不依赖 rosbag2_py 这个包
    (rosbag2_py 在部分发行版/安装方式下是独立 apt 包, 未必装了;
     但 bag 存储格式本身就是标准 sqlite3, 表结构固定, 直接读最稳)"""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bagdir = pathlib.Path(bagdir)
    db3_files = sorted(bagdir.glob('*.db3'))
    if not db3_files:
        sys.exit(f'{bagdir} 下没有找到 .db3 文件，确认这是 ros2 bag record 的输出目录')

    out = {t: [] for t in topics}
    msg_types = {}

    for db3 in db3_files:
        conn = sqlite3.connect(str(db3))
        cur = conn.cursor()
        cur.execute("SELECT id, name, type FROM topics")
        id2topic = {}
        for tid, name, ttype in cur.fetchall():
            if name in topics:
                id2topic[tid] = name
                if name not in msg_types:
                    msg_types[name] = get_message(ttype)

        if not id2topic:
            conn.close()
            continue

        placeholder = ','.join('?' * len(id2topic))
        cur.execute(
            f"SELECT topic_id, timestamp, data FROM messages "
            f"WHERE topic_id IN ({placeholder}) ORDER BY timestamp",
            list(id2topic.keys()))
        for topic_id, ts, data in cur.fetchall():
            name = id2topic[topic_id]
            msg = deserialize_message(bytes(data), msg_types[name])
            # ★用 header.stamp 而不是 bag 的录制时间戳★
            #   录制时间戳是墙钟时间; 回放慢于实时(节点负载重)时它会被拉长,
            #   导致 scan 与 tf 的时间轴对不上, searchsorted 全部落到端点,
            #   所有帧被画在同一个位姿上 —— 表现为叠加图塌缩成一条弧。
            #   /tf 是 TFMessage, header 在每个 transform 里, 交给调用方处理。
            hdr_ts = None
            if hasattr(msg, 'header'):
                hdr_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            elif hasattr(msg, 'transforms') and len(msg.transforms) > 0:
                st0 = msg.transforms[0].header.stamp
                hdr_ts = st0.sec + st0.nanosec * 1e-9
            if hdr_ts is None or hdr_ts <= 0.0:
                hdr_ts = ts / 1e9          # 退化情况: 回落到录制时间戳
            out[name].append((hdr_ts, msg))
        conn.close()

    for t in topics:
        out[t].sort(key=lambda x: x[0])
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--tf', action='store_true', help='用 /tf 投到 odom 系')
    ap.add_argument('--min-valid', type=int, default=12)
    args = ap.parse_args()

    topics = ['/scan'] + (['/tf'] if args.tf else [])
    data = read_ros2_bag(args.bag, topics)
    scans = data['/scan']
    if not scans:
        sys.exit('bag 里没有 /scan')

    print(f'共 {len(scans)} 帧 /scan')
    scans.sort(key=lambda x: x[0])
    t0 = scans[0][0]
    dur = scans[-1][0] - t0
    print(f'时长 {dur:.0f}s, 平均帧率 {len(scans)/max(dur,1e-9):.3f} Hz '
          f'(期望 ~0.18 Hz)')
    # 时间轴一致性: 帧间隔应稳定在 ~5.6s (π/2 步进); 出现远大于此的间隔说明
    # 回放中断或时间轴混入了墙钟时间
    gaps = np.diff([s[0] for s in scans])
    if len(gaps):
        print(f'  帧间隔: 中位 {np.median(gaps):.2f}s  最大 {gaps.max():.1f}s'
              f'  >30s 的间隔 {int(np.sum(gaps > 30))} 处')

    # ---- 每帧统计 ----
    times, vbins, nsegs, rrs = [], [], [], []
    for ts, m in scans:
        r = np.array(m.ranges, dtype=float)
        fin = np.isfinite(r) & (r > 0)
        times.append(ts - t0)
        vbins.append(int(fin.sum()))
        # 复现节点的分段逻辑数段数(gap<=2, jump<=1.2*gap)
        idx = np.where(fin)[0]
        n = len(r)
        if len(idx) == 0:
            nsegs.append(0)
        else:
            gaps = [(idx[(k+1) % len(idx)] - idx[k]) % n for k in range(len(idx))]
            cut = int(np.argmax(gaps))
            order = [idx[(cut + 1 + k) % len(idx)] for k in range(len(idx))]
            s = 1
            for a, b in zip(order[:-1], order[1:]):
                g = (b - a) % n
                if not (g <= 2 and abs(r[b] - r[a]) <= 1.2 * g):
                    s += 1
            nsegs.append(s)
        rrs.append(r)

    vbins = np.array(vbins); times = np.array(times)
    print(f'\nvalidBins: 中位 {np.median(vbins):.0f}  P10 {np.percentile(vbins,10):.0f}  '
          f'P90 {np.percentile(vbins,90):.0f}  最小 {vbins.min()}')
    low = np.mean(vbins < args.min_valid)
    print(f'  < {args.min_valid} 点的帧: {100*low:.0f}%  '
          f'(闸门已生效时应为 0%; >0 说明旧版本仍在跑)')
    seg_arr = np.array(nsegs)
    print('段数分布: ' + '  '.join(f'{k}段:{100*np.mean(seg_arr==k):.0f}%'
                                   for k in range(0, 5)))

    # ---- 帧间重叠率: 相邻帧共同有效 bin 上距离差 <0.5m 的比例 ----
    ov = []
    for k in range(1, len(rrs)):
        a, b = rrs[k-1], rrs[k]
        both = np.isfinite(a) & np.isfinite(b)
        if both.sum() < 5:
            ov.append(0.0); continue
        ov.append(float(np.mean(np.abs(a[both] - b[both]) < 0.5)))
    ov = np.array(ov)
    print(f'帧间重叠率(共同bin距离差<0.5m): 中位 {np.median(ov):.2f}  '
          f'P10 {np.percentile(ov,10):.2f}   —— 顺序匹配可行性的直接指标')

    # ---- TF (可选) ----
    poses = None
    if args.tf and data.get('/tf'):
        tfs = []
        for _ts, m in data['/tf']:
            for tr in m.transforms:
                if tr.child_frame_id == 'base_link':
                    q = tr.transform.rotation
                    yaw = np.arctan2(2*(q.w*q.z + q.x*q.y),
                                     1 - 2*(q.y*q.y + q.z*q.z))
                    tts = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                    tfs.append((tts, tr.transform.translation.x,
                                tr.transform.translation.y, yaw))
        if tfs:
            tfs = np.array(tfs)
            poses = tfs[np.argsort(tfs[:, 0])]
            print(f'/tf odom→base_link: {len(poses)} 条, '
                  f'覆盖 {poses[-1,0]-poses[0,0]:.0f}s')
            # ★关键检查★ 落在 tf 时间范围之外的 scan 会被钳到端点,
            # 表现为叠加图上所有帧塌缩到同一个位姿
            outside = np.sum([(s[0] < poses[0, 0]) or (s[0] > poses[-1, 0])
                              for s in scans])
            if outside:
                print(f'  ⚠ 有 {outside}/{len(scans)} 帧的时间戳落在 /tf 范围之外 '
                      f'→ 这些帧的位姿不可信, odom 叠加图会失真')
            else:
                print('  ✓ 全部 scan 时间戳都在 /tf 覆盖范围内')

    # ---- 画图 ----
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(16, 11))

    ax = fig.add_subplot(2, 2, 1)
    ax.plot(times, vbins, 'b.-', ms=3, lw=0.6)
    ax.axhline(args.min_valid, color='r', ls='--', label=f'gate={args.min_valid}')
    ax.set_xlabel('t (s)'); ax.set_ylabel('validBins')
    ax.legend(); ax.grid(alpha=0.3); ax.set_title('validBins per frame')

    ax = fig.add_subplot(2, 2, 2)
    ax.hist(seg_arr, bins=np.arange(-0.5, 6.5), rwidth=0.8, color='teal')
    ax.set_xlabel('segments per frame'); ax.set_ylabel('#frames')
    ax.grid(alpha=0.3); ax.set_title('segment count (expect mostly 1-2)')

    ax = fig.add_subplot(2, 2, 3)
    for r in rrs:
        fin = np.where(np.isfinite(r) & (np.array(r) > 0))[0]
        th = fin * (2*np.pi/len(r)) - np.pi
        rr = np.array(r)[fin]
        ax.scatter(rr*np.cos(th), rr*np.sin(th), s=1, alpha=0.15, c='crimson')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_title('sensor-frame overlay (structure = walls, fog = residual speckle)')

    ax = fig.add_subplot(2, 2, 4)
    if poses is not None:
        for (ts, m), r in zip(scans, rrs):
            i = np.searchsorted(poses[:, 0], ts)
            i = np.clip(i, 0, len(poses)-1)
            px, py, yaw = poses[i, 1], poses[i, 2], poses[i, 3]
            fin = np.where(np.isfinite(r) & (np.array(r) > 0))[0]
            th = fin * (2*np.pi/len(r)) - np.pi + yaw
            rr = np.array(r)[fin]
            ax.scatter(px + rr*np.cos(th), py + rr*np.sin(th),
                       s=1, alpha=0.2, c='navy')
        ax.plot(poses[:, 1], poses[:, 2], 'g-', lw=0.8)
        ax.set_title('odom-frame overlay (should show pond outline)')
    else:
        ax.text(0.5, 0.5, 'run with --tf to enable', ha='center')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig('scan_report.png', dpi=110)
    print('\n已生成 scan_report.png')

if __name__ == '__main__':
    main()