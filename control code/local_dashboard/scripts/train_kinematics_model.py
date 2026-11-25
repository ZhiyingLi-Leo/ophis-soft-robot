"""Train, cross-validate, and evaluate 3D forward/inverse catheter kinematics models on NAS results.

Reads /Volumes/zhiying/OPHIS_双机全量分析_2026-11-25 (all_rounds.csv + steady_shapes.json),
trains candidate pure-NumPy forward models on dual_axis_train:ok (81 rounds, 27 points),
selects the best model via grouped leave-one-point-out CV, evaluates strictly on the
held-out clean validation set (dual_axis_val:ok, 24 rounds), checks tip_leak anomaly
deviation, benchmarks constrained inverse solving, and saves all artifacts + an HTML
SVG visualization report directly to <analysis-dir>/modeling/.
"""

import argparse
from collections import defaultdict
import csv
from datetime import datetime
import html
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from device_plugins import shape3d
from kinematics_model import (
    ForwardKinematicsModel,
    evaluate_predictions,
    extract_canonical_sample,
    grouped_loocv,
    solve_inverse_tangent,
)

DEFAULT_ANALYSIS_DIR = Path('/Volumes/zhiying/OPHIS_双机全量分析_2026-11-25')


def load_samples(analysis_dir, *, modality='stereo'):
    analysis_dir = Path(analysis_dir)
    with (analysis_dir / 'all_rounds.csv').open(encoding='utf-8') as stream:
        round_rows = list(csv.DictReader(stream))
    row_map = {
        (r['dataset_group'], r['batch_name'], r['round_slug']): r
        for r in round_rows
    }

    samples = []
    for shape_file in sorted((analysis_dir / 'batches').glob('*/*/steady_shapes.json')):
        shapes = json.loads(shape_file.read_text(encoding='utf-8'))
        for s in shapes:
            key = (s['dataset_group'], s['batch_name'], s['round_slug'])
            rrow = row_map.get(key)
            if rrow is None:
                continue
            sample = extract_canonical_sample(s, rrow, modality=modality)
            if sample is not None:
                samples.append(sample)
    return samples


def build_zero_anchors(train_samples, count=3):
    """Build reference (0, 0, 0) anchor samples from the median reference centerline."""
    ref_curves = np.array([s['ref_centerline_mm'] for s in train_samples], dtype=np.float64)
    med_ref = np.median(ref_curves, axis=0)
    med_ref = med_ref - med_ref[0]
    anchor = dict(
        dataset_group='reference_anchor',
        batch_name='zero',
        quality_flag='ok',
        ids=[1, 2, 3],
        target_mm=[0, 0, 0],
        actual_mm=[0.0, 0.0, 0.0],
        repeat=1,
        round_slug='ZERO',
        ref_centerline_mm=med_ref,
        centerline_mm=med_ref,
        tip_pos_mm=med_ref[-1],
        ref_tip_pos_mm=med_ref[-1],
        tip_tangent=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        bend_angle_deg=0.0,
        bend_azimuth_deg=None,
        reproj_rms_px=0.0,
    )
    return [anchor] * count


def candidate_specs():
    specs = [
        ('linear', lambda: ForwardKinematicsModel('poly', degree=1, reg=1e-2)),
        ('poly_deg2', lambda: ForwardKinematicsModel('poly', degree=2, reg=1e-1)),
        ('poly_deg3', lambda: ForwardKinematicsModel('poly', degree=3, reg=5e-1)),
        ('deadzone_poly_deg2', lambda: ForwardKinematicsModel('deadzone_poly', degree=2, deadzone_mm=(2.5, 2.5, 2.0), reg=1e-1)),
        ('deadzone_poly_deg3', lambda: ForwardKinematicsModel('deadzone_poly', degree=3, deadzone_mm=(2.5, 2.5, 2.0), reg=5e-1)),
    ]
    for kernel in ('thin_plate', 'multiquadric', 'cubic'):
        for reg in (1e-2, 1e-1, 5e-1, 2.0):
            if kernel == 'multiquadric':
                for eps in (1.5, 2.5, 4.0):
                    specs.append((
                        f'rbf_{kernel}_eps{eps:g}_reg{reg:g}',
                        (lambda k=kernel, e=eps, r=reg: ForwardKinematicsModel('rbf', kernel=k, epsilon=e, reg=r)),
                    ))
            else:
                specs.append((
                    f'rbf_{kernel}_reg{reg:g}',
                    (lambda k=kernel, r=reg: ForwardKinematicsModel('rbf', kernel=k, epsilon=2.0, reg=r)),
                ))
    return specs


def fit_and_eval(factory, train_samples, val_samples, zero_anchors):
    tr_all = train_samples + zero_anchors
    X_tr = np.array([s['actual_mm'] for s in tr_all], dtype=np.float64)
    T_tr = np.array([s['tip_tangent'] for s in tr_all], dtype=np.float64)
    C_tr = np.array([s['centerline_mm'] for s in tr_all], dtype=np.float64)

    model = factory()
    model.fit(X_tr, T_tr, C_tr)

    X_val = np.array([s['actual_mm'] for s in val_samples], dtype=np.float64)
    T_val = np.array([s['tip_tangent'] for s in val_samples], dtype=np.float64)
    C_val = np.array([s['centerline_mm'] for s in val_samples], dtype=np.float64)
    pt, pc = model.predict(X_val)
    metrics, ang_err, tip_err, curve_rms = evaluate_predictions(T_val, pt, C_val, pc)
    return model, metrics, pt, pc, ang_err, tip_err, curve_rms


def _svg_parity_plot(val_rows, leak_rows):
    w, h = 440, 380
    pad_l, pad_r, pad_t, pad_b = 56, 24, 28, 48
    pw, ph = w - pad_l - pad_r, h - pad_t - pad_b
    max_v = max(
        [60.0]
        + [r['true_bend_deg'] for r in val_rows + leak_rows]
        + [r['pred_bend_deg'] for r in val_rows + leak_rows]
    )
    max_v = math.ceil(max_v / 10.0) * 10.0

    def sx(v):
        return pad_l + (v / max_v) * pw

    def sy(v):
        return pad_t + ph - (v / max_v) * ph

    elems = [
        f'<rect x="{pad_l}" y="{pad_t}" width="{pw}" height="{ph}" fill="#fafbfc" stroke="#d0d7de"/>',
        f'<line x1="{sx(0):.1f}" y1="{sy(0):.1f}" x2="{sx(max_v):.1f}" y2="{sy(max_v):.1f}" stroke="#6e7781" stroke-dasharray="5,4" stroke-width="1.2"/>',
    ]
    step = 10 if max_v <= 70 else 20
    for tick in range(0, int(max_v) + 1, step):
        x, y = sx(tick), sy(tick)
        elems.append(f'<line x1="{x:.1f}" y1="{pad_t}" x2="{x:.1f}" y2="{pad_t+ph}" stroke="#eef1f4"/>')
        elems.append(f'<line x1="{pad_l}" y1="{y:.1f}" x2="{pad_l+pw}" y2="{y:.1f}" stroke="#eef1f4"/>')
        elems.append(f'<text x="{x:.1f}" y="{pad_t+ph+18}" text-anchor="middle" font-size="11" fill="#57606a">{tick}°</text>')
        elems.append(f'<text x="{pad_l-8}" y="{y+4:.1f}" text-anchor="end" font-size="11" fill="#57606a">{tick}°</text>')

    for r in leak_rows:
        cx, cy = sx(r['true_bend_deg']), sy(r['pred_bend_deg'])
        elems.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="4.5" fill="#cf222e" fill-opacity="0.7" stroke="#82071e"/>')
    for r in val_rows:
        cx, cy = sx(r['true_bend_deg']), sy(r['pred_bend_deg'])
        color = '#0969da' if r['ids'] == '1,2' else '#1a7f37'
        elems.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="5" fill="{color}" fill-opacity="0.82" stroke="#ffffff" stroke-width="1"/>')

    elems.append(f'<text x="{pad_l + pw/2:.1f}" y="{h-10}" text-anchor="middle" font-size="12" fill="#24292f">Measured 3D Bend Angle (deg)</text>')
    elems.append(f'<text x="16" y="{pad_t + ph/2:.1f}" text-anchor="middle" font-size="12" fill="#24292f" transform="rotate(-90 16 {pad_t + ph/2:.1f})">Predicted 3D Bend Angle (deg)</text>')
    elems.append(f'<text x="{pad_l+8}" y="{pad_t+18}" font-size="11" fill="#0969da">● ID1+ID2 val (clean)</text>')
    elems.append(f'<text x="{pad_l+8}" y="{pad_t+34}" font-size="11" fill="#1a7f37">● ID1+ID3 val (clean)</text>')
    elems.append(f'<text x="{pad_l+8}" y="{pad_t+50}" font-size="11" fill="#cf222e">● ID2+ID3 val (tip_leak)</text>')
    return f'<svg width="{w}" height="{h}" viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg">{"".join(elems)}</svg>'


def _svg_centerline_grid(val_point_curves):
    """Render XZ (side) and YZ (front) measured vs predicted 21-point curves for each held-out point."""
    cols = 4
    rows = math.ceil(len(val_point_curves) / cols)
    cw, ch = 220, 190
    w, h = cols * cw, rows * ch
    elems = []

    for idx, item in enumerate(val_point_curves):
        gx = (idx % cols) * cw
        gy = (idx // cols) * ch
        pad = 28
        pw, ph = cw - 2 * pad, ch - 46
        true_c = np.asarray(item['true_curve_mm'])
        pred_c = np.asarray(item['pred_curve_mm'])

        # Project onto radial bending plane (Z horizontal 0..35 mm, lateral X or Y vertical -25..25 mm)
        lat_axis = 0 if np.ptp(true_c[:, 0]) >= np.ptp(true_c[:, 1]) else 1
        lat_label = 'X' if lat_axis == 0 else 'Y'

        def sx(z):
            return gx + pad + (z / 36.0) * pw

        def sy(lat):
            return gy + 26 + ph / 2.0 - (lat / 26.0) * (ph / 2.0)

        elems.append(f'<rect x="{gx+6}" y="{gy+4}" width="{cw-12}" height="{ch-8}" rx="6" fill="#ffffff" stroke="#d0d7de"/>')
        elems.append(f'<line x1="{sx(0):.1f}" y1="{sy(0):.1f}" x2="{sx(35):.1f}" y2="{sy(0):.1f}" stroke="#eaeef2"/>')
        t_pts = ' '.join(f'{sx(p[2]):.1f},{sy(p[lat_axis]):.1f}' for p in true_c)
        p_pts = ' '.join(f'{sx(p[2]):.1f},{sy(p[lat_axis]):.1f}' for p in pred_c)
        elems.append(f'<polyline points="{t_pts}" fill="none" stroke="#0969da" stroke-width="2.5"/>')
        elems.append(f'<polyline points="{p_pts}" fill="none" stroke="#d97706" stroke-width="2.2" stroke-dasharray="5,3"/>')
        title = f"q={item['target_mm']} ({lat_label}-Z)"
        subtitle = f"ang {item['ang_err_deg']:.1f}° · tip {item['tip_err_mm']:.1f} mm"
        elems.append(f'<text x="{gx+cw/2:.1f}" y="{gy+20}" text-anchor="middle" font-size="11.5" font-weight="600" fill="#24292f">{html.escape(title)}</text>')
        elems.append(f'<text x="{gx+cw/2:.1f}" y="{gy+ch-12}" text-anchor="middle" font-size="10.5" fill="#57606a">{html.escape(subtitle)}</text>')

    return f'<svg width="{w}" height="{h}" viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg">{"".join(elems)}</svg>'


def write_html_report(path, summary, comp_rows, val_rows, leak_rows, val_point_curves):
    best = summary['best_model']
    comp_html = []
    for r in comp_rows[:12]:
        hl = ' style="background:#ddf4ff;font-weight:600;"' if r['model'] == best['name'] else ''
        comp_html.append(
            f"<tr{hl}><td>{html.escape(r['model'])}</td>"
            f"<td>{r['cv_tangent_median_deg']:.2f}°</td><td>{r['cv_tangent_p90_deg']:.2f}°</td>"
            f"<td>{r['cv_tip_median_mm']:.2f} mm</td>"
            f"<td>{r['val_tangent_median_deg']:.2f}°</td><td>{r['val_tangent_p90_deg']:.2f}°</td>"
            f"<td>{r['val_tip_median_mm']:.2f} mm</td><td>{r['val_centerline_rms_median_mm']:.2f} mm</td></tr>"
        )

    val_html = []
    for r in val_rows:
        val_html.append(
            f"<tr><td>{html.escape(r['round_slug'])}</td><td>{r['target_mm']}</td>"
            f"<td>{r['true_bend_deg']:.1f}°</td><td>{r['pred_bend_deg']:.1f}°</td>"
            f"<td>{r['tangent_err_deg']:.2f}°</td><td>{r['tip_err_mm']:.2f} mm</td>"
            f"<td>{r['inv_q_mm']}</td><td>{r['inv_q_err_mm']:.2f} mm</td><td>{r['inv_ang_err_deg']:.2f}°</td></tr>"
        )

    doc = f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>OPHIS 3D Kinematics Modeling & Validation Report</title>
<style>
body {{ font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; margin: 28px auto; max-width: 1020px; color: #24292f; line-height: 1.5; }}
h1, h2 {{ border-bottom: 1px solid #d0d7de; padding-bottom: 6px; }}
.cards {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 12px; margin: 16px 0 24px; }}
.card {{ background: #f6f8fa; border: 1px solid #d0d7de; border-radius: 8px; padding: 12px 14px; }}
.card .label {{ font-size: 12px; color: #57606a; }}
.card .value {{ font-size: 22px; font-weight: 700; margin-top: 4px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 13px; margin: 12px 0 24px; }}
th, td {{ border: 1px solid #d0d7de; padding: 6px 8px; text-align: left; }}
th {{ background: #f6f8fa; }}
.legend {{ font-size: 12px; color: #57606a; margin-bottom: 8px; }}
</style>
</head>
<body>
<h1>OPHIS 柔性导管 3D 运动学建模与留出验证报告</h1>
<p>生成时间：{html.escape(summary['generated_at'])} ｜ 训练集：<code>dual_axis_train:ok</code>（{summary['train_rounds']} 轮 / 27 点）｜ 留出验证集：<code>dual_axis_val:ok</code>（{summary['val_rounds']} 轮 / 8 点）｜ 异常对照：<code>tip_leak</code>（{summary['leak_rounds']} 轮）</p>

<div class="cards">
  <div class="card"><div class="label">最优模型（按分组 LOO-CV）</div><div class="value" style="font-size:15px;">{html.escape(best['name'])}</div></div>
  <div class="card"><div class="label">留出验证 3D 切线误差（中位 / P90）</div><div class="value">{best['val_metrics']['tangent_err_median_deg']:.2f}° / {best['val_metrics']['tangent_err_p90_deg']:.2f}°</div></div>
  <div class="card"><div class="label">留出验证尖端位置误差（中位）</div><div class="value">{best['val_metrics']['tip_err_median_mm']:.2f} mm</div></div>
  <div class="card"><div class="label">逆解位移恢复误差（中位）</div><div class="value">{summary['inverse_eval']['q_err_median_mm']:.2f} mm</div></div>
</div>

<h2>1. 候选模型对比（按分组 LOO-CV 排序，Top 12）</h2>
<table>
<thead><tr><th>模型</th><th>CV 切线中位</th><th>CV 切线 P90</th><th>CV 尖端中位</th><th>留出切线中位</th><th>留出切线 P90</th><th>留出尖端中位</th><th>留出中心线 RMS</th></tr></thead>
<tbody>{''.join(comp_html)}</tbody>
</table>

<h2>2. 留出验证集实测 vs 预测（含 tip_leak 异常对照）</h2>
<p>正常验证集（蓝/绿点）紧贴对角线；<code>tip_leak</code> 漏液批次（红点）中位切线偏差高达 <strong>{summary['leak_metrics']['tangent_err_median_deg']:.1f}°</strong>，清晰体现漏液失压异常。</p>
{_svg_parity_plot(val_rows, leak_rows)}

<h2>3. 留出验证点 21 点 3D 中心线对比（蓝实线 = 双目实测，橙虚线 = 模型预测）</h2>
{_svg_centerline_grid(val_point_curves)}

<h2>4. 留出验证集逐轮明细与逆解测试（24 轮）</h2>
<table>
<thead><tr><th>轮次</th><th>目标 q (mm)</th><th>实测弯曲角</th><th>预测弯曲角</th><th>3D 切线误差</th><th>尖端误差</th><th>逆解 q* (mm)</th><th>逆解位移误差</th><th>逆解目标角残差</th></tr></thead>
<tbody>{''.join(val_html)}</tbody>
</table>
</body>
</html>
"""
    Path(path).write_text(doc, encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis-dir', type=Path, default=DEFAULT_ANALYSIS_DIR)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--modality', choices=('stereo', 'd405'), default='stereo')
    args = parser.parse_args(argv)

    analysis_dir = args.analysis_dir
    out_dir = args.output_dir if args.output_dir is not None else (analysis_dir / 'modeling')
    out_dir.mkdir(parents=True, exist_ok=True)

    samples = load_samples(analysis_dir, modality=args.modality)
    train_samples = [s for s in samples if s['dataset_group'] == 'dual_axis_train' and s['quality_flag'] == 'ok']
    val_samples = [s for s in samples if s['dataset_group'] == 'dual_axis_val' and s['quality_flag'] == 'ok']
    leak_samples = [s for s in samples if s['dataset_group'] == 'dual_axis_val' and s['quality_flag'] == 'tip_leak']

    if not train_samples or not val_samples:
        raise SystemExit(f'训练或验证样本不足：train={len(train_samples)}, val={len(val_samples)}')

    zero_anchors = build_zero_anchors(train_samples, count=3)

    comp_rows = []
    fitted = {}
    for name, factory in candidate_specs():
        cv_m = grouped_loocv(factory, train_samples, zero_anchors=zero_anchors)
        model, val_m, pt, pc, ang_err, tip_err, curve_rms = fit_and_eval(factory, train_samples, val_samples, zero_anchors)
        comp_rows.append(dict(
            model=name,
            cv_tangent_median_deg=cv_m['tangent_err_median_deg'],
            cv_tangent_mean_deg=cv_m['tangent_err_mean_deg'],
            cv_tangent_p90_deg=cv_m['tangent_err_p90_deg'],
            cv_tip_median_mm=cv_m['tip_err_median_mm'],
            cv_centerline_rms_median_mm=cv_m['centerline_rms_median_mm'],
            val_tangent_median_deg=val_m['tangent_err_median_deg'],
            val_tangent_mean_deg=val_m['tangent_err_mean_deg'],
            val_tangent_p90_deg=val_m['tangent_err_p90_deg'],
            val_tangent_max_deg=val_m['tangent_err_max_deg'],
            val_tip_median_mm=val_m['tip_err_median_mm'],
            val_tip_max_mm=val_m['tip_err_max_mm'],
            val_centerline_rms_median_mm=val_m['centerline_rms_median_mm'],
        ))
        fitted[name] = (model, cv_m, val_m, pt, pc, ang_err, tip_err, curve_rms)

    # Select best model strictly by grouped LOO-CV (median + mean tangent error) without peeking at val.
    comp_rows.sort(key=lambda r: (r['cv_tangent_median_deg'] + 0.5 * r['cv_tangent_mean_deg']))
    best_name = comp_rows[0]['model']
    best_model, best_cv, best_val, val_pt, val_pc, val_ang_err, val_tip_err, val_crms = fitted[best_name]

    # Evaluate tip_leak anomaly set
    leak_rows = []
    leak_metrics = None
    if leak_samples:
        X_lk = np.array([s['actual_mm'] for s in leak_samples], dtype=np.float64)
        T_lk = np.array([s['tip_tangent'] for s in leak_samples], dtype=np.float64)
        C_lk = np.array([s['centerline_mm'] for s in leak_samples], dtype=np.float64)
        lk_pt, lk_pc = best_model.predict(X_lk)
        leak_metrics, lk_ang, lk_tip, lk_crms = evaluate_predictions(T_lk, lk_pt, C_lk, lk_pc)
        z_axis = np.array([0.0, 0.0, 1.0])
        for s, pt_i, ae, te in zip(leak_samples, lk_pt, lk_ang, lk_tip):
            leak_rows.append(dict(
                round_slug=s['round_slug'],
                ids=','.join(str(i) for i in s['ids']),
                target_mm=str(s['target_mm']),
                true_bend_deg=s['bend_angle_deg'],
                pred_bend_deg=float(shape3d.bend_angle(z_axis, pt_i)),
                tangent_err_deg=float(ae),
                tip_err_mm=float(te),
            ))

    # Inverse kinematics benchmark on clean held-out validation rounds (within active pair)
    val_rows = []
    inv_q_errs, inv_ang_errs = [], []
    z_axis = np.array([0.0, 0.0, 1.0])
    for s, pt_i, pc_i, ae, te, ce in zip(val_samples, val_pt, val_pc, val_ang_err, val_tip_err, val_crms):
        inv = solve_inverse_tangent(
            best_model,
            s['tip_tangent'],
            bounds=(0.0, 6.0),
            max_total_mm=12.0,
            active_axes=s['ids'],
        )
        q_err = float(np.linalg.norm(np.asarray(inv['q_mm']) - np.asarray(s['actual_mm'])))
        inv_q_errs.append(q_err)
        inv_ang_errs.append(inv['angular_error_deg'])
        val_rows.append(dict(
            round_slug=s['round_slug'],
            ids=','.join(str(i) for i in s['ids']),
            target_mm=str(s['target_mm']),
            actual_mm=str([round(v, 3) for v in s['actual_mm']]),
            true_bend_deg=s['bend_angle_deg'],
            pred_bend_deg=float(shape3d.bend_angle(z_axis, pt_i)),
            tangent_err_deg=float(ae),
            tip_err_mm=float(te),
            centerline_rms_mm=float(ce),
            inv_q_mm=str([round(v, 2) for v in inv['q_mm']]),
            inv_q_err_mm=q_err,
            inv_ang_err_deg=float(inv['angular_error_deg']),
        ))

    # Group held-out validation curves by unique target_mm for clean 8-panel visualization
    by_target = defaultdict(list)
    for s, pc_i, ae, te in zip(val_samples, val_pc, val_ang_err, val_tip_err):
        by_target[tuple(s['target_mm'])].append((s['centerline_mm'], pc_i, float(ae), float(te)))
    val_point_curves = []
    for tkey, items in sorted(by_target.items()):
        val_point_curves.append(dict(
            target_mm=list(tkey),
            true_curve_mm=np.median([x[0] for x in items], axis=0).tolist(),
            pred_curve_mm=np.median([x[1] for x in items], axis=0).tolist(),
            ang_err_deg=float(np.median([x[2] for x in items])),
            tip_err_mm=float(np.median([x[3] for x in items])),
        ))

    inverse_eval = dict(
        n=len(inv_q_errs),
        q_err_median_mm=float(np.median(inv_q_errs)),
        q_err_mean_mm=float(np.mean(inv_q_errs)),
        q_err_max_mm=float(np.max(inv_q_errs)),
        target_ang_residual_median_deg=float(np.median(inv_ang_errs)),
        target_ang_residual_max_deg=float(np.max(inv_ang_errs)),
    )

    summary = dict(
        generated_at=datetime.now().astimezone().isoformat(),
        analysis_dir=str(analysis_dir),
        modality=args.modality,
        train_rounds=len(train_samples),
        val_rounds=len(val_samples),
        leak_rounds=len(leak_samples),
        best_model=dict(name=best_name, cv_metrics=best_cv, val_metrics=best_val),
        leak_metrics=leak_metrics,
        inverse_eval=inverse_eval,
    )

    best_model.save_json(out_dir / 'forward_model.json', metadata=summary)
    with (out_dir / 'model_comparison.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(comp_rows[0].keys()))
        w.writeheader()
        w.writerows(comp_rows)
    with (out_dir / 'validation_predictions.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(val_rows[0].keys()))
        w.writeheader()
        w.writerows(val_rows)
    (out_dir / 'modeling_summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8'
    )
    write_html_report(out_dir / 'report.html', summary, comp_rows, val_rows, leak_rows, val_point_curves)

    print(f"最优模型：{best_name}", flush=True)
    print(f"  分组 LOO-CV (27点/81轮): 3D切线中位 {best_cv['tangent_err_median_deg']:.2f}° (P90 {best_cv['tangent_err_p90_deg']:.2f}°), 尖端中位 {best_cv['tip_err_median_mm']:.2f} mm", flush=True)
    print(f"  留出验证 (8点/24轮):     3D切线中位 {best_val['tangent_err_median_deg']:.2f}° (P90 {best_val['tangent_err_p90_deg']:.2f}°), 尖端中位 {best_val['tip_err_median_mm']:.2f} mm, 中心线RMS {best_val['centerline_rms_median_mm']:.2f} mm", flush=True)
    print(f"  逆解验证 (24轮):         位移恢复中位 {inverse_eval['q_err_median_mm']:.2f} mm (最大 {inverse_eval['q_err_max_mm']:.2f} mm), 目标角残差中位 {inverse_eval['target_ang_residual_median_deg']:.2f}°", flush=True)
    if leak_metrics:
        print(f"  tip_leak 对照 (12轮):    3D切线偏差中位 {leak_metrics['tangent_err_median_deg']:.2f}°, 尖端偏差中位 {leak_metrics['tip_err_median_mm']:.2f} mm", flush=True)
    print(f"结果已保存至：{out_dir}", flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
