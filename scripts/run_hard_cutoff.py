"""Portable runner for 1200 m hard-cutoff active-set ranging + SCI numerical experiments."""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CONFIG_DIR = ROOT / 'configs'
SCHED_DIR = CONFIG_DIR / 'schedules'

sys.path.insert(0, str(HERE))
import active_set_ranges as ranging
from active_set_ranges import active_set_ranges
from cr3_estimator import EKFInLoopService as CR3Service
from range_perturbation import RangePerturbation
from trace_estimator import EKFInLoopService as SCIService

CASES = {
    'main14': {'config': 'main14-1200m-piqp.json', 'seed': 2026102801},
    'n10': {'config': 'n10-1200m-piqp.json', 'seed': 2026102906},
    'squad21': {'config': 'squad21-1200m-piqp.json', 'seed': 2026102901},
    'square2000': {'config': 'scene-square-2000-3sigma-1200m-piqp.json', 'seed': 2026102801},
    'square2500': {'config': 'scene-square-2500-3sigma-1200m-piqp.json', 'seed': 2026102801},
    'triangle': {'config': 'scene-triangle-3sigma-1200m-piqp.json', 'seed': 2026102801},
    'trapezoid': {'config': 'scene-trapezoid-3sigma-1200m-piqp.json', 'seed': 2026102801},
    'fixed1m': {'config': 'main14-1200m-piqp.json', 'seed': 2026102801, 'fixed': 1.0},
    'fixed3m': {'config': 'main14-1200m-piqp.json', 'seed': 2026102801, 'fixed': 3.0},
    'fixed9m': {'config': 'main14-1200m-piqp.json', 'seed': 2026102801, 'fixed': 9.0},
    'cr3': {'config': 'main14-1200m-piqp.json', 'seed': 2026102801, 'estimator': 'cr3'},
    'bias': {
        'config': 'main14-1200m-piqp.json',
        'seed': 2026102801,
        'perturbation': {'perturbation': 'bias', 'amplitude_m': 0.30, 'window_s': None},
    },
    'burst': {
        'config': 'main14-1200m-piqp.json',
        'seed': 2026102801,
        'perturbation': {'perturbation': 'burst', 'amplitude_m': 3.0, 'window_s': None},
    },
    'dispersed20': {
        'config': 'main14-1200m-piqp.json',
        'seed': 2026102801,
        'perturbation': {
            'perturbation': 'matched',
            'amplitude_m': 3.0,
            'seed': 2026102801,
            'window_s': 20,
            'schedule_sha256': 'd005aa1c731337ede6cfde42acee15be07ea154d7b08f0d233ceca53d0efd8ec',
            'schedule_canonical_sha256': 'd005aa1c731337ede6cfde42acee15be07ea154d7b08f0d233ceca53d0efd8ec',
        },
        'schedule': 'dispersed20-schedule.json',
    },
    'dispersed40': {
        'config': 'main14-1200m-piqp.json',
        'seed': 2026102801,
        'perturbation': {
            'perturbation': 'matched',
            'amplitude_m': 3.0,
            'seed': 2026102801,
            'window_s': 40,
            'schedule_sha256': '1570e13d7fb7112c59e164f6837069ab521fcb6b7f835363867dfe705da654be',
            'schedule_canonical_sha256': '1570e13d7fb7112c59e164f6837069ab521fcb6b7f835363867dfe705da654be',
        },
        'schedule': 'dispersed40-schedule.json',
    },
    'dispersed100': {
        'config': 'main14-1200m-piqp.json',
        'seed': 2026102801,
        'perturbation': {
            'perturbation': 'matched',
            'amplitude_m': 3.0,
            'seed': 2026102801,
            'window_s': 100,
            'schedule_sha256': '43836450b43a98c0e279199845c3500f9a890decc9f9a22d0d2f68f8d9b0e7f8',
            'schedule_canonical_sha256': '43836450b43a98c0e279199845c3500f9a890decc9f9a22d0d2f68f8d9b0e7f8',
        },
        'schedule': 'dispersed100-schedule.json',
    },
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path: Path, obj) -> None:
    with path.open('x') as handle:
        json.dump(obj, handle, indent=2, allow_nan=False)
        handle.write('\n')


def dir_size(root: Path) -> int:
    total = 0
    for parent, _, files in os.walk(root):
        for name in files:
            p = Path(parent) / name
            if p.is_file() and not p.is_symlink():
                total += p.stat().st_size
    return total


def run_case(case: str, binary: Path, out: Path, duration: float, cutoff_m, terminate_on_complete: bool) -> dict:
    spec = CASES[case]
    seed = spec['seed']
    if not binary.is_file():
        raise FileNotFoundError(f'native executable not found: {binary}')
    out = out.resolve()
    if out.exists():
        raise FileExistsError(f'output directory already exists: {out}')
    out.mkdir(parents=True, exist_ok=False)

    cfg = json.loads((CONFIG_DIR / spec['config']).read_text())
    cfg['execute'].update({
        'time-total': duration,
        'time-step': 0.1,
        'terminate-on-coverage-complete': terminate_on_complete,
    })
    exp = cfg['tvt-experiment']
    for key in ('signed-reserve-enabled', 'interpolation-filter-enabled', 'stall-stop'):
        exp.pop(key, None)
    if 'fixed' in spec:
        exp['ekf-epsilon-scale'] = 1.0
        exp['error-margin-mode'] = 'fixed'
        exp['node-error-bound-m'] = spec['fixed']

    perturb = None
    if 'perturbation' in spec:
        sched = str(SCHED_DIR / spec['schedule']) if 'schedule' in spec else None
        perturb = RangePerturbation(spec['perturbation'], sched)

    cfg['output_path'] = str(out / 'raw')
    cfg['run_suffix'] = '_sampled_hold_current'
    cfg['estimator-in-loop'].update({
        'state-path': str(out / 'state.json'),
        'estimates-path': str(out / 'estimates.json'),
        'wait-timeout-s': 90.0,
    })
    fifo = out / 'frames.fifo'
    os.mkfifo(fifo)
    cfg['work2'] = {
        'arm': 'original',
        'response_tau_s': 0.0,
        'frame_log': str(fifo),
        'terminal_path': str(out / 'terminal.json'),
    }
    write_new(out / 'config.json', cfg)

    zstd_bin = shutil.which('zstd')
    errors = []

    def pump():
        worker = None
        try:
            if zstd_bin:
                with fifo.open('rb', buffering=0) as src, (out / 'frames.jsonl.zst').open('xb') as dst:
                    worker = subprocess.Popen(
                        [zstd_bin, '-q', '-9', '-c', '--threads=1'],
                        stdin=subprocess.PIPE,
                        stdout=dst,
                        stderr=subprocess.PIPE,
                    )
                    shutil.copyfileobj(src, worker.stdin, length=65536)
                    worker.stdin.close()
                    if worker.wait():
                        raise RuntimeError('zstd failed: ' + worker.stderr.read().decode(errors='replace'))
            else:
                with fifo.open('rb', buffering=0) as src, gzip.open(out / 'frames.jsonl.gz', 'wb', compresslevel=6) as dst:
                    shutil.copyfileobj(src, dst, length=65536)
        except Exception:
            errors.append(traceback.format_exc())
            if worker is not None and worker.poll() is None:
                worker.kill()
                worker.wait()

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()

    deploy = {i + 1: p for i, p in enumerate(cfg['initial']['position']['positions'])}
    if spec.get('estimator') == 'cr3':
        ekf = CR3Service(deployment_positions=deploy, bases=cfg['bases'], dt=0.1,
                         anchor_covariance_scale=3.0, fusion_mode='cr')
    else:
        ekf = SCIService(deployment_positions=deploy, bases=cfg['bases'], dt=0.1)

    expected = 0
    error = None
    proc = None
    started = time.monotonic()
    try:
        with (out / 'stdout.log').open('x') as stdout, \
             (out / 'stderr.log').open('x') as stderr, \
             gzip.open(out / 'estimator.jsonl.gz', 'wt', compresslevel=1) as elog, \
             gzip.open(out / 'ranging-audit.jsonl.gz', 'wt', compresslevel=1) as alog:
            proc = subprocess.Popen(
                [str(binary), str(out / 'config.json')],
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=stderr,
                env={**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'VECLIB_MAXIMUM_THREADS': '1'},
            )
            deadline = time.monotonic() + 120.0
            while proc.poll() is None:
                ready = out / 'state.json.ready'
                if ready.exists():
                    state = json.loads((out / 'state.json').read_text())
                    ready.unlink()
                    frame = state['frame_index']
                    if frame != expected:
                        raise RuntimeError('estimator frame order mismatch')
                    latent = frame // 5 if frame % 5 == 0 else 1000000 + 4 * (frame // 5) + (frame % 5) - 1
                    refs = active_set_ranges(state, cfg['bases'], seed, latent, cutoff_m)
                    alog.write(json.dumps(
                        {'frame': frame, 'latent_index': latent, 'edges': ranging.LAST_AUDIT},
                        separators=(',', ':'), allow_nan=False
                    ) + '\n')
                    events = perturb.apply(refs, frame) if perturb else []
                    t0 = time.perf_counter()
                    result = ekf.step(
                        frame_index=frame,
                        raw_reference_groups=refs,
                        held_commands={r['id']: [r['vx'], r['vy']] for r in state['robots']},
                    )
                    elog.write(json.dumps({
                        'frame': frame,
                        'result': result,
                        'covariance': {str(i): ekf.state[i]['cov'].tolist() for i in sorted(result)},
                        'ranges': refs,
                        'perturbation_events': events,
                        'decisions': ekf.measurement_audit,
                        'ekf_wall_s': time.perf_counter() - t0,
                    }, separators=(',', ':'), allow_nan=False) + '\n')
                    payload = {
                        'frame_index': frame,
                        'robots': [{'id': i, **{k: result[i][k] for k in ('estimate', 'epsilon', 'tier')}} for i in sorted(result)],
                    }
                    tmp = out / 'estimates.json.tmp'
                    tmp.write_text(json.dumps(payload, allow_nan=False) + '\n')
                    os.replace(tmp, out / 'estimates.json')
                    expected += 1
                    deadline = time.monotonic() + 120.0
                elif time.monotonic() > deadline:
                    raise TimeoutError('native state timed out')
                else:
                    time.sleep(0.002)
            proc.wait()
    except BaseException as ex:
        error = {'type': type(ex).__name__, 'message': str(ex), 'traceback': traceback.format_exc()}
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    thread.join(timeout=30)
    if thread.is_alive():
        errors.append('compression thread failed to stop')
    terminal = json.loads((out / 'terminal.json').read_text()) if (out / 'terminal.json').exists() else None
    status = 'finished' if terminal and proc and proc.returncode == 0 and not error and not errors else 'failed'
    receipt = {
        'case': case,
        'status': status,
        'seed': seed,
        'binary_sha256': sha256(binary),
        'served_frames': expected,
        'native_returncode': proc.returncode if proc else None,
        'cutoff_m': cutoff_m,
        'applied_outlier_events': perturb.applied if perturb else 0,
        'error': error,
        'compression_errors': errors,
        'terminal_time_s': terminal.get('time_s') if terminal else None,
        'true_coverage': terminal.get('true_coverage') if terminal else None,
        'elapsed_wall_s': time.monotonic() - started,
        'output_bytes': dir_size(out),
    }
    write_new(out / 'run-receipt.json', receipt)
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run 1200 m active-set ranging SCI / ablation cases.')
    parser.add_argument('--case', choices=sorted(CASES), required=True)
    parser.add_argument('--binary', type=Path, default=ROOT / 'build/cbf2026_native')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=700.0)
    parser.add_argument('--cutoff-m', type=float, default=1200.0)
    parser.add_argument('--no-cutoff', action='store_true')
    parser.add_argument('--no-early-stop', action='store_true')
    args = parser.parse_args()
    res = run_case(
        args.case,
        args.binary,
        args.output,
        args.duration,
        None if args.no_cutoff else args.cutoff_m,
        terminate_on_complete=not args.no_early_stop,
    )
    print(json.dumps(res, indent=2))
    raise SystemExit(0 if res['status'] == 'finished' else 1)
