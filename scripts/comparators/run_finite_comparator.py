"""Portable runner for 1200 m finite-range comparator baselines (Oruganti, Cano, Goudar, Tube-DMPC)."""
import argparse
import gzip
import hashlib
import json
import os
import selectors
import shutil
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCRIPTS_DIR = HERE.parent
ROOT = SCRIPTS_DIR.parent
CONFIG_DIR = ROOT / 'configs'

sys.path.insert(0, str(SCRIPTS_DIR))
import active_set_ranges as ranging
from active_set_ranges import active_set_ranges
from trace_estimator import EKFInLoopService

METHODS = {
    'oruganti': {'method': 'oruganti', 'parameters': {'solver': 'clarabel'}},
    'cano': {'method': 'cano', 'parameters': {'target_method': 'cano'}},
    'goudar': {
        'method': 'goudar',
        'parameters': {
            'target_method': 'goudar',
            'target_parameters': {
                'horizon_steps': 10,
                'acceleration_psd': 10.0,
                'velocity_prior_variance': 1.0,
                'covariance_floor': 1e-8,
                'max_nfev': 80,
                'edge_length_m': 750.0,
            },
        },
    },
    'mpc': {'method': 'artificial-reference-tube-dmpc', 'parameters': None},
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new(path: Path, obj) -> None:
    with path.open('x') as handle:
        json.dump(obj, handle, indent=2, allow_nan=False)
        handle.write('\n')


def run_comparator(method_key: str, binary: Path, out: Path, duration: float, cutoff_m, terminate_on_complete: bool) -> dict:
    spec = METHODS[method_key]
    seed = 2026102801
    if not binary.is_file():
        raise FileNotFoundError(f'native executable not found: {binary}')
    out = out.resolve()
    if out.exists():
        raise FileExistsError(f'output directory already exists: {out}')
    out.mkdir(parents=True, exist_ok=False)

    cfg = json.loads((CONFIG_DIR / 'main14-1200m-piqp.json').read_text())
    cfg['execute'].update({
        'time-total': duration,
        'time-step': 0.1,
        'terminate-on-coverage-complete': terminate_on_complete,
    })
    cfg['output_path'] = str(out / 'raw')
    cfg['run_suffix'] = f'_{method_key}'
    cfg['estimator-in-loop'].update({
        'state-path': str(out / 'state.json'),
        'estimates-path': str(out / 'estimates.json'),
        'wait-timeout-s': 180.0,
    })
    fifo = out / 'frames.fifo'
    os.mkfifo(fifo)
    cfg['work2'] = {
        'arm': 'original',
        'response_tau_s': 0.0,
        'frame_log': str(fifo),
        'terminal_path': str(out / 'terminal.json'),
    }
    params = spec['parameters']
    if method_key == 'mpc':
        from artificial_reference_controller import declared_parameters
        params = declared_parameters()
    cfg['comparator'] = {'method': spec['method'], 'parameters': params, 'ipc_directory': str(out)}
    write_new(out / 'config.json', cfg)

    zstd_bin = shutil.which('zstd')
    compression_errors = []

    def pump():
        worker = None
        try:
            if zstd_bin:
                with fifo.open('rb', buffering=0) as src, (out / 'frames.jsonl.zst').open('xb') as dst:
                    worker = subprocess.Popen(
                        [zstd_bin, '-q', '-9', '-c', '--threads=1'],
                        stdin=subprocess.PIPE, stdout=dst, stderr=subprocess.PIPE,
                    )
                    shutil.copyfileobj(src, worker.stdin, length=65536)
                    worker.stdin.close()
                    if worker.wait():
                        raise RuntimeError('zstd failed: ' + worker.stderr.read().decode(errors='replace'))
            else:
                with fifo.open('rb', buffering=0) as src, gzip.open(out / 'frames.jsonl.gz', 'wb', compresslevel=6) as dst:
                    shutil.copyfileobj(src, dst, length=65536)
        except Exception:
            compression_errors.append(traceback.format_exc())

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()

    ekf = EKFInLoopService(
        deployment_positions={i + 1: p for i, p in enumerate(cfg['initial']['position']['positions'])},
        bases=cfg['bases'],
        dt=0.1,
    )
    started = time.monotonic()
    served = controls = 0
    cov = None
    failure = None
    process = worker = None

    try:
        with (out / 'stdout.log').open('x') as stdout, \
             (out / 'stderr.log').open('x') as stderr, \
             (out / 'controller-stderr.log').open('x') as cstderr, \
             gzip.open(out / 'estimator.jsonl.gz', 'wt', compresslevel=1) as elog, \
             gzip.open(out / 'controller.jsonl.gz', 'wt', compresslevel=1) as clog, \
             gzip.open(out / 'ranging-audit.jsonl.gz', 'wt', compresslevel=1) as alog:
            worker = subprocess.Popen(
                [sys.executable, str(HERE / 'controller_worker.py'), str(out / 'config.json')],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=cstderr, text=True, bufsize=1,
            )
            selector = selectors.DefaultSelector()
            selector.register(worker.stdout, selectors.EVENT_READ)
            process = subprocess.Popen(
                [str(binary), str(out / 'config.json')],
                stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                env={**os.environ, 'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'VECLIB_MAXIMUM_THREADS': '1'},
            )
            last = time.monotonic()
            while process.poll() is None:
                acted = False
                ready = out / 'state.json.ready'
                if ready.exists():
                    state = json.loads((out / 'state.json').read_text())
                    ready.unlink()
                    frame = state['frame_index']
                    latent = frame // 5 if frame % 5 == 0 else 1000000 + 4 * (frame // 5) + (frame % 5) - 1
                    refs = active_set_ranges(state, cfg['bases'], seed, latent, cutoff_m)
                    alog.write(json.dumps({'frame': frame, 'latent_index': latent, 'edges': ranging.LAST_AUDIT},
                                          separators=(',', ':'), allow_nan=False) + '\n')
                    result = ekf.step(frame_index=frame, raw_reference_groups=refs,
                                      held_commands={r['id']: [r['vx'], r['vy']] for r in state['robots']})
                    cov = {str(i): ekf.state[i]['cov'].tolist() for i in sorted(result)}
                    elog.write(json.dumps({'frame': frame, 'result': result, 'covariance': cov,
                                           'ranges': refs, 'decisions': ekf.measurement_audit},
                                          separators=(',', ':'), allow_nan=False) + '\n')
                    tmp = out / 'estimates.json.tmp'
                    tmp.write_text(json.dumps({'frame_index': frame,
                                               'robots': [{'id': i, **{k: result[i][k] for k in ('estimate', 'epsilon', 'tier')}} for i in sorted(result)]},
                                              allow_nan=False) + '\n')
                    os.replace(tmp, out / 'estimates.json')
                    served += 1
                    last = time.monotonic()
                    acted = True
                cready = out / 'control-state.ready'
                if cready.exists():
                    request = json.loads((out / 'control-state.json').read_text())
                    cready.unlink()
                    cframe = request['frame']
                    worker.stdin.write(json.dumps({'request': request, 'covariance': [cov[str(n['id'])] for n in request['nodes']]},
                                                  separators=(',', ':'), allow_nan=False) + '\n')
                    worker.stdin.flush()
                    if not selector.select(timeout=180.0):
                        raise TimeoutError('external controller timed out')
                    line = worker.stdout.readline()
                    if not line:
                        raise RuntimeError('external controller exited: ' + (out / 'controller-stderr.log').read_text())
                    response = json.loads(line)
                    clog.write(json.dumps({'frame': cframe, 'response': response}, separators=(',', ':'), allow_nan=False) + '\n')
                    tmp = out / 'control-response.tmp'
                    tmp.write_text(json.dumps(response, allow_nan=False) + '\n')
                    os.replace(tmp, out / 'control-response.json')
                    controls += 1
                    last = time.monotonic()
                    acted = True
                if not acted:
                    if time.monotonic() - last > 200.0:
                        raise TimeoutError('native comparator IPC stalled')
                    time.sleep(0.002)
            process.wait()
    except BaseException as exc:
        failure = {'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()}
        if process and process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
    finally:
        if worker and worker.poll() is None:
            worker.terminate()
            worker.wait(timeout=5)
    thread.join(timeout=30)
    terminal = json.loads((out / 'terminal.json').read_text()) if (out / 'terminal.json').exists() else None
    status = 'finished' if terminal and process and process.returncode == 0 and not failure and not compression_errors else 'failed'
    receipt = {
        'method': method_key,
        'status': status,
        'binary_sha256': sha256(binary),
        'served_frames': served,
        'control_steps': controls,
        'terminal_time_s': terminal.get('time_s') if terminal else None,
        'true_coverage': terminal.get('true_coverage') if terminal else None,
        'elapsed_wall_s': time.monotonic() - started,
        'failure': failure,
        'compression_errors': compression_errors,
    }
    write_new(out / 'run-receipt.json', receipt)
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run 1200 m finite-range comparator baselines.')
    parser.add_argument('--method', choices=sorted(METHODS), required=True)
    parser.add_argument('--binary', type=Path, default=ROOT / 'build/cbf2026_native')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=700.0)
    parser.add_argument('--cutoff-m', type=float, default=1200.0)
    parser.add_argument('--no-early-stop', action='store_true')
    args = parser.parse_args()
    res = run_comparator(args.method, args.binary, args.output, args.duration, args.cutoff_m,
                         terminate_on_complete=not args.no_early_stop)
    print(json.dumps(res, indent=2))
    raise SystemExit(0 if res['status'] == 'finished' else 1)
