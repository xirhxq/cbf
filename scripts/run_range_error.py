"""Single 0.1-second native full task with streaming-compressed telemetry."""
from pathlib import Path
import argparse,gzip,hashlib,json,math,os,shutil,subprocess,threading,time,traceback
import numpy as np
from range_error_ekf_service import EKFInLoopService
from range_perturbation import RangePerturbation
import counter_field
import cano_planner as planner
HERE=Path(__file__).resolve().parent
DT=.1
OLD_FIELD=[]
FIELDS={'normal':'91a2a0da75e37efa217d6973879ec6545f49330030545aa510f772af19aea2fa','shared':'296e44b1e3056ad01477f89762a596af7caa192d916b2f7261e11ed0759515cd'}
INVENTORY=json.loads((HERE.parent/'config/range-error-index.json').read_text())['cases']
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2,allow_nan=False);f.write('\n')
def group_size(root):
    total=0
    for directory,_,names in os.walk(root):
        for name in names:
            try:
                p=Path(directory)/name
                if p.is_file() and not p.is_symlink():total+=p.stat().st_size
            except FileNotFoundError:pass
    return total

def paired_ranges(state,bases,seed,frame):
    pos={r['id']:np.array([r['x'],r['y']]) for r in state['robots']}
    latent=OLD_FIELD[frame] if frame<len(OLD_FIELD) else counter_field.latent(frame,[r['formation'] for r in state['robots']],seed);refs={i:[] for i in pos}
    for edge in latent['edges']:
        i,j,kind=edge['node'],edge['reference'],edge['kind'];anchor=np.array(bases[j]) if kind=='base' else pos[j]
        d=float(np.linalg.norm(pos[i]-anchor));sigma=.5*(1+(d/850)**2)
        if edge['availability_uniform']<=1/(1+(d/850)**2):
            refs[i].append({'key':(kind,j),'range':d+sigma*edge['standard_normal'],'true_range':d,'ranging_sigma':sigma})
    for values in refs.values():values.sort(key=lambda x:x['key'])
    return refs

def run(args):
    global OLD_FIELD
    case=INVENTORY[args.case];config_path=HERE.parent/case['config']
    perturbation=RangePerturbation(case,args.schedule)
    if sha(config_path)!=case['sanitized_config_sha256']:raise RuntimeError('case configuration hash mismatch')
    field=case['random_field'];OLD_FIELD=[]
    if field!='counter':
        if not args.field:raise ValueError('this case requires its historical frozen field')
        field_path=Path(args.field).expanduser().resolve()
        if sha(field_path)!=FIELDS[field]:raise RuntimeError('frozen field SHA-256 mismatch')
        with field_path.open() as f:OLD_FIELD=[json.loads(line) for line in f]
        if len(OLD_FIELD)!=700:raise RuntimeError('frozen field must contain 700 records')
    elif args.field:raise ValueError('this case uses the counter field, not a frozen field')
    args.seed=case['seed'];horizon=case['horizon_s'];num=case['num_uav'];max_frames=round(horizon/DT)
    is_cano=case['variant'] in ('cano','immediate-cano')
    binary=Path(args.binary).expanduser().resolve()
    if not binary.is_file():raise FileNotFoundError(f"native binary not found: {binary}")
    zstd=shutil.which("zstd")
    if not zstd:raise RuntimeError("zstd executable not found in PATH")
    out=Path(args.output).expanduser().resolve()
    if out.exists():raise FileExistsError(f"output path already exists: {out}")
    out.parent.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(out.parent).free < args.min_free_mib*1024**2:raise RuntimeError("insufficient output filesystem free space")
    out.mkdir(exist_ok=False)
    cfg=json.loads(config_path.read_text())
    cfg['output_path']=str(out/'raw')
    cfg['estimator-in-loop'].update({'state-path':str(out/'state.json'),'estimates-path':str(out/'estimates.json')})
    fifo=out/'frames.fifo';os.mkfifo(fifo)
    cfg['work2']['frame_log']=str(fifo);cfg['work2']['terminal_path']=str(out/'terminal.json')
    if is_cano:
        cfg['cano-target-request-path']=str(out/'request.json');cfg['cano-target-response-path']=str(out/'response.json')
    write(out/'config.json',cfg);write(out/'once.json',{'config_template_sha256':sha(config_path),
        'binary_sha256':sha(binary),'seed':args.seed,'case':args.case,'variant_declared':case['variant'],'field_sha256':None if field=='counter' else FIELDS[field],'schedule_sha256':perturbation.schedule_sha256,'historical_schedule_sha256':case.get('schedule_sha256'),'perturbation':case['perturbation'],'amplitude_m':case['amplitude_m'],'dt':DT,'no_retry':True})
    failures=[]
    def pump():
        worker=None
        try:
            with fifo.open('rb',buffering=0) as source,(out/'frames.jsonl.zst').open('xb') as dest:
                worker=subprocess.Popen([zstd,'-q','-9','-c','--threads=1'],stdin=subprocess.PIPE,stdout=dest,stderr=subprocess.PIPE)
                shutil.copyfileobj(source,worker.stdin,length=65536)
                worker.stdin.close();rc=worker.wait()
                if rc:raise RuntimeError('zstd encoder failed: '+worker.stderr.read().decode(errors='replace'))
        except Exception:
            failures.append(traceback.format_exc())
            if worker is not None and worker.poll() is None:
                worker.kill();worker.wait()
    thread=threading.Thread(target=pump,daemon=True);thread.start()
    ekf=EKFInLoopService(deployment_positions={i+1:x for i,x in enumerate(cfg['initial']['position']['positions'])},
                         bases=cfg['bases'],anchor_covariance_scale=3.,dt=DT)
    proc=None;error=None;served=0;started=time.monotonic();follower=planner.PeriodicFollowerState()
    print(json.dumps({'started':'dt0.1','seed':args.seed,'max_frames':max_frames}),flush=True)
    try:
        with (out/'stdout.log').open('x') as stdout,(out/'stderr.log').open('x') as stderr,gzip.open(out/'estimator.jsonl.gz','wt',compresslevel=1) as elog,gzip.open(out/'planner-events.jsonl.gz','wt',compresslevel=1) as plog:
            proc=subprocess.Popen([str(binary),str(out/'config.json')],stdin=subprocess.DEVNULL,stdout=stdout,stderr=stderr,
                env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','VECLIB_MAXIMUM_THREADS':'1'})
            for frame in range(max_frames):
                if frame % 50 == 0:
                    if shutil.disk_usage(out).free < args.min_free_mib*1024**2:
                        raise RuntimeError('output filesystem free-space threshold reached')
                    if group_size(out) >= args.max_output_mib*1024**2:
                        raise RuntimeError('monitored output-size limit reached')
                ready=out/'state.json.ready';deadline=time.monotonic()+90
                while not ready.exists():
                    if proc.poll() is not None:break
                    if time.monotonic()>deadline:raise TimeoutError(f'no state for frame {frame}')
                    time.sleep(.002)
                if not ready.exists():break
                state=json.loads((out/'state.json').read_text());ready.unlink()
                assert state['frame_index']==frame and len(state['robots'])==num
                # Observe and predict at each 0.1 s update; map nominal 0.5 s draws to their original indices.
                observation=True
                latent_index=frame//5 if frame%5==0 else 1000000+4*(frame//5)+(frame%5)-1
                refs=paired_ranges(state,cfg['bases'],args.seed,latent_index)
                events=perturbation.apply(refs,frame)
                ekf.injected_audit=[]
                # Use the original formula Q=(q*dt)^2 I with q=1 m/s at every update.
                ekf.process_noise_mps=1.
                result=ekf.step(frame_index=frame,raw_reference_groups=refs,held_commands={r['id']:[r['vx'],r['vy']] for r in state['robots']})
                elog.write(json.dumps({'frame':frame,'measurement_update':observation,'measurement_index':latent_index,'result':result,'covariance':{str(i):ekf.state[i]['cov'].tolist() for i in sorted(result)},'ranges':refs,'injection_events':events,'outlier_gate_audit':ekf.injected_audit},separators=(',',':'),allow_nan=False)+'\n')
                payload={'frame_index':frame,'robots':[{'id':i,'estimate':result[i]['estimate'],'epsilon':result[i]['epsilon'],'tier':result[i]['tier']} for i in sorted(result)]}
                temp=out/'estimates.json.tmp';temp.write_text(json.dumps(payload,allow_nan=False)+'\n');os.replace(temp,out/'estimates.json');served+=1
                if is_cano:
                    request=out/'request.json';deadline=time.monotonic()+90
                    while not request.exists():
                        if proc.poll() is not None:break
                        if time.monotonic()>deadline:raise TimeoutError('Cano target request')
                        time.sleep(.002)
                    if not request.exists():
                        if proc.poll()==0:break # terminal online observation needs no new plan
                        raise RuntimeError('native exited before Cano target request')
                    packet=json.loads(request.read_text());request.unlink();assert packet['frame']==frame
                    edges=planner.assigned_graph(state['robots']);positions=np.array([result[i]['estimate'] for i in range(1,15)])
                    targets,event=follower.advance(frame,positions,cfg['bases'],edges)
                    if event is not None:
                        event.update({'snapshot_frame':frame,'ranging_edges':edges,'estimated_positions_m':positions.tolist(),'known_bases_m':cfg['bases'],'fim_interpretation':'fixed design sigma, not measured instantaneous FIM'})
                        plog.write(json.dumps(event,separators=(',',':'),allow_nan=False)+'\n')
                    reply={'frame':frame,'followers':[{'id':i+1,'plan_frame':frame-frame%50,'raw_m':targets[i].tolist()} for i in range(14) if i not in (6,13)]}
                    temporary=out/'response.json.tmp';temporary.write_text(json.dumps(reply,allow_nan=False)+'\n');os.replace(temporary,out/'response.json')
                if served%500==0:print('dt0.1 served',served,'elapsed',round(time.monotonic()-started,1),flush=True)
            rc=proc.wait(timeout=120)
            if rc:raise RuntimeError(f'native exited {rc}')
            terminal=json.loads((out/'terminal.json').read_text())
            if terminal['time_s']<horizon-1e-8 and terminal['estimated_coverage_last_measured']<1.-1e-12:raise RuntimeError('early native exit before online completion')
    except BaseException as ex:
        error={'type':type(ex).__name__,'message':str(ex),'traceback':traceback.format_exc()}
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()
    # Release an unopened FIFO reader on early native startup failure.
    if thread.is_alive() and (proc is None or proc.poll() is not None):
        try:fd=os.open(fifo,os.O_WRONLY|os.O_NONBLOCK);os.close(fd)
        except OSError:pass
    thread.join(timeout=30)
    if thread.is_alive() or failures:
        error=error or {'type':'TelemetryError','message':'compressed frame stream incomplete','details':failures}
    receipt={'category':'runs','arm':'original','seed':args.seed,'dt_s':DT,'served_estimator_frames':served,
        'requested_frames':max_frames,'scheduled_events':len(perturbation.matched),'applied_outlier_events':perturbation.applied,'native_returncode':None if proc is None else proc.returncode,
        'status':'finished' if error is None else 'failed-prefix-retained','error':error,
        'elapsed_s':time.monotonic()-started,'no_retry':True,'frames_zstd':str(out/'frames.jsonl.zst')}
    write(out/'run-receipt.json',receipt)
    print(json.dumps(receipt),flush=True);return receipt


def main():
    parser=argparse.ArgumentParser(description="Run one of the 22 ranging-error study conditions")
    parser.add_argument("--case",choices=sorted(INVENTORY),required=True,help="case identifier from the configuration index")
    parser.add_argument("--schedule",help="historical matched schedule; reference-run label may be relocated")
    parser.add_argument("--field",help="historical frozen random-field file when required by the case")
    parser.add_argument("--binary",required=True,help="path to the separately built native executable")
    parser.add_argument("--output",required=True,help="new output directory; must not already exist")
    parser.add_argument("--min-free-mib",type=int,default=1024,help="minimum free space on the output filesystem")
    parser.add_argument("--max-output-mib",type=int,default=320,help="monitored output-size limit")
    args=parser.parse_args()
    if args.min_free_mib<0 or args.max_output_mib<=0:parser.error("invalid output limits")
    result=run(args)
    if result.get("status")!="finished":raise SystemExit(1)

if __name__=="__main__":
    main()
