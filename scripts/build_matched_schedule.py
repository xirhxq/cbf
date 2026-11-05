"""Build matched schedules from a completed burst estimator log, without running a simulation."""
import argparse
import collections
import gzip
import hashlib
import json
from pathlib import Path


def filehash(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda:source.read(1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()


def build_schedule(source,seed,width,reference_run=None):
    if width not in (20,40,100):raise ValueError('window must be 20, 40 or 100 seconds')
    source=Path(source);quota=collections.Counter();candidates=collections.defaultdict(list);events=frames=0
    with gzip.open(source,'rt') as f:
        for line in f:
            x=json.loads(line);tick=x['frame']
            if tick!=frames:raise ValueError('burst log frames are not contiguous from zero')
            frames+=1;block=tick//1000;within=tick%1000;inburst=500<=within<550
            for inode,refs in x['ranges'].items():
                for e in refs:
                    kind,j=e['key'];key=(block,int(inode),kind,j);flag=bool(e.get('injected_outlier'))
                    if flag!=inburst:raise ValueError('input is not a completed burst measurement log')
                    if flag:quota[key]+=1;events+=1
                    elif (width==100 or (width==20 and 400<=within<600) or (width==40 and 300<=within<700)):
                        candidates[key].append(tick)
    frame_map=collections.defaultdict(list);groups=[]
    for key,q in sorted(quota.items()):
        block,i,kind,j=key;opts=candidates.get(key,[])
        def rank(t):return hashlib.sha256(f'matched-v1|{seed}|{width}|{block}|{i}|{kind}|{j}|{t}'.encode()).digest()
        selected=sorted(opts,key=lambda t:(rank(t),t))[:q]
        for tick in selected:frame_map[str(tick)].append([i,kind,j])
        groups.append({'block':block,'edge':[i,kind,j],'reference_events':q,'eligible_nonburst_reference_samples':len(opts),'scheduled_events':len(selected),'nominal_deficit':q-len(selected)})
    for entries in frame_map.values():entries.sort(key=lambda a:(a[0],a[1],a[2]))
    return {'schema':'fixed-per-edge-window-count-matched-v1','seed':seed,'window_s':width,'amplitude_m':3.0,
            'reference_run':str(reference_run if reference_run is not None else source.parent),
            'reference_estimator_sha256':filehash(source),'reference_frames':frames,'reference_burst_events':events,
            'rank_rule':'SHA256(matched-v1|seed|window_seconds|100s_block|robot|kind|reference|frame), ascending; eligible times outside original 5s burst',
            'selection_note':'eligible times obtained from saved burst-run availability, not future run; changed trajectory can make scheduled observations unavailable, which must be reported rather than filled post hoc',
            'frames':dict(frame_map),'groups':groups}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--burst-log',required=True,help='completed burst estimator.jsonl.gz')
    parser.add_argument('--seed',required=True,type=int)
    parser.add_argument('--window',required=True,type=int,choices=(20,40,100))
    parser.add_argument('--reference-run',help='original reference-run label; affects output SHA but not selected events')
    parser.add_argument('--output',required=True,help='new schedule JSON; existing files are not overwritten')
    args=parser.parse_args();data=build_schedule(args.burst_log,args.seed,args.window,args.reference_run)
    with Path(args.output).open('x') as f:json.dump(data,f,sort_keys=True,separators=(',',':'));f.write('\n')
    print(json.dumps({'output_sha256':filehash(args.output),'planned_events':sum(g['scheduled_events'] for g in data['groups']),
                      'nominal_deficit':sum(g['nominal_deficit'] for g in data['groups'])}))


if __name__=='__main__':main()
