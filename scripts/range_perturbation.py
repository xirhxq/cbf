"""Measurement perturbations after nominal range availability and noise."""
import hashlib,json
from pathlib import Path

class RangePerturbation:
    def __init__(self, case, schedule_path=None):
        self.kind=case['perturbation'];self.amplitude=float(case['amplitude_m']);self.matched=set();self.applied=0;self.schedule_sha256=None
        if self.kind=='matched':
            if not schedule_path:raise ValueError('matched case requires its historical frozen schedule')
            raw=Path(schedule_path).read_bytes()
            self.schedule_sha256=hashlib.sha256(raw).hexdigest();schedule=json.loads(raw)
            canonical=dict(schedule);canonical.pop('reference_run',None)
            canonical_sha=hashlib.sha256(json.dumps(canonical,sort_keys=True,separators=(',',':')).encode()).hexdigest()
            if self.schedule_sha256!=case['schedule_sha256'] and canonical_sha!=case['schedule_canonical_sha256']:
                raise ValueError('historical schedule SHA-256 mismatch')
            if schedule['seed']!=case['seed'] or schedule['window_s']!=case['window_s'] or schedule['amplitude_m']!=self.amplitude:raise ValueError('schedule metadata mismatch')
            self.matched={(int(t),int(e[0]),e[1],int(e[2])) for t,edges in schedule['frames'].items() for e in edges}
            if len(self.matched)!=sum(len(edges) for edges in schedule['frames'].values()):raise ValueError('duplicate scheduled event')
        elif schedule_path:raise ValueError('schedule supplied for a non-matched case')
    def apply(self, refs, tick):
        events=[]
        for robot,measurements in refs.items():
            for ref in measurements:
                kind,j=ref['key'];event=False
                if self.kind=='bias':ref['range']+=self.amplitude
                elif self.kind=='burst':event=tick>=500 and (tick-500)%1000<50
                elif self.kind=='matched':event=(tick,int(robot),kind,int(j)) in self.matched
                if event:
                    ref['range']+=self.amplitude;ref['injected_outlier']=True
                    events.append({'robot':int(robot),'key':[kind,int(j)]});self.applied+=1
        return events
