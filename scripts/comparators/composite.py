"""Target-only cross comparison on the unchanged repaired joint robust CBF.

The native executable emits its original target soft rows before IPC. Replace
only those rows using the same documented native formulas and verify their
equivalence against saved native-applied target rows before freezing a run.
"""
import copy, math, time
import numpy as np
from legacy_controllers import Oruganti
from refined_goudar import Goudar
import cano_planner


def task_soft_rows(node, goal, cfg):
    softcfg=cfg['cbfs']['with-slack']
    assert not cfg['tvt-experiment'].get('near-observation-local',False)
    assert not softcfg['target-yaw']['on']
    p=np.asarray(node['position'],float);q=np.asarray(goal,float)
    direction=q-p;distance=float(np.sqrt(direction@direction))
    kp=softcfg['cvt']['kp']
    velocity=np.r_[kp*direction/distance if distance>1e-12 else np.zeros(2),0.]
    cconf=softcfg['cvt'];yconf=softcfg['cvt-yaw']
    def alpha(conf,h):return conf['alpha']['coe']*h**conf['alpha']['pow']
    state=np.r_[p,node['yaw']]
    def yaw_h(x):
        dx,dy=q-x[:2]
        desired=math.atan2(dy,dx)
        if math.sqrt(dx*dx+dy*dy)<=1e-12:desired=x[2]
        error=desired-x[2]
        error=math.atan2(math.sin(error),math.cos(error))
        return yconf['k_yaw']*(1.-math.cos(error))/(-2.)
    delta=.001
    yaw_gradient=[]
    for i in range(3):
        step=np.zeros(3);step[i]=delta
        yaw_gradient.append((yaw_h(state+step)-yaw_h(state-step))/2./delta)
    mapped={cconf['name']:{'name':cconf['name'],'coe':velocity.tolist(),'const':alpha(cconf,-kp*distance)},
            yconf['name']:{'name':yconf['name'],'coe':yaw_gradient,'const':alpha(yconf,yaw_h(state))}}
    assert {r['name'] for r in node['soft']}==set(mapped)
    return [mapped[r['name']] for r in node['soft']]


class CanoTarget:
    def __init__(self,cfg):self.cfg=cfg;self.previous=-1
    def step(self,request,cov):
        frame=request['frame'];assert frame==self.previous+1;self.previous=frame
        nodes=request['nodes'];leaders=tuple(i for i,n in enumerate(nodes) if n['leader'])
        edges=cano_planner.assigned_graph(nodes)
        result=cano_planner.plan([n['position'] for n in nodes],self.cfg['bases'],edges,leaders)
        targets=np.array(result['raw_targets_m'])
        for i in leaders:targets[i]=nodes[i]['goal']
        return {'frame':frame,'status':'success','targets':targets.tolist(),
                'diagnostics':{'replanned':True,'replan_period_s':.1,**result}}


class Composite:
    def __init__(self,cfg,parameters):
        self.cfg=cfg;self.method=parameters['target_method']
        self.target=CanoTarget(cfg) if self.method=='cano' else Goudar(cfg,parameters['target_parameters'])
        self.safety=Oruganti(cfg,{'solver':'clarabel'})
        boundary=np.asarray(cfg['world']['boundary'],float)
        self.lower=boundary.min(axis=0);self.upper=boundary.max(axis=0)
        assert len(boundary)==4 and all(np.all((p==self.lower)|(p==self.upper)) for p in boundary)
    def step(self,request,cov):
        start=time.perf_counter()
        planned=self.target.step(request,cov)
        if planned['status']!='success':
            return {'frame':request['frame'],'status':planned['status'],
                    'diagnostics':{'stage':'target','target':planned.get('diagnostics',{})}}
        target_s=time.perf_counter()-start
        raw=np.asarray(planned['targets'])
        applied=np.clip(raw,self.lower,self.upper) if self.method=='cano' else raw.copy()
        safety_request=copy.deepcopy(request)
        for node,q in zip(safety_request['nodes'],applied):
            node['soft']=task_soft_rows(node,q,self.cfg)
            node['goal']=q.tolist()
        safety_start=time.perf_counter()
        response=self.safety.step(safety_request,cov)
        safety_s=time.perf_counter()-safety_start
        response['diagnostics'].update(target_method=self.method,target=planned['diagnostics'],
            target_raw=raw.tolist(),target_applied=applied.tolist(),
            projected_by_same_native_rule=self.method=='cano',
            safety_soft_rows=[n['soft'] for n in safety_request['nodes']],
            target_wall_s=target_s,safety_wall_s=safety_s,total_module_wall_s=time.perf_counter()-start)
        assert 'targets' not in response
        return response
