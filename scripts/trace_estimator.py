"""Frozen SCI implementation with trace radius and input-direction logging.

Known beacon ranges use ordinary scalar Kalman updates. UAV ranges use Cros
SCI with trace-optimal omega and no reference covariance inflation factor.
The existing 3-sigma innovation gate uses a conservative innovation variance
(sqrt(a)+sqrt(b))**2+r; gating remains an implementation choice, not part of
the SCI consistency theorem. All state and covariance updates are logged.
"""
import time
import numpy as np
from distance_sci import sci_update_matrices


class EKFInLoopService:
    def __init__(self, *, deployment_positions, bases, dt=.1,
                 process_noise_mps=1., p0_std=1., innovation_gate=3.):
        self.bases=[np.asarray(x,float) for x in bases]
        self.dt=dt
        self.process_noise_mps=process_noise_mps
        self.innovation_gate=innovation_gate
        self.state={i:{'mean':np.asarray(x,float).copy(),
                       'cov':np.eye(2)*p0_std**2,'updates':0,'rejected':0}
                    for i,x in sorted(deployment_positions.items())}

    def step(self, *, frame_index, raw_reference_groups, held_commands):
        outputs={}; self.measurement_audit={}; self.robot_update_s={}
        for i in sorted(self.state):
            started=time.perf_counter(); rows=[]
            mean=self.state[i]['mean']+np.asarray(held_commands[i])*self.dt
            cov=self.state[i]['cov']+np.eye(2)*(self.process_noise_mps*self.dt)**2
            updates=rejected=unused=0
            for ref in raw_reference_groups.get(i,[]):
                kind,j=ref['key']
                if kind=='base':
                    anchor=self.bases[j]; anchor_cov=np.zeros((2,2))
                elif kind=='uav':
                    assert j<i, 'Current-frame upstream schedule requires lower-index references'
                    anchor=self.state[j]['mean']; anchor_cov=self.state[j]['cov']
                else:raise ValueError(kind)
                delta=mean-anchor; distance=float(np.linalg.norm(delta))
                row={'key':list(ref['key'])}
                if distance<1e-6:
                    row['decision']='skipped-degenerate'; rows.append(row); continue
                u=delta/distance
                innovation=float(ref['range'])-distance
                a=float(u@cov@u); b=float(u@anchor_cov@u)
                r=float(ref['ranging_sigma'])**2
                gate_S=(np.sqrt(a)+np.sqrt(b))**2+r
                row.update({'a':a,'b':b,'r':r,'innovation':innovation,'gate_S':gate_S,
                            'direction':u.tolist()})
                if abs(innovation)>self.innovation_gate*np.sqrt(gate_S):
                    rejected+=1; row['decision']='rejected'; rows.append(row); continue
                gain, posterior, omega=sci_update_matrices(cov,b,u,r)
                row.update({'omega':omega,'gain':gain.tolist(),
                            'trace_before':float(np.trace(cov)),
                            'trace_after':float(np.trace(posterior))})
                if omega==0.:
                    unused+=1; row['decision']='unused-omega-zero'; rows.append(row); continue
                mean=mean+gain*innovation; cov=posterior; updates+=1
                row['decision']='accepted'; rows.append(row)
            eigen=np.linalg.eigvalsh(cov)
            if eigen[0]<=0 or not np.all(np.isfinite(mean)):
                raise ArithmeticError('SCI state or covariance invalid')
            self.state[i]={'mean':mean,'cov':cov,'updates':updates,'rejected':rejected}
            self.measurement_audit[i]=rows
            self.robot_update_s[i]=time.perf_counter()-started
            outputs[i]={'estimate':mean.tolist(),'epsilon':3.*float(np.sqrt(np.trace(cov))),
                        'epsilon_lambda_max':3.*float(np.sqrt(eigen[-1])),
                        'tier':'fresh' if updates else 'coast','updates':updates,
                        'rejected':rejected,'unused_zero_weight':unused}
        return outputs
