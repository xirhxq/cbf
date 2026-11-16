from pathlib import Path
import json
import subprocess
import time
import numpy as np
from scipy.linalg import block_diag, solve_triangular
from scipy.optimize import least_squares, minimize

HERE=Path(__file__).resolve().parent

def graph_from_request(request,bases):
    links=set();anchors=[]
    for i,node in enumerate(request['nodes']):
        for j in node['formation']['anchorIds']:links.add(tuple(sorted((i,j-1))))
        for b in node['formation']['baseIds']:anchors.append((i,int(b)))
    return sorted(links),sorted(anchors)

def oruganti_rows(request,cfg):
    nodes=request['nodes'];p=np.array([n['position'] for n in nodes]);e=np.array([n['epsilon'] for n in nodes])
    U=float(cfg['cbfs']['input-limits']['planar-component-max']);T=cfg['execute']['time-step']
    D=cfg['cbfs']['without-slack']['safety']['safe-distance'];R=cfg['cbfs']['without-slack']['comm-fixed']['max-range']
    ac=cfg['tvt-experiment']['squared-alpha'];al=cfg['tvt-experiment']['work2-link-alpha'];links,anchors=graph_from_request(request,cfg['bases'])
    rows=[]
    def append(i,j,kind,base=None):
        delta=p[i]-(p[j] if j>=0 else np.array(cfg['bases'][base]));s=np.linalg.norm(delta)
        error=e[i]+(e[j] if j>=0 else 0.);rho=error+T*np.sqrt(2)*U*(2 if j>=0 else 1);M=s+rho
        sign=1. if kind=='collision' else -1.;alpha=ac if kind=='collision' else al
        h=s*s-D*D if kind=='collision' else R*R-s*s
        rows.append({'i':i,'j':j,'base':base,'kind':kind,'a':(sign*2*delta).tolist(),
                     'c':float(alpha*h-2*alpha*M*rho),'norm_loss':float(2*rho),'rho':float(rho),'s':float(s),'error':float(error)})
    for i in range(len(p)):
        for j in range(i):append(i,j,'collision')
    for i,j in links:append(i,j,'link')
    for i,b in anchors:append(i,-1,'link',b)
    return rows

class Oruganti:
    def __init__(self,cfg,parameters):
        self.cfg=cfg
        self.backend=parameters.get('solver','clarabel');self.process=None
        if self.backend=='gurobi':self.process=subprocess.Popen([str(HERE/'conic_solver')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.stdin.close();self.process.wait(timeout=10)
    def step(self,request,cov):
        rows=oruganti_rows(request,self.cfg)
        problem={'nodes':request['nodes'],'hard':rows,'U':self.cfg['cbfs']['input-limits']['planar-component-max'],
                 'omega_max':self.cfg['cbfs']['input-limits']['yaw-rate-max'],'k_delta':self.cfg['cbfs']['objective-function']['k_delta']}
        if self.backend=='gurobi':
            self.process.stdin.write(json.dumps(problem)+'\n');self.process.stdin.flush()
            line=self.process.stdout.readline()
            if not line:raise RuntimeError('conic solver exited: '+self.process.stderr.read())
            result=json.loads(line)
        else:
            from conic_backend import solve_conic
            result=solve_conic(problem)
        commands=result.pop('commands',None)
        return {'frame':request['frame'],'status':result['status'],'commands':commands,
                'diagnostics':result,'hard_rows':rows}

class Goudar:
    def __init__(self,cfg,parameters):
        self.cfg=cfg;self.parameters=parameters;self.H=parameters['horizon_steps'];self.dt=cfg['execute']['time-step']
        self.Phi=np.block([[np.eye(2),self.dt*np.eye(2)],[np.zeros((2,2)),np.eye(2)]])
        d=self.dt;qc=parameters['acceleration_psd']
        self.Qgp=np.kron(np.array([[d**3/3,d*d/2],[d*d/2,d]]),qc*np.eye(2))
        self.Lgp=solve_triangular(np.linalg.cholesky(self.Qgp),np.eye(4),lower=True)
        self.deployment=np.array(cfg['initial']['position']['positions'],float);self.previous={}
        self.neighbors={i:[] for i in range(len(self.deployment))}
        for offset in (0,7):
            for i in range(offset,offset+7):
                for j in (i-1,i-2):
                    if j>=offset:
                        distance=np.linalg.norm(self.deployment[i]-self.deployment[j]) if parameters['edge_length_m']=='deployment' else float(parameters['edge_length_m'])
                        self.neighbors[i].append((j,distance));self.neighbors[j].append((i,distance))
    def residual_jacobian(self,x,prior,L0,neighbors):
        H=self.H;states=x.reshape(H+1,4);res=[L0@(states[0]-prior)]
        J=np.zeros((4+4*H+len(neighbors)*(H+1),4*(H+1)))
        J[:4,:4]=L0;row=4
        for k in range(H):
            res.append(self.Lgp@(states[k+1]-self.Phi@states[k]));J[row:row+4,4*k:4*k+4]=-self.Lgp@self.Phi;J[row:row+4,4*k+4:4*k+8]=self.Lgp;row+=4
        for position,distance,sd in neighbors:
            delta=states[:,:2]-position;norm=np.linalg.norm(delta,axis=1)
            res.append((norm-distance)/sd)
            unit=delta/np.maximum(norm[:,None],1e-12)/sd
            for k in range(H+1):J[row+k,4*k:4*k+2]=unit[k]
            row+=H+1
        return np.concatenate(res),J
    def step(self,request,cov):
        started=time.perf_counter();nodes=request['nodes'];targets=np.array([n['goal'] for n in nodes],float);stats=[]
        for i,node in enumerate(nodes):
            if node['leader']:continue
            prior=np.r_[node['position'],node['held_command'][:2]]
            covariance=block_diag(cov[i],np.eye(2)*self.parameters['velocity_prior_variance'])
            L0=solve_triangular(np.linalg.cholesky(covariance),np.eye(4),lower=True)
            neighbors=[(np.array(nodes[j]['position']),d,np.sqrt(max(np.trace(cov[j]),self.parameters['covariance_floor']))) for j,d in self.neighbors[i]]
            if i in self.previous:
                initial=self.previous[i].copy();initial=np.vstack([initial[1:],self.Phi@initial[-1]])
            else:initial=np.array([np.r_[prior[:2]+k*self.dt*prior[2:],prior[2:]] for k in range(self.H+1)])
            function=lambda x:self.residual_jacobian(x,prior,L0,neighbors)
            r0=function(initial.ravel())[0]
            result=least_squares(lambda x:function(x)[0],initial.ravel(),jac=lambda x:function(x)[1],method='lm',max_nfev=self.parameters['max_nfev'],ftol=1e-7,xtol=1e-7,gtol=1e-7)
            stats.append({'id':i+1,'status':int(result.status),'nfev':result.nfev,'cost':float(result.cost),'initial_cost':float(.5*r0@r0),'optimality':float(result.optimality)})
            if not result.success or not np.all(np.isfinite(result.x)):
                return {'frame':request['frame'],'status':'formation-solver-failure','diagnostics':{'nodes':stats,'message':result.message}}
            states=result.x.reshape(self.H+1,4);self.previous[i]=states;targets[i]=states[-1,:2]
        return {'frame':request['frame'],'status':'success','targets':targets.tolist(),
                'diagnostics':{'nodes':stats,'wall_s':time.perf_counter()-started,'edge_length_m':self.parameters['edge_length_m']}}

def segment_minimum(start,end):
    delta=end-start;den=np.sum(delta*delta,axis=-1)
    tau=np.clip(-np.sum(start*delta,axis=-1)/np.maximum(den,1e-30),0.,1.)
    closest=start+tau[...,None]*delta
    return np.linalg.norm(closest,axis=-1),tau,closest

class Nikou:
    def __init__(self,cfg,parameters):
        self.cfg=cfg;self.p=parameters;self.dt=cfg['execute']['time-step'];self.H=parameters['horizon_steps']
        self.n=cfg['num'];self.k=parameters['ancillary_gain'];self.kf=parameters['terminal_gain']
        self.Ebar=parameters['error_bound_m'];self.zeta=self.Ebar+parameters['physical_disturbance_mps']/self.k
        self.eps=parameters['terminal_radius_m'];self.U=cfg['cbfs']['input-limits']['planar-component-max']
        self.Ubar=self.U-self.k*(self.zeta+self.Ebar)
        self.D=cfg['cbfs']['without-slack']['safety']['safe-distance'];self.R=cfg['cbfs']['without-slack']['comm-fixed']['max-range']
        self.qweight=parameters['state_weight'];self.rweight=parameters['input_weight']
        self.S=self.dt*(self.qweight+self.kf**2*self.rweight)/(1-(1-self.kf*self.dt)**2)
        self.A=self.dt*np.tril(np.ones((self.H+1,self.H)),k=-1)
        self.z=None;self.reference=None;self.plan=None;self.links=None;self.anchors=None
        assert 0<self.k*self.dt<=1 and 0<self.kf*self.dt<=1 and self.Ubar>0 and self.kf*self.eps<=self.Ubar
    def paths(self,z,plan):return z[:,None,:]+np.einsum('lh,nhd->nld',self.A,plan)
    def terminal_safe(self,r):
        dist=np.linalg.norm(r[:,None]-r[None,:],axis=-1);np.fill_diagonal(dist,np.inf)
        if np.min(dist)<self.D+2*self.zeta+2*self.eps-1e-8:return False
        if any(np.linalg.norm(r[i]-r[j])>self.R-2*self.zeta-2*self.eps+1e-8 for i,j in self.links):return False
        if any(np.linalg.norm(r[i]-np.array(self.cfg['bases'][b]))>self.R-self.zeta-self.eps+1e-8 for i,b in self.anchors):return False
        return True
    def local_functions(self,i,z,neighbors,reference):
        H=self.H;A=self.A
        def state(x):return z+A@x.reshape(H,2)
        def objective(x):
            v=x.reshape(H,2);p=state(x);error=p-reference
            value=self.dt*(self.qweight*np.sum(error[:-1]**2)+self.rweight*np.sum(v*v))+self.S*np.sum(error[-1]**2)
            grad=2*self.dt*self.qweight*A[:-1].T@error[:-1]+2*self.dt*self.rweight*v+2*self.S*A[-1,:,None]*error[-1]
            return value/1000.,grad.ravel()/1000.
        def constraints(x):
            p=state(x);values=[];jac=[]
            for peer,is_link,radius,is_collision in neighbors:
                difference=p-peer
                if is_collision:
                    distance,tau,closest=segment_minimum(difference[:-1],difference[1:])
                    derivative=closest/np.maximum(distance[:,None],1e-12)
                    coeff=(1-tau[:,None])*A[:-1]+tau[:,None]*A[1:]
                    values.append(distance-(self.D+radius))
                    jac.append((coeff[:,:,None]*derivative[:,None,:]).reshape(H,2*H))
                if is_link:
                    distance=np.linalg.norm(difference,axis=1);derivative=-difference/np.maximum(distance[:,None],1e-12)
                    values.append(self.R-radius-distance)
                    jac.append((A[:,:,None]*derivative[:,None,:]).reshape(H+1,2*H))
            delta=p[-1]-reference;distance=np.linalg.norm(delta)
            values.append(np.array([self.eps-distance]));jac.append((-A[-1,:,None]*delta/ max(distance,1e-12)).reshape(1,2*H))
            return np.concatenate(values),np.vstack(jac)
        return objective,constraints
    def validate_plan(self,plan,reference):
        paths=self.paths(self.z,plan);minimum=1e300
        for i in range(self.n):
            for j in range(i):
                distances,_,_=segment_minimum(paths[i,:-1]-paths[j,:-1],paths[i,1:]-paths[j,1:])
                minimum=min(minimum,float(np.min(distances)-self.D-2*self.zeta))
        for i,j in self.links:minimum=min(minimum,float(self.R-2*self.zeta-np.max(np.linalg.norm(paths[i]-paths[j],axis=1))))
        for i,b in self.anchors:minimum=min(minimum,float(self.R-self.zeta-np.max(np.linalg.norm(paths[i]-np.array(self.cfg['bases'][b]),axis=1))))
        minimum=min(minimum,float(self.eps-np.max(np.linalg.norm(paths[:,-1]-reference,axis=1))),float(self.Ubar-np.max(np.abs(plan))))
        return minimum
    def solve_candidate(self,reference):
        plan=self.plan.copy();stats=[];linkset=set(self.links)
        for i in range(self.n):
            paths=self.paths(self.z,plan);neighbors=[]
            for j in range(self.n):
                if j!=i:neighbors.append((paths[j],tuple(sorted((i,j))) in linkset,2*self.zeta,True))
            for node,b in self.anchors:
                if node==i:neighbors.append((np.tile(self.cfg['bases'][b],(self.H+1,1)),True,self.zeta,False))
            objective,constraints=self.local_functions(i,self.z[i],neighbors,reference[i])
            result=minimize(lambda x:objective(x),plan[i].ravel(),method='SLSQP',jac=True,
                            bounds=[(-self.Ubar,self.Ubar)]*(2*self.H),
                            constraints={'type':'ineq','fun':lambda x:constraints(x)[0],'jac':lambda x:constraints(x)[1]},
                            options={'maxiter':self.p['maxiter'],'ftol':1e-7,'disp':False})
            margin=float(np.min(constraints(result.x)[0]));stats.append({'id':i+1,'status':int(result.status),'iterations':int(result.nit),'margin_m':margin})
            if not result.success or margin < -1e-5 or not np.all(np.isfinite(result.x)):return None,stats
            plan[i]=result.x.reshape(self.H,2)
        return (plan if self.validate_plan(plan,reference)>=-1e-5 else None),stats
    def step(self,request,cov):
        started=time.perf_counter();positions=np.array([n['position'] for n in request['nodes']]);scales=np.array([n['epsilon'] for n in request['nodes']]);goals=np.array([n['goal'] for n in request['nodes']])
        if self.z is None:
            self.links,self.anchors=graph_from_request(request,self.cfg['bases']);self.z=positions.copy();self.reference=positions.copy();self.plan=np.zeros((self.n,self.H,2))
            if not self.terminal_safe(self.reference):return {'frame':request['frame'],'status':'initial-terminal-set-infeasible','diagnostics':{}}
        if np.max(scales)>self.Ebar:
            return {'frame':request['frame'],'status':'declared-estimator-bound-exceeded','diagnostics':{'max_scale':float(np.max(scales)),'bound':self.Ebar}}
        if np.max(np.linalg.norm(positions-self.z,axis=1))>self.zeta+self.Ebar+1e-5:
            return {'frame':request['frame'],'status':'tube-information-consistency-failure','diagnostics':{}}
        attempts=[];accepted_lambda=0.;updated=False
        if request['frame']%self.p['replan_every_steps']==0:
            for alpha in self.p['reference_fractions']:
                candidate=self.reference+alpha*(goals-self.reference)
                if not self.terminal_safe(candidate):attempts.append({'lambda':alpha,'status':'terminal-geometry-rejected'});continue
                if np.max(np.linalg.norm(candidate-self.z,axis=1))>self.H*self.dt*np.sqrt(2)*self.Ubar+self.eps:
                    attempts.append({'lambda':alpha,'status':'horizon-reachability-rejected'});continue
                plan,stats=self.solve_candidate(candidate);attempts.append({'lambda':alpha,'status':'accepted' if plan is not None else 'candidate-solver-rejected','nodes':stats})
                if plan is not None:self.plan=plan;self.reference=candidate;accepted_lambda=alpha;updated=True;break
        margin=self.validate_plan(self.plan,self.reference)
        if margin < -1e-5:return {'frame':request['frame'],'status':'backup-plan-infeasible','diagnostics':{'margin_m':margin,'attempts':attempts}}
        nominal=self.plan[:,0].copy();command=nominal-self.k*(positions-self.z)
        if np.max(np.abs(command))>self.U+1e-7:return {'frame':request['frame'],'status':'actual-input-bound-failure','diagnostics':{}}
        old_end=self.paths(self.z,self.plan)[:,-1]
        self.z=self.z+self.dt*nominal
        tail=-self.kf*(old_end-self.reference)
        self.plan=np.concatenate([self.plan[:,1:],tail[:,None,:]],axis=1)
        return {'frame':request['frame'],'status':'success','commands':np.c_[command,np.zeros(self.n)].tolist(),'targets':self.reference.tolist(),
                'diagnostics':{'updated':updated,'accepted_lambda':accepted_lambda,'attempts':attempts,'plan_margin_m':margin,
                               'zeta_m':self.zeta,'nominal_position':self.z.tolist(),'nominal_command':nominal.tolist(),
                               'wall_s':time.perf_counter()-started,'prediction_payload_bytes':int(self.plan.nbytes)}}

def make_controller(method,cfg,parameters):
    return {'oruganti':Oruganti,'goudar':Goudar,'nikou':Nikou}[method](cfg,parameters)
