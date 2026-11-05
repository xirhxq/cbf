"""Cano-inspired follower waypoint planner: centralized joint FIM, synthetic prototype.

Only externally supplied measured graph edges contribute to FIM/connectivity.
IDs in `edges` index mobiles then anchors, all undirected edges listed once.
Leader positions are frozen by plan(), while they remain FIM unknowns.
"""
from __future__ import annotations
import math,time
import numpy as np

# Frozen engineering scenario mapping, see plan.md; no empirical tuning.
RANGE_M=850.0
SCALE_M=RANGE_M/75.0
SIGMA_M=0.5
DA_NORM=2.0
DC_NORM=50.0
DELTA_NORM=2.0
KL=50000.0
KC=1000.0
KA=1000.0
ITERATIONS=30
PERIOD_HOLDS=50

class InvalidGeometry(ValueError):
    def __init__(self, message, diagnostics=None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}

def bounded_step(g, delta=DELTA_NORM):
    """Overflow-safe norm for Eq.43; undefined input must fail closed."""
    x,y=map(float,g)
    if not math.isfinite(x) or not math.isfinite(y):raise InvalidGeometry('nonfinite gradient')
    norm=math.hypot(x,y)
    if not math.isfinite(norm):raise InvalidGeometry('gradient norm unrepresentable')
    if norm==0:return np.zeros(2)
    if norm<=delta:return np.array([-x,-y])
    return np.array([-(x/norm)*delta,-(y/norm)*delta])

def pairnorm(values):
    result=np.hypot(values[:,0],values[:,1])
    if not np.isfinite(result).all():raise InvalidGeometry('vector norm unrepresentable')
    return result

def assigned_graph(robots):
    """Known design graph only; no observation is made or imputed."""
    if len(robots)!=14:raise InvalidGeometry('not the frozen 14-mobile case')
    edges=set();ids=set()
    for r in robots:
        i=int(r['id'])-1;ids.add(i);formation=r['formation']
        if formation['id']!=i+1:raise InvalidGeometry('formation identity mismatch')
        for j in formation['baseIds']:
            j=int(j)
            if not 0<=j<3:raise InvalidGeometry('unknown base index')
            edges.add((i,14+j))
        for j in formation['anchorIds']:
            j=int(j)-1
            if not 0<=j<14 or j==i:raise InvalidGeometry('invalid mobile neighbor')
            edges.add(tuple(sorted((i,j))))
    if ids!=set(range(14)) or len(edges)!=28 or any(a in (6,13) for a,b in edges if b>=14):
        raise InvalidGeometry('assigned topology differs from frozen design')
    return sorted(edges)

def potential_grad(mobile, anchors, edges):
    """Evaluate original potentials/gradients in normalized coordinates.

    J_con's directed summation gives weight 1 for a mobile-mobile edge and
    1/2 for mobile-anchor.  J_avd's directed all-pairs has same convention.
    """
    mobile=np.asarray(mobile,dtype=np.float64); anchors=np.asarray(anchors,dtype=np.float64)
    if mobile.ndim!=2 or mobile.shape[1]!=2 or anchors.ndim!=2 or anchors.shape[1]!=2 or not np.isfinite(mobile).all() or not np.isfinite(anchors).all():
        raise InvalidGeometry('bad coordinate array')
    n=len(mobile); vertices=np.vstack((mobile,anchors)); rankdim=2*n
    if n<1 or len(anchors)<1: raise InvalidGeometry('missing tags or anchors')
    unique=set()
    for ia,ib in edges:
        if not (0<=ia<len(vertices) and 0<=ib<len(vertices) and ia!=ib) or (ia>=n and ib>=n): raise InvalidGeometry('bad graph edge')
        edge=tuple(sorted((int(ia),int(ib))))
        if edge in unique: raise InvalidGeometry('duplicate graph edge')
        unique.add(edge)
    sigma_norm=SIGMA_M/SCALE_M; w=1.0/(sigma_norm*sigma_norm)
    fim=np.zeros((rankdim,rankdim)); ee=[]
    for a,b in sorted(unique):
        v=vertices[a]-vertices[b]; d=float(np.linalg.norm(v))
        if not np.isfinite(d) or d<=0:raise InvalidGeometry('coincident ranging pair')
        u=v/d; block=w*np.outer(u,u); ia=slice(2*a,2*a+2)
        if a<n:fim[ia,ia]+=block
        if b<n:
            ib=slice(2*b,2*b+2);fim[ib,ib]+=block
            if a<n:fim[ia,ib]-=block;fim[ib,ia]-=block
        ee.append((a,b,d,u))
    eig=np.linalg.eigvalsh(fim); lo=float(eig[0]);hi=float(eig[-1]);tol=rankdim*np.finfo(np.float64).eps*max(0.,hi)
    if not np.isfinite(eig).all() or hi<=0 or lo<=tol:
        raise InvalidGeometry('FIM singular/numerically unresolved', {'lambda_min':lo,'lambda_max':hi,'rank_tol':tol,'rank':int(np.count_nonzero(eig>tol)),'dimension':rankdim})
    try:
        L=np.linalg.cholesky(fim);inv=np.linalg.solve(L.T,np.linalg.solve(L,np.eye(rankdim)))
    except np.linalg.LinAlgError as e:raise InvalidGeometry('FIM Cholesky failure') from e
    resid=float(np.linalg.norm(fim@inv-np.eye(rankdim),ord=np.inf)/max(1.,np.linalg.norm(fim,ord=np.inf)*np.linalg.norm(inv,ord=np.inf)))
    if not np.isfinite(resid) or resid>1e-12:raise InvalidGeometry(f'FIM inverse residual {resid}')
    JD=-2*float(np.log(np.diag(L)).sum());gd=np.zeros_like(mobile);gc=np.zeros_like(mobile);ga=np.zeros_like(mobile);jc=ja=0.
    for a,b,d,u in ee:
        A=np.zeros((2,2));ia=slice(2*a,2*a+2)
        if a<n:A+=inv[ia,ia]
        if b<n:
            ib=slice(2*b,2*b+2);A+=inv[ib,ib]
            if a<n:A-=inv[ia,ib]+inv[ib,ia]
        Au=A@u; dg=-(2*w/d)*(Au-u*float(u@Au))
        if a<n:gd[a]+=dg
        if b<n:gd[b]-=dg
        if d>DC_NORM:
            weight=1.0 if a<n and b<n else 0.5
            jc+=weight*(d-DC_NORM)**2
            v=2*weight*(d-DC_NORM)*u
            if a<n:gc[a]+=v
            if b<n:gc[b]-=v
    # Every mobile/mobile and mobile/anchor pair, not extra ranging edges.
    for a in range(n):
        for b in range(a+1,len(vertices)):
            v=vertices[a]-vertices[b];d=float(np.linalg.norm(v))
            if not np.isfinite(d) or d<=0:raise InvalidGeometry('coincident avoidance pair')
            if d<DA_NORM:
                weight=1.0 if b<n else 0.5
                dv=1/d-1/DA_NORM;ja+=weight*dv*dv
                g=(-2*weight*dv/(d*d))*(v/d);ga[a]+=g
                if b<n:ga[b]-=g
    g=KL*gd+KC*gc+KA*ga; J=KL*JD+KC*jc+KA*ja
    if not np.isfinite(g).all() or not np.isfinite(J):raise InvalidGeometry('nonfinite potential or gradient')
    return {'J':J,'J_D':JD,'J_con':jc,'J_avd':ja,'gradient':g,'gradient_D':gd,'gradient_con':gc,'gradient_avd':ga,'fim':fim,'rank':rankdim,'lambda_min':lo,'lambda_max':hi,'rank_tol':tol,'inverse_relative_residual':resid}

def plan(mobile_m, anchors_m, edges, leaders=(6,13), bounds=(-1500.,1500.,-1500.,1500.)):
    """30 simultaneous follower steps in virtual coordinates; no physical action."""
    start=time.perf_counter(); q=np.asarray(mobile_m,dtype=np.float64).copy()/SCALE_M; anchor=np.asarray(anchors_m,dtype=np.float64)/SCALE_M
    leaders=set(leaders)
    if any(i<0 or i>=len(q) for i in leaders):raise InvalidGeometry('bad leader index')
    trace=[]
    for k in range(ITERATIONS):
        t=time.perf_counter()
        try:e=potential_grad(q,anchor,edges)
        except InvalidGeometry as exc:
            exc.diagnostics.update({'iteration':k,'completed_iterations':trace,'elapsed_s':time.perf_counter()-start})
            raise
        g=e['gradient'];step=np.zeros_like(q)
        for i in range(len(q)):
            if i in leaders:continue
            try:step[i]=bounded_step(g[i])
            except InvalidGeometry as exc:
                exc.diagnostics.update({'iteration':k,'mobile_index':i,'completed_iterations':trace,'elapsed_s':time.perf_counter()-start})
                raise
        q+=step
        if not np.isfinite(q).all():raise InvalidGeometry('virtual position nonfinite',{'iteration':k,'completed_iterations':trace})
        trace.append({'iteration':k,'J':e['J'],'J_D':e['J_D'],'J_con':e['J_con'],'J_avd':e['J_avd'],'lambda_min':e['lambda_min'],'lambda_max':e['lambda_max'],'rank':e['rank'],'rank_tol':e['rank_tol'],'inverse_relative_residual':e['inverse_relative_residual'],'max_step_m':float(np.max(pairnorm(step))*SCALE_M),'seconds':time.perf_counter()-t})
    try:final=potential_grad(q,anchor,edges)
    except InvalidGeometry as exc:
        exc.diagnostics.update({'iteration':ITERATIONS,'completed_iterations':trace,'elapsed_s':time.perf_counter()-start})
        raise
    raw=q*SCALE_M; projected=raw.copy();xmin,xmax,ymin,ymax=bounds
    for i in range(len(q)):
        if i not in leaders:projected[i]=np.clip(raw[i],[xmin,ymin],[xmax,ymax])
    if not np.isfinite(projected).all():raise InvalidGeometry('nonfinite final target')
    return {'raw_targets_m':raw.tolist(),'projected_targets_m':projected.tolist(),'projection_distance_m':pairnorm(raw-projected).tolist(),'target_displacement_from_current_m':pairnorm(projected-np.asarray(mobile_m)).tolist(),'projection_kind':'python_rectangular_preview_not_native','trace':trace,'final_fim':{'rank':final['rank'],'lambda_min':final['lambda_min'],'lambda_max':final['lambda_max'],'rank_tol':final['rank_tol'],'inverse_relative_residual':final['inverse_relative_residual']},'wall_seconds':time.perf_counter()-start}

class PeriodicFollowerState:
    """Explicit 0,10,20,... trigger and fail-closed cached waypoint state."""
    def __init__(self, leaders=(6,13)):
        self.leaders=tuple(leaders);self.previous_frame=-1;self.current=None;self.current_raw=None
    def snapshot(self):
        if self.current is None or self.current_raw is None or self.previous_frame<0:
            raise InvalidGeometry('no initialized follower state to save')
        return {'previous_frame':self.previous_frame,'leaders':list(self.leaders),'current_raw_m':self.current_raw.tolist(),'current_preview_m':self.current.tolist()}
    def restore(self, data):
        if self.previous_frame!=-1 or self.current is not None:raise InvalidGeometry('follower state already initialized')
        if tuple(data['leaders'])!=self.leaders:raise InvalidGeometry('leader set differs from saved state')
        frame=data['previous_frame'];raw=np.asarray(data['current_raw_m'],dtype=np.float64);preview=np.asarray(data['current_preview_m'],dtype=np.float64)
        if type(frame) is not int or frame<0 or raw.shape!=(14,2) or preview.shape!=(14,2) or not np.isfinite(raw).all() or not np.isfinite(preview).all():
            raise InvalidGeometry('invalid follower state snapshot')
        self.previous_frame=frame;self.current_raw=raw.copy();self.current=preview.copy()
    def advance(self, frame, mobile_m, anchors_m, edges):
        if frame!=self.previous_frame+1:raise InvalidGeometry('nonsequential estimator frames')
        if frame%PERIOD_HOLDS==0:
            try:r=plan(mobile_m,anchors_m,edges,self.leaders)
            except InvalidGeometry as exc:
                exc.diagnostics.update({'frame':frame,'status':'planner_failed_no_fallback'})
                raise
            targets=np.asarray(r['projected_targets_m'])
            if self.current is None:r['preview_target_jump_from_previous_m']=None
            else:r['preview_target_jump_from_previous_m']=pairnorm(targets-self.current).tolist()
            r['frame']=frame;r['trigger']='fixed_period';self.current=targets;self.current_raw=np.asarray(r['raw_targets_m'])
        else:r=None
        if self.current is None:raise InvalidGeometry('missing follower targets')
        self.previous_frame=frame
        return self.current_raw.copy(),r
