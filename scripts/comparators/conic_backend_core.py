from pathlib import Path
from types import SimpleNamespace
import time
import clarabel
import numpy as np
from scipy import sparse

ADMISSION_POLICY = {
    'version': 'physical-kkt-gap-v2-dimensionless', 'candidate_statuses': ['Solved', 'AlmostSolved'],
    'hard_absolute': 1e-4, 'hard_normalized': 1e-8, 'soft_absolute': 1e-5,
    'slack_absolute': 1e-7, 'input_absolute': 1e-7,
    'primal_residual': 1e-8, 'dual_residual': 1e-8,
    'stationarity_relative': 1e-8, 'dual_cone_relative': 1e-8,
    'duality_gap_relative': 1e-8, 'objective_agreement_relative': 1e-8,
    'canonical_primal_relative': 1e-8
}

NUMERICAL_SETTINGS = {
    'tol_gap_abs':1e-10, 'tol_gap_rel':1e-12, 'tol_feas':1e-12,
    'static_regularization_constant':1e-10,
    'dynamic_regularization_eps':1e-15, 'dynamic_regularization_delta':1e-9,
    'iterative_refinement_reltol':1e-14, 'iterative_refinement_abstol':1e-14,
    'iterative_refinement_max_iter':20,
}

def scaled_problem(P,q,A,b,linear_count,request):
    """x=D y, cone scaling R (one scalar per SOC), objective divided by gamma."""
    n=len(request['nodes']);D=np.ones(len(q));D[:3*n]=np.tile([request['U'],request['U'],request['omega_max']],n)
    D[3*n:]=max(1.,request['U']**2/request['k_delta']);gamma=request['U']**2
    AD=A@sparse.diags(D)
    rowmax=np.asarray(abs(AD).max(axis=1).toarray()).ravel()
    R=1./np.maximum(1.,np.maximum(np.abs(b),rowmax))
    for start in range(linear_count,len(b),3):R[start:start+3]=np.min(R[start:start+3])
    return (sparse.csc_matrix(sparse.diags(D)@P@sparse.diags(D)/gamma),q*D/gamma,
            sparse.csc_matrix(sparse.diags(R)@AD),b*R,D,R,gamma)

def solve_conic(request):
    start=time.perf_counter();nodes=request['nodes'];n=len(nodes);m=sum(len(p['soft']) for p in nodes);size=3*n+m
    P=sparse.diags(np.r_[np.full(3*n,2.),np.zeros(m)],format='csc');q=np.r_[np.zeros(3*n),np.full(m,request['k_delta'])]
    blocks=[];bs=[];cones=[];linear=[];bounds=[]
    for i in range(n):
        for axis in range(3):
            bound=request['omega_max'] if axis==2 else request['U']
            for sign in (-1.,1.):
                row=np.zeros(size);row[3*i+axis]=sign;linear.append(row);bounds.append(bound)
    index=3*n
    for i,node in enumerate(nodes):
        for soft in node['soft']:
            row=np.zeros(size);row[3*i:3*i+3]=-np.array(soft['coe']);row[index]=-1.;linear.append(row);bounds.append(soft['const'])
            row=np.zeros(size);row[index]=-1.;linear.append(row);bounds.append(0.);index+=1
    blocks.append(sparse.csc_matrix(np.array(linear)));bs.append(np.array(bounds));cones.append(clarabel.NonnegativeConeT(len(linear)))
    for hard in request['hard']:
        i,j=hard['i'],hard['j'];difference=np.zeros((2,size));difference[0,3*i]=1.;difference[1,3*i+1]=1.
        if j>=0:difference[0,3*j]=-1.;difference[1,3*j+1]=-1.
        loss=hard['norm_loss'];matrix=np.zeros((3,size));matrix[0]=-np.array(hard['a'])@difference/loss;matrix[1:]=-difference
        blocks.append(sparse.csc_matrix(matrix));bs.append(np.array([hard['c']/loss,0.,0.]));cones.append(clarabel.SecondOrderConeT(3))
    A=sparse.vstack(blocks,format='csc');b=np.concatenate(bs)
    settings=clarabel.DefaultSettings();settings.verbose=False;settings.max_iter=200;settings.time_limit=30.
    for key,value in NUMERICAL_SETTINGS.items():setattr(settings,key,value)
    Ps,qs,As,bs,D,R,gamma=scaled_problem(P,q,A,b,len(linear),request)
    scaled=clarabel.DefaultSolver(Ps,qs,As,bs,cones,settings).solve()
    # Map both primal and dual variables back before applying the unchanged physical thresholds.
    result=SimpleNamespace(status=scaled.status,iterations=scaled.iterations,
        x=(np.asarray(scaled.x)*D).tolist(),z=(np.asarray(scaled.z)*R*gamma).tolist(),s=(np.asarray(scaled.s)/R).tolist(),
        obj_val=scaled.obj_val*gamma,obj_val_dual=scaled.obj_val_dual*gamma,r_prim=scaled.r_prim,r_dual=scaled.r_dual)
    output=assess_solution(request,result,P,q,A,b,len(linear))
    output['numerical_settings']=NUMERICAL_SETTINGS
    output['scaling']={'variable':D.tolist(),'cone_rows':R.tolist(),'objective_divisor':gamma}
    output['wall_s']=time.perf_counter()-start
    return output

def assess_solution(request,result,P,q,A,b,linear_count):
    nodes=request['nodes'];n=len(nodes)
    output={'solver':'Clarabel','solver_version':clarabel.__version__,'solver_status':str(result.status),
            'iterations':result.iterations,'status':'solver-failure','admission_policy':ADMISSION_POLICY['version']}
    if str(result.status) not in ADMISSION_POLICY['candidate_statuses']:
        output['rejection_reasons']=['solver-status-not-an-eligible-candidate']
        return output
    numeric=np.r_[result.x,result.z,result.s,result.obj_val,result.obj_val_dual,result.r_prim,result.r_dual]
    if not np.all(np.isfinite(numeric)):
        output.update({'status':'nonfinite-solver-output','nonfinite_numeric_indices':np.where(~np.isfinite(numeric))[0].tolist()})
        return output
    x=np.array(result.x);u=x[:3*n].reshape(n,3);ds=x[3*n:];residual=[];normalized=[];soft_res=[];index=0
    for i,node in enumerate(nodes):
        for row in node['soft']:soft_res.append(np.array(row['coe'])@u[i]+row['const']+ds[index]);index+=1
    for row in request['hard']:
        w=u[row['i'],:2]-(u[row['j'],:2] if row['j']>=0 else 0.)
        value=np.array(row['a'])@w+row['c']-row['norm_loss']*np.linalg.norm(w)
        residual.append(value);normalized.append(value/max(1.,abs(row['c']),*np.abs(row['a']),row['norm_loss']))
    primal=float(np.sum(u*u)+request['k_delta']*np.sum(ds));minimum=float(min(residual));norm_min=float(min(normalized));soft_min=float(min(soft_res))
    input_excess=float(np.max(np.abs(u)-np.array([request['U'],request['U'],request['omega_max']])))
    z=np.array(result.z);px=P@x;atz=A.T@z
    stationarity=float(np.linalg.norm(px+q+atz,np.inf)/max(1.,np.linalg.norm(px,np.inf),np.linalg.norm(q,np.inf),np.linalg.norm(atz,np.inf)))
    dual_violations=[max(0.,-float(np.min(z[:linear_count])))]
    for start in range(linear_count,len(z),3):dual_violations.append(max(0.,float(np.linalg.norm(z[start+1:start+3])-z[start])))
    dual_cone_relative=max(dual_violations)/max(1.,float(np.linalg.norm(z,np.inf)))
    ax=A@x;s=np.asarray(result.s)
    canonical_primal=float(np.linalg.norm(ax+s-b,np.inf)/max(1.,np.linalg.norm(ax,np.inf),np.linalg.norm(s,np.inf),np.linalg.norm(b,np.inf)))
    gap=abs(result.obj_val-result.obj_val_dual);objective_scale=max(1.,abs(primal),abs(result.obj_val_dual))
    gap_relative=gap/objective_scale;objective_agreement=abs(primal-result.obj_val)/objective_scale
    output.update({'commands':u.tolist(),'slacks':ds.tolist(),'hard_residuals':residual,'min_hard_residual':minimum,
                   'min_normalized_hard_residual':norm_min,'min_soft_residual':soft_min,'min_slack':float(np.min(ds)),
                   'input_excess':input_excess,'primal_cost':primal,'solver_objective':result.obj_val,'cost_gap':abs(primal-result.obj_val),
                   'primal_residual':result.r_prim,'dual_residual':result.r_dual,'dual_objective':result.obj_val_dual,
                   'absolute_duality_gap':gap,'relative_duality_gap':gap_relative,'stationarity_relative':stationarity,
                   'dual_cone_relative':dual_cone_relative,'objective_agreement_relative':objective_agreement,
                   'canonical_primal_relative':canonical_primal,
                   'stationarity_absolute':float(np.linalg.norm(px+q+atz,np.inf)),
                   'stationarity_vector':(px+q+atz).tolist(),
                   'raw_hard_units':'m^2/s', 'solver_residual_scope':'internal scaled coordinates; canonical residuals use original matrices'})
    violations={
        'hard-absolute':minimum < -ADMISSION_POLICY['hard_absolute'],
        'hard-normalized':norm_min < -ADMISSION_POLICY['hard_normalized'],
        'soft-absolute':soft_min < -ADMISSION_POLICY['soft_absolute'],
        'slack-absolute':np.min(ds) < -ADMISSION_POLICY['slack_absolute'],
        'input-absolute':input_excess > ADMISSION_POLICY['input_absolute'],
        'primal-residual':result.r_prim > ADMISSION_POLICY['primal_residual'],
        'dual-residual':result.r_dual > ADMISSION_POLICY['dual_residual'],
        'stationarity':stationarity > ADMISSION_POLICY['stationarity_relative'],
        'dual-cone':dual_cone_relative > ADMISSION_POLICY['dual_cone_relative'],
        'duality-gap':gap_relative > ADMISSION_POLICY['duality_gap_relative'],
        'objective-agreement':objective_agreement > ADMISSION_POLICY['objective_agreement_relative'],
        'canonical-primal':canonical_primal > ADMISSION_POLICY['canonical_primal_relative']
    }
    output['rejection_reasons']=[key for key,value in violations.items() if value]
    execution_keys=['hard-absolute','hard-normalized','input-absolute']
    output['execution_constraints_pass']=not any(violations[key] for key in execution_keys)
    output['optimality_and_representation_pass']=not any(value for key,value in violations.items() if key not in execution_keys)
    output['failure_class']=None if not output['rejection_reasons'] else ('physical-constraint-violation' if not output['execution_constraints_pass'] else 'numerical-optimality-or-representation-failure')
    output['status']='residual-quality-failure' if output['rejection_reasons'] else 'success'
    if True:  # Preserve full primal/dual/slack for every external solve.
        output['raw_solver_candidate']={'x':result.x,'z':result.z,'s':result.s}
    return output
