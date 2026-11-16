"""Same GP/distance objective with one exact-curvature numerical refinement.

Original successful LM candidates are retained bit for bit. Only evaluation-
budget exhaustion triggers refinement. Safety is handled by the unchanged
joint CBF module, independently of this unconstrained target optimizer.
"""
import time
import numpy as np
from scipy.linalg import block_diag, solve_triangular
from scipy.optimize import least_squares, minimize
from legacy_controllers import Goudar as OriginalGoudar


class Goudar(OriginalGoudar):
    @staticmethod
    def exact_hessian(x, A, neighbors):
        out = A.T @ A
        for position, distance, sd in neighbors:
            for k, state in enumerate(x.reshape(-1,4)):
                delta = state[:2] - position
                norm = np.linalg.norm(delta)
                if norm <= 1e-10:
                    raise ValueError('distance factor is not twice differentiable at coincident positions')
                unit = delta / norm
                uu = np.outer(unit, unit)
                out[4*k:4*k+2,4*k:4*k+2] += (uu + (1.-distance/norm)*(np.eye(2)-uu)) / sd**2
        return out

    def solve_target(self, initial, prior, L0, neighbors):
        def evaluate(x): return self.residual_jacobian(x, prior, L0, neighbors)
        x0 = initial.ravel()
        r0, J0 = evaluate(x0)
        initial_cost = float(.5 * r0 @ r0)
        result = least_squares(lambda x:evaluate(x)[0], x0, jac=lambda x:evaluate(x)[1],
                               method='lm', max_nfev=self.parameters['max_nfev'],
                               ftol=1e-7, xtol=1e-7, gtol=1e-7)
        info = {'status':int(result.status),'nfev':result.nfev,'cost':float(result.cost),
                'initial_cost':initial_cost,'optimality':float(result.optimality),
                'refined':False,'original_message':str(result.message)}
        if result.success and np.isfinite(result.x).all():
            return result.x, info
        if result.status != 0 or not np.isfinite(result.x).all():
            return None, info
        dimension = len(x0)
        A = J0[:dimension]
        b = A @ x0 - r0[:dimension]
        center = np.linalg.solve(A,b)
        M = np.linalg.solve(A,np.eye(dimension))
        y0 = A @ result.x - b
        reference_residual = evaluate(result.x)[0]
        def residual(y): return evaluate(center+M@y)[0]
        def objective(y):
            f=residual(y)
            # Subtract a fixed objective constant using a stable difference.
            return .5*(f-reference_residual)@(f+reference_residual)
        def gradient(y):
            f,J=evaluate(center+M@y)
            return M.T@J.T@f
        def hessian(y):
            return M.T@self.exact_hessian(center+M@y,A,neighbors)@M
        started=time.perf_counter()
        polished=minimize(objective,y0,jac=gradient,hess=hessian,method='trust-exact',
                          options={'gtol':1e-7,'maxiter':80})
        candidate=center+M@polished.x
        f,J=evaluate(candidate)
        candidate_cost=float(.5*f@f)
        cos=float(np.max(np.abs(J.T@f)/np.maximum(np.linalg.norm(J,axis=0)*np.linalg.norm(f),1e-30)))
        eig=np.linalg.eigvalsh(self.exact_hessian(candidate,A,neighbors))
        finite=bool(np.isfinite(candidate).all() and np.isfinite(f).all() and np.isfinite(J).all() and np.isfinite(eig).all())
        descent=bool(candidate_cost<=result.cost+1e-12*max(1.,abs(result.cost)))
        stationary=bool(cos<=1e-7)
        curvature=bool(eig[0]>=-1e-8*max(1.,float(np.max(np.abs(eig)))))
        accepted=finite and descent and stationary and curvature
        info.update(refined=True, refinement={'status':int(polished.status),'success':bool(polished.success),
            'message':str(polished.message),'nit':int(polished.nit),'nfev':int(polished.nfev),
            'cost':candidate_cost,'normalized_gradient_cosine':cos,'hessian_min_eigenvalue':float(eig[0]),
            'finite':finite,'cost_nonincrease':descent,'stationarity_pass':stationary,'curvature_pass':curvature,
            'accepted':accepted,'wall_s':time.perf_counter()-started,
            'terminal_change_from_lm_m':float(np.linalg.norm(candidate.reshape(-1,4)[-1,:2]-result.x.reshape(-1,4)[-1,:2]))})
        return (candidate if accepted else None),info

    def step(self,request,cov):
        started=time.perf_counter();nodes=request['nodes']
        targets=np.array([n['goal'] for n in nodes],float);stats=[]
        for i,node in enumerate(nodes):
            if node['leader']:continue
            prior=np.r_[node['position'],node['held_command'][:2]]
            covariance=block_diag(cov[i],np.eye(2)*self.parameters['velocity_prior_variance'])
            L0=solve_triangular(np.linalg.cholesky(covariance),np.eye(4),lower=True)
            neighbors=[(np.array(nodes[j]['position']),d,np.sqrt(max(np.trace(cov[j]),self.parameters['covariance_floor']))) for j,d in self.neighbors[i]]
            if i in self.previous:
                old=self.previous[i]
                initial=np.vstack([old[1:],self.Phi@old[-1]])
            else:initial=np.array([np.r_[prior[:2]+k*self.dt*prior[2:],prior[2:]] for k in range(self.H+1)])
            x,info=self.solve_target(initial,prior,L0,neighbors)
            stats.append({'id':i+1,**info})
            if x is None:
                return {'frame':request['frame'],'status':'formation-solver-failure',
                        'diagnostics':{'nodes':stats,'message':'LM and fixed exact-curvature refinement did not pass numerical qualification'}}
            states=x.reshape(self.H+1,4);self.previous[i]=states;targets[i]=states[-1,:2]
        return {'frame':request['frame'],'status':'success','targets':targets.tolist(),
                'diagnostics':{'nodes':stats,'wall_s':time.perf_counter()-started,
                               'edge_length_m':self.parameters['edge_length_m'],
                               'policy':'original-success-unchanged; budget-exhaustion-only exact-curvature refinement, maxiter80'}}
