"""Continuous artificial-reference tube DMPC; frozen physical interface retained.

This is a declared adaptation combining the prior Nikou tube/terminal construction
with a tracking-MPC artificial equilibrium. Dynamic targets and nonconvex
collision constraints do not inherit the original global convergence theorem.
"""
import time
import numpy as np
from scipy.optimize import minimize
from projected_tube_controller import (ProjectedTubeDMPC, declared_parameters as inherited_parameters,
                                       project_ball, yaw_from_shared_soft, _legacy)


def declared_parameters():
    p = inherited_parameters()
    p.pop('reference_fractions')
    p.update(reference_policy='continuous-per-node-artificial-equilibrium',
             offset_weight_policy='equal-to-inherited-terminal-S',
             solver_coordinates='exact-objective-Hessian-whitening',
             coordinate_sweeps=1, solver_ftol=1e-9,
             objective_acceptance_relative=1e-10)
    return p


class ArtificialReferenceDMPC(ProjectedTubeDMPC):
    def __init__(self, cfg, parameters=None):
        p = declared_parameters()
        if parameters is not None and parameters != p:
            raise ValueError('Protocol differs from predeclared configuration')
        super().__init__(cfg)
        self.protocol = p.copy()
        self.p = p.copy()
        self.offset_weight = self.S
        # A bijective change of variables; physical objective and feasible set
        # are unchanged. The quadratic objective Hessian becomes identity.
        B = np.zeros((self.H+1, 2, 2*self.H+2))
        for ell in range(self.H+1):
            B[ell, :, :-2] = np.kron(self.A[ell:ell+1], np.eye(2))
            B[ell, :, -2:] = -np.eye(2)
        hessian = 2*self.dt*self.qweight*np.einsum('lai,laj->ij', B[:-1], B[:-1])
        hessian += 2*self.S*B[-1].T@B[-1]
        hessian[:-2, :-2] += 2*self.dt*self.rweight*np.eye(2*self.H)
        hessian[-2:, -2:] += 2*self.offset_weight*np.eye(2)
        hessian /= 1000.
        self.coordinate_map = np.linalg.solve(np.linalg.cholesky(hessian).T, np.eye(2*self.H+2))
        self.objective_hessian = hessian

    def local_artificial_functions(self, i, paths, references, goal):
        """Exact objective/constraint derivatives in physical v,r coordinates."""
        H, A, z = self.H, self.A, self.z[i]
        peers = [j for j in range(self.n) if j != i]
        linkset = set(self.links)
        links = [j for j in peers if tuple(sorted((i, j))) in linkset]
        anchors = [np.asarray(self.cfg['bases'][b]) for node, b in self.anchors if node == i]
        def objective(x):
            v, r = x[:-2].reshape(H, 2), x[-2:]
            error = z + A @ v - r
            value = self.dt*(self.qweight*np.sum(error[:-1]**2)+self.rweight*np.sum(v*v))
            value += self.S*np.sum(error[-1]**2)+self.offset_weight*np.sum((r-goal)**2)
            gv = (2*self.dt*self.qweight*A[:-1].T@error[:-1]+2*self.dt*self.rweight*v
                  +2*self.S*A[-1, :, None]*error[-1])
            gr = (-2*self.dt*self.qweight*error[:-1].sum(axis=0)-2*self.S*error[-1]
                  +2*self.offset_weight*(r-goal))
            return float(value/1000.), np.r_[gv.ravel(), gr]/1000.
        def constraints(x):
            v, r = x[:-2].reshape(H, 2), x[-2:]
            p = z+A@v
            values, jac = [], []
            def add_path(distances, derivatives, coefficients, limit, sign):
                values.append(sign*(distances-limit))
                j = np.zeros((len(distances), 2*H+2))
                j[:, :-2] = (coefficients[:, :, None]*(sign*derivatives[:, None, :])).reshape(len(distances), 2*H)
                jac.append(j)
            def add_reference(other, limit, sign):
                delta = r-other
                norm = np.linalg.norm(delta)
                row = np.zeros(2*H+2)
                row[-2:] = sign*delta/max(norm, 1e-12)
                values.append(np.array([sign*(norm-limit)]))
                jac.append(row[None, :])
            for j in peers:
                delta = p-paths[j]
                distance, tau, closest = _legacy.segment_minimum(delta[:-1], delta[1:])
                coeff = (1-tau[:, None])*A[:-1]+tau[:, None]*A[1:]
                add_path(distance, closest/np.maximum(distance[:, None], 1e-12), coeff, self.D+2*self.zeta, 1.)
            for j in links:
                delta = p-paths[j]
                distance = np.linalg.norm(delta, axis=1)
                add_path(distance, delta/np.maximum(distance[:, None], 1e-12), A, self.R-2*self.zeta, -1.)
            for anchor in anchors:
                delta = p-anchor
                distance = np.linalg.norm(delta, axis=1)
                add_path(distance, delta/np.maximum(distance[:, None], 1e-12), A, self.R-self.zeta, -1.)
            delta = p[-1]-r
            distance = np.linalg.norm(delta)
            unit = delta/max(distance, 1e-12)
            values.append(np.array([self.eps-distance]))
            jac.append(np.r_[(-A[-1, :, None]*unit).ravel(), unit][None, :])
            for j in peers:
                add_reference(references[j], self.D+2*self.zeta+2*self.eps, 1.)
            for j in links:
                add_reference(references[j], self.R-2*self.zeta-2*self.eps, -1.)
            for anchor in anchors:
                add_reference(anchor, self.R-self.zeta-self.eps, -1.)
            return np.concatenate(values), np.vstack(jac)
        return objective, constraints

    def optimize_artificial(self, goals):
        plan, reference = self.plan.copy(), self.reference.copy()
        stats = []
        for i in range(self.n):
            paths = self.paths(self.z, plan)
            objective, constraints = self.local_artificial_functions(i, paths, reference, goals[i])
            initial = np.r_[plan[i].ravel(), reference[i]]
            initial_cost = objective(initial)[0]
            initial_margin = float(np.min(constraints(initial)[0]))
            if initial_margin < -self.protocol['certificate_tolerance_m']:
                raise ValueError('Artificial-reference coordinate initial point is not certified')
            D = self.coordinate_map
            def physical(y):
                return initial+D@y
            def scaled_objective(y):
                value, gradient = objective(physical(y))
                return value, D.T@gradient
            def scaled_constraints(y):
                x = physical(y)
                value, jacobian = constraints(x)
                # Preserve the original physical input box exactly.
                return (np.r_[value, self.Ubar-x[:-2], self.Ubar+x[:-2]],
                        np.vstack((jacobian@D, -D[:-2], D[:-2])))
            result = minimize(scaled_objective, np.zeros_like(initial), method='SLSQP', jac=True,
                              constraints={'type':'ineq', 'fun':lambda y:scaled_constraints(y)[0],
                                           'jac':lambda y:scaled_constraints(y)[1]},
                              options={'maxiter':self.p['maxiter'], 'ftol':self.p['solver_ftol'], 'disp':False})
            result.x = physical(result.x)
            finite = bool(np.all(np.isfinite(result.x)))
            accepted = False
            detail = {'id':i+1, 'solver_status':int(result.status), 'solver_message':str(result.message),
                      'iterations':int(result.nit), 'initial_objective_scaled':initial_cost,
                      'initial_constraint_margin_m':initial_margin, 'success':bool(result.success)}
            if finite:
                trial_plan, trial_reference = plan.copy(), reference.copy()
                trial_plan[i], trial_reference[i] = result.x[:-2].reshape(self.H, 2), result.x[-2:]
                cost = objective(result.x)[0]
                margin = float(np.min(constraints(result.x)[0]))
                terminal_pass = bool(self.terminal_safe(trial_reference))
                batch_margin = float(self.validate_plan(trial_plan, trial_reference))
                input_excess = float(np.max(np.abs(trial_plan))-self.Ubar)
                cost_pass = cost <= initial_cost+self.p['objective_acceptance_relative']*max(1., abs(initial_cost))
                accepted = bool(result.success and margin >= -self.protocol['certificate_tolerance_m']
                                and batch_margin >= -self.protocol['certificate_tolerance_m']
                                and input_excess <= 1e-7 and terminal_pass and cost_pass)
                detail.update(candidate_objective_scaled=cost, candidate_constraint_margin_m=margin,
                              terminal_geometry_pass=terminal_pass, batch_margin=batch_margin,
                              nominal_input_excess_mps=input_excess,
                              objective_nonincrease=bool(cost_pass),
                              reference_displacement_m=float(np.linalg.norm(trial_reference[i]-reference[i])))
                if accepted:
                    plan, reference = trial_plan, trial_reference
            detail['accepted'] = accepted
            stats.append(detail)
        changed = bool(np.max(np.linalg.norm(reference-self.reference, axis=1)) > 1e-10)
        self.plan, self.reference = plan, reference
        return stats, changed

    def step(self, request, cov=None):
        started = time.perf_counter()
        frame = request['frame']
        if frame != self.previous_frame+1:
            raise ValueError('Nonconsecutive frame: expected '+str(self.previous_frame+1))
        nodes = request['nodes']
        if [n['id'] for n in nodes] != list(range(1, self.n+1)):
            raise ValueError('Node order differs from declared deployment order')
        estimates = np.array([n['position'] for n in nodes], float)
        held = np.array([n['held_command'] for n in nodes], float)
        goals = np.array([n['goal'] for n in nodes], float)
        if not all(np.all(np.isfinite(x)) for x in (estimates, held, goals)):
            raise ValueError('Nonfinite input')
        command_discrepancy = 0.
        if self.previous_command is None:
            if frame != 0 or np.max(np.abs(held)) > self.protocol['held_command_tolerance']:
                raise ValueError('Initialization requires frame 0 with zero prior held command')
        else:
            command_discrepancy = float(np.max(np.abs(held-self.previous_command)))
            if command_discrepancy > self.protocol['held_command_tolerance']:
                raise ValueError('Applied command differs from previously emitted command')
            self.center += self.dt*held[:, :2]
        projected = project_ball(estimates, self.center, self.r0)
        amplitudes = np.linalg.norm(projected-estimates, axis=1)
        count = int(np.count_nonzero(amplitudes > 1e-12))
        if self.z is None:
            self.links, self.anchors = _legacy.graph_from_request(request, self.cfg['bases'])
            self.z = projected.copy()
            self.reference = projected.copy()
            self.plan = np.zeros((self.n, self.H, 2))
            if not self.terminal_safe(self.reference):
                raise ValueError('Initial terminal set does not pass declared geometry')
        elif _legacy.graph_from_request(request, self.cfg['bases']) != (self.links, self.anchors):
            raise ValueError('Reference graph changed; old feasible witness is not certified')
        z_before = self.z.copy()
        b_norms = np.linalg.norm(self.center-z_before, axis=1)
        tol = self.protocol['certificate_tolerance_m']
        if float(b_norms.max()) > self.r0+tol:
            raise ValueError('Propagated center-to-nominal invariant violated')
        backup_margin = float(self.validate_plan(self.plan, self.reference))
        if backup_margin < -tol:
            raise ValueError('Stored shifted backup fails numerical certificate')
        attempts = []
        updated = False
        if frame % self.p['replan_every_steps'] == 0:
            attempts, updated = self.optimize_artificial(goals)
        margin = float(self.validate_plan(self.plan, self.reference))
        if margin < -tol:
            raise ValueError('Selected full-batch plan fails certificate')
        nominal = self.plan[:, 0].copy()
        ancillary = -self.k*(projected-self.z)
        planar = nominal+ancillary
        if np.max(np.abs(planar)) > self.U+1e-7:
            raise ValueError('Actual planar command exceeds declared hard limit')
        commands = np.zeros((self.n, 3))
        commands[:, :2] = planar
        yaw_diagnostics = []
        for i, node in enumerate(nodes):
            commands[i, 2], yaw_info = yaw_from_shared_soft(
                node['soft'], planar[i], self.cfg['cbfs']['input-limits']['yaw-rate-max'],
                self.cfg['cbfs']['objective-function']['k_delta'])
            yaw_diagnostics.append(yaw_info)
        paths = self.paths(self.z, self.plan)
        tail = -self.kf*(paths[:, -1]-self.reference)
        shifted = np.concatenate((self.plan[:, 1:], tail[:, None, :]), axis=1)
        z_after = self.z+self.dt*nominal
        shifted_margin = self._margin_at(z_after, shifted, self.reference)
        if shifted_margin < -tol:
            raise ValueError('Terminal append fails full-batch shifted certificate')
        self.total_projection_count += count
        self.max_projection_amplitude = max(self.max_projection_amplitude, float(amplitudes.max()))
        diagnostic = {
            'method':'continuous-artificial-reference-set-projected-tube-DMPC',
            'initial_event':self.protocol['initial_event'],
            'r0_m':self.r0, 'zeta_m':self.zeta, 'nominal_limit_mps':self.Ubar,
            'center':self.center.tolist(), 'z_before':z_before.tolist(),
            'z_after':z_after.tolist(), 'SCI_original':estimates.tolist(),
            'SCI_projected':projected.tolist(), 'center_to_z_norms':b_norms.tolist(),
            'projection_amplitudes_m':amplitudes.tolist(), 'projection_count':count,
            'projection_count_cumulative':self.total_projection_count,
            'projection_max_amplitude_cumulative_m':self.max_projection_amplitude,
            'reference':self.reference.tolist(), 'task_q_unchanged':goals.tolist(),
            'updated':updated, 'coordinate_updates':attempts, 'offset_weight':self.offset_weight,
            'backup_plan_margin_m':backup_margin, 'plan_margin_m':margin,
            'shifted_plan_margin_m':shifted_margin,
            'nominal_plan_cost':self._cost(z_before, self.plan, self.reference),
            'offset_cost':float(self.offset_weight*np.sum((self.reference-goals)**2)),
            'nominal_plan':self.plan.tolist(), 'nominal_command':nominal.tolist(),
            'ancillary_command':ancillary.tolist(), 'actual_command':commands.tolist(),
            'held_command_discrepancy':command_discrepancy, 'yaw':yaw_diagnostics,
            'prediction_payload_bytes':int(self.plan.nbytes),
            'wall_s':time.perf_counter()-started,
        }
        self.z = z_after
        self.plan = shifted
        self.previous_command = commands.copy()
        self.previous_frame = frame
        # Top-level targets would silently select the public QP in current native.
        return {'frame':frame, 'status':'success', 'commands':commands.tolist(), 'diagnostics':diagnostic}


Controller = ArtificialReferenceDMPC
