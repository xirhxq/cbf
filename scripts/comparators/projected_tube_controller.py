"""Conditional set-projected SCI tube DMPC; no plant or estimator is owned here.

Qualification assumptions and adaptation provenance are in REPORT.md.  In
particular, initial true positions must lie in the declared deployment balls,
and the physical planar plant must execute the commanded exact integrator.
"""
from pathlib import Path
import importlib.util
import json
import time
import numpy as np

HERE = Path(__file__).resolve().parent
LEGACY = HERE / 'legacy_controllers.py'
_spec = importlib.util.spec_from_file_location('_projected_tube_legacy', LEGACY)
_legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_legacy)


def declared_parameters():
    """Frozen inherited settings; old empirical 5 m bound is not retained."""
    path = HERE / 'inherited-mpc-plan.json'
    p = json.loads(path.read_text())['parameters'].copy()
    p.pop('error_bound_m')
    p.update(initial_set_radius_m=float(3*np.sqrt(2.)),
             feedback_interface='projection-onto-propagated-initial-set',
             initial_event='p_i0 in B(deployment_i, initial_set_radius_m)',
             physical_model='exact-commanded-single-integrator',
             yaw_policy='all-shared-soft-rows-fixed-original-q',
             held_command_tolerance=1e-7,
             certificate_tolerance_m=1e-5)
    return p


def project_ball(points, centers, radius):
    displacement = np.asarray(points, float) - np.asarray(centers, float)
    lengths = np.linalg.norm(displacement, axis=-1)
    scale = np.minimum(1., radius / np.maximum(lengths, 1e-30))
    return np.asarray(centers, float) + displacement * scale[..., None]


def yaw_from_shared_soft(rows, planar, omega_max, penalty):
    """Exact 1-D convex minimization after eliminating all nonnegative slacks."""
    if omega_max < 0 or penalty < 0:
        raise ValueError('Yaw bound and slack penalty must be nonnegative')
    a = np.array([row['coe'][2] for row in rows], float)
    c = np.array([row['const'] + np.dot(row['coe'][:2], planar) for row in rows], float)
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(c)):
        raise ValueError('Nonfinite shared soft row')
    cuts = [-float(omega_max), float(omega_max)]
    for aj, cj in zip(a, c):
        if abs(aj) > 1e-15 and -omega_max < -cj/aj < omega_max:
            cuts.append(float(-cj/aj))
    cuts = sorted(set(cuts))
    candidates = cuts + [0.]
    for lower, upper in zip(cuts[:-1], cuts[1:]):
        middle = .5*(lower+upper)
        stationary = .5*penalty*np.sum(a[a*middle+c < 0])
        candidates.append(float(np.clip(stationary, lower, upper)))
    def objective(w):
        return float(w*w + penalty*np.maximum(0., -(a*w+c)).sum())
    chosen = min(candidates, key=lambda w: (objective(w), abs(w), w))
    residual = a*chosen+c
    slack = np.maximum(0., -residual)
    return float(chosen), {'objective':objective(chosen), 'slacks':slack.tolist(),
                           'row_names':[row.get('name', '') for row in rows],
                           'soft_residual_after_slack':(residual+slack).tolist()}


class ProjectedTubeDMPC(_legacy.Nikou):
    """Preserves the legacy sequential nominal optimizer and certified tail."""
    def __init__(self, cfg, parameters=None):
        p = declared_parameters()
        if parameters is not None:
            for key, value in parameters.items():
                if key not in p or value != p[key]:
                    raise ValueError('Undeclared or changed protocol parameter: '+key)
        self.protocol = p.copy()
        self.r0 = p['initial_set_radius_m']
        legacy_p = p.copy()
        legacy_p['error_bound_m'] = 2*self.r0
        super().__init__(cfg, legacy_p)
        if abs(self.dt-.1) > 1e-12 or p['physical_disturbance_mps'] != 0:
            raise ValueError('This certificate requires T=.1 and zero physical disturbance')
        self.Ubar = self.U - 2*self.k*self.r0
        if self.Ubar <= 0 or self.kf*self.eps > self.Ubar:
            raise ValueError('Empty tightened input or inadmissible terminal input')
        self.center = np.array(cfg['initial']['position']['positions'], float)
        if self.center.shape != (self.n, 2):
            raise ValueError('Deployment center shape mismatch')
        self.previous_frame = -1
        self.previous_command = None
        self.total_projection_count = 0
        self.max_projection_amplitude = 0.

    def _cost(self, z, plan, reference):
        path = self.paths(z, plan)
        error = path-reference[:, None, :]
        costs = self.dt*(self.qweight*np.sum(error[:, :-1]**2, axis=(1, 2))
                        +self.rweight*np.sum(plan**2, axis=(1, 2)))
        costs += self.S*np.sum(error[:, -1]**2, axis=1)
        return {'total':float(costs.sum()), 'per_node':costs.tolist()}

    def _margin_at(self, z, plan, reference):
        old = self.z
        self.z = z
        try:
            return float(self.validate_plan(plan, reference))
        finally:
            self.z = old

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
        accepted_lambda = 0.
        updated = False
        if frame % self.p['replan_every_steps'] == 0:
            for alpha in self.p['reference_fractions']:
                candidate = self.reference+alpha*(goals-self.reference)
                if not self.terminal_safe(candidate):
                    attempts.append({'lambda':alpha, 'status':'terminal-geometry-rejected'})
                    continue
                if np.max(np.linalg.norm(candidate-self.z, axis=1)) > self.H*self.dt*np.sqrt(2)*self.Ubar+self.eps:
                    attempts.append({'lambda':alpha, 'status':'horizon-reachability-rejected'})
                    continue
                plan, stats = self.solve_candidate(candidate)
                attempts.append({'lambda':alpha, 'status':'accepted' if plan is not None else 'numerical-solver-or-certificate-rejected', 'nodes':stats})
                if plan is not None:
                    self.plan = plan
                    self.reference = candidate
                    accepted_lambda = alpha
                    updated = True
                    break
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
            'method':'conditional-set-projected-SCI-tube-DMPC',
            'initial_event':self.protocol['initial_event'],
            'r0_m':self.r0, 'zeta_m':self.zeta, 'nominal_limit_mps':self.Ubar,
            'center':self.center.tolist(), 'z_before':z_before.tolist(),
            'z_after':z_after.tolist(), 'SCI_original':estimates.tolist(),
            'SCI_projected':projected.tolist(), 'center_to_z_norms':b_norms.tolist(),
            'projection_amplitudes_m':amplitudes.tolist(), 'projection_count':count,
            'projection_count_cumulative':self.total_projection_count,
            'projection_max_amplitude_cumulative_m':self.max_projection_amplitude,
            'reference':self.reference.tolist(), 'task_q_unchanged':goals.tolist(),
            'accepted_lambda':accepted_lambda, 'updated':updated, 'attempts':attempts,
            'backup_plan_margin_m':backup_margin, 'plan_margin_m':margin,
            'shifted_plan_margin_m':shifted_margin,
            'nominal_plan_cost':self._cost(z_before, self.plan, self.reference),
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


Controller = ProjectedTubeDMPC
