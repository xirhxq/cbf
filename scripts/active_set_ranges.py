"""Active-set ranging: assigned CBF references plus every causally upstream peer.

Restores the pre-rewrite admission rule of the manuscript lineage
(TAES draft d6a9573, Eq. active_reference_set), read as the physical situation:

    R_i(t) = R_i^CBF  U  B_i^visible(t)  U  A_i^visible(t)

- R_i^CBF: the two assigned references emitted by the simulator formation;
  these are also the only CBF control edges and are always admitted.
- B_i^visible(t): other bUAVs that return an available range.
- A_i^visible(t): other sUAVs with a strictly smaller index that return an
  available range.

Two deliberate differences from a literal fixed-radius reading:

1. No hard range cutoff. The user's intent is to model the physical link:
   whether a range is received is decided by the distance-dependent
   availability model p(d) = 1/(1+(d/850)^2), which decays smoothly and never
   becomes exactly zero. 850 m is a scale parameter (and the reference-distance
   design limit), not a sensing horizon. Ranges beyond 850 m are therefore
   admissible and simply less likely and noisier.
2. Causality is preserved by index rather than by radius. Every extra UAV
   anchor has a strictly smaller index, so the estimator's existing
   ascending-index sequential update always consumes a state that was already
   written in the current frame. Mutual feedback between two sUAVs is
   impossible by construction; no common snapshot is required.

Availability and noise keep the original model:
p(d) = 1/(1+(d/850)^2), sigma(d) = 0.5*(1+(d/850)^2).
Assigned-edge latent draws keep the frozen historical random field; extra
edges use the same per-edge counter-field mechanism as the all-range run.
"""
from pathlib import Path
import hashlib
import json

import numpy as np

import counter_field

# Counter-only history: no frozen prefix.
OLD_FIELD = []

LAST_SELECTION = {}
LAST_AUDIT = []


def active_set_ranges(state, bases, seed, frame, cutoff_m=None):
    if cutoff_m is not None and (not np.isfinite(cutoff_m) or cutoff_m <= 0):
        raise ValueError('cutoff must be finite and positive')
    LAST_AUDIT.clear()
    pos = {r['id']: np.array([r['x'], r['y']]) for r in state['robots']}
    ids = sorted(pos)

    # Candidates: all bases, plus every strictly lower-index sUAV (any squad).
    candidates = {}
    for i in ids:
        candidates[i] = {'baseIds': list(range(len(bases))),
                         'anchorIds': [j for j in ids if j < i]}

    latent = counter_field.latent(frame, [{'id': i, **candidates[i]} for i in ids], seed)

    assigned = {}
    for r in state['robots']:
        assigned[r['id']] = (
            {(r['id'], 'base', j) for j in r['formation']['baseIds']}
            | {(r['id'], 'uav', j) for j in r['formation']['anchorIds']}
        )

    frozen = {(e['node'], e['kind'], e['reference']): e for e in OLD_FIELD[frame]['edges']} if frame < len(OLD_FIELD) else {}

    refs = {i: [] for i in ids}
    counts = {i: {'potential': 0, 'available': 0, 'assigned_available': 0,
                  'extra_admitted': 0, 'extra_beyond_850': 0,
                  'max_extra_range': 0.0} for i in ids}

    for generated in latent['edges']:
        key = (generated['node'], generated['kind'], generated['reference'])
        edge = frozen.get(key, generated)
        i, j, kind = edge['node'], edge['reference'], edge['kind']
        is_assigned = key in assigned[i]
        anchor = np.array(bases[j]) if kind == 'base' else pos[j]
        d = float(np.linalg.norm(pos[i] - anchor))
        sigma = 0.5 * (1 + (d / 850) ** 2)
        counts[i]['potential'] += 1
        if cutoff_m is not None and d >= cutoff_m:
            LAST_AUDIT.append([i, kind, j, d, 'hard-cutoff'])
            continue
        if edge['availability_uniform'] <= 1 / (1 + (d / 850) ** 2):
            LAST_AUDIT.append([i, kind, j, d, 'delivered'])
            refs[i].append({
                'key': (kind, j),
                'range': d + sigma * edge['standard_normal'],
                'true_range': d,
                'ranging_sigma': sigma,
            })
            counts[i]['available'] += 1
            if is_assigned:
                counts[i]['assigned_available'] += 1
            else:
                c = counts[i]
                c['extra_admitted'] += 1
                if d > 850.0:
                    c['extra_beyond_850'] += 1
                if d > c['max_extra_range']:
                    c['max_extra_range'] = d
        else:
            LAST_AUDIT.append([i, kind, j, d, 'random-loss'])

    for values in refs.values():
        values.sort(key=lambda x: x['key'])
    LAST_SELECTION.clear()
    LAST_SELECTION.update(counts)
    return refs
