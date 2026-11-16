"""Unified comparator worker supporting Oruganti, Cano, Goudar, and Artificial-Reference Tube DMPC."""
import json
import sys
import traceback
import numpy as np

from artificial_reference_controller import ArtificialReferenceDMPC
from composite import Composite
from legacy_controllers import Oruganti


def build_controller(cfg: dict):
    method = cfg['comparator']['method']
    params = cfg['comparator'].get('parameters', {})
    if method == 'oruganti':
        return Oruganti(cfg, params)
    if method in ('cano', 'goudar'):
        return Composite(cfg, params)
    if method == 'artificial-reference-tube-dmpc':
        return ArtificialReferenceDMPC(cfg, params)
    raise ValueError(f'Unsupported comparator method: {method}')


def main() -> None:
    cfg = json.loads(open(sys.argv[1]).read())
    controller = build_controller(cfg)
    for line in sys.stdin:
        packet = json.loads(line)
        req = packet['request']
        try:
            response = controller.step(req, np.array(packet['covariance']))
        except Exception as exc:
            response = {
                'frame': req['frame'],
                'status': 'controller-exception',
                'diagnostics': {
                    'type': type(exc).__name__,
                    'message': str(exc),
                    'traceback': traceback.format_exc(),
                },
            }
        print(json.dumps(response, separators=(',', ':'), allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
