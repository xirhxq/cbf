"""Reference implementation of joint gamma-star candidate feedback."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable, Sequence

from scripts.analysis.gamma_star_lp import ResidualConstraint, solve_gamma_star


Acceleration = tuple[float, float]
ConstraintPredictor = Callable[[Acceleration], Sequence[ResidualConstraint]]


@dataclass(frozen=True)
class CandidateScore:
    acceleration: Acceleration
    gamma: float


@dataclass(frozen=True)
class FeedbackSelection:
    acceleration: Acceleration
    gamma: float
    active: bool
    scores: tuple[CandidateScore, ...]


def generate_candidate_accelerations(
    *,
    half_box: float = 2.0,
    direction_count: int = 8,
    magnitude_count: int = 3,
) -> list[Acceleration]:
    """Return an L-infinity-bounded direction-by-magnitude lattice."""
    if not math.isfinite(half_box) or half_box <= 0.0:
        raise ValueError("half_box must be finite and positive")
    if direction_count < 1 or magnitude_count < 1:
        raise ValueError("direction_count and magnitude_count must be positive")

    candidates: list[Acceleration] = []
    for magnitude_index in range(1, magnitude_count + 1):
        magnitude = half_box * magnitude_index / magnitude_count
        for direction_index in range(direction_count):
            angle = 2.0 * math.pi * direction_index / direction_count
            dx = math.cos(angle)
            dy = math.sin(angle)
            scale = magnitude / max(abs(dx), abs(dy))
            candidates.append((scale * dx, scale * dy))
    return candidates


def select_feedback_nominal(
    *,
    current_gamma: float,
    search_nominal: Acceleration,
    predict_constraints: ConstraintPredictor,
    threshold: float = 1.0,
    feedback_half_box: float = 2.0,
    direction_count: int = 8,
    magnitude_count: int = 3,
    candidates: Iterable[Acceleration] | None = None,
) -> FeedbackSelection:
    """Select the candidate with the largest exact predicted feedback budget.

    ``predict_constraints`` owns the dynamics and neighbor prediction.  It
    receives one candidate nominal and returns the predicted affine HOCBF
    residuals.  The historical controller used ``feedback_half_box`` both for
    its candidate lattice and for the online gamma-star score.  Physical
    feasibility over the larger actuator box is a separate offline diagnostic.
    """
    if not math.isfinite(current_gamma):
        raise ValueError("current_gamma must be finite")
    if current_gamma >= threshold:
        return FeedbackSelection(
            acceleration=search_nominal,
            gamma=current_gamma,
            active=False,
            scores=(),
        )

    candidate_list = list(
        candidates
        if candidates is not None
        else generate_candidate_accelerations(
            half_box=feedback_half_box,
            direction_count=direction_count,
            magnitude_count=magnitude_count,
        )
    )
    if not candidate_list:
        raise ValueError("at least one candidate is required")

    scores = tuple(
        CandidateScore(
            acceleration=candidate,
            gamma=solve_gamma_star(
                predict_constraints(candidate),
                half_box=feedback_half_box,
            ).gamma,
        )
        for candidate in candidate_list
    )
    best = max(enumerate(scores), key=lambda item: (item[1].gamma, -item[0]))[1]
    return FeedbackSelection(
        acceleration=best.acceleration,
        gamma=best.gamma,
        active=True,
        scores=scores,
    )
