#!/usr/bin/env python3
"""Bayesian optimisation of the horizontal gains while the vehicle is flying.

Same flight, score and watchdog as online_tuner, but the next gains to try
come from a Gaussian-process model of the score instead of a coordinate
search. Every trial is one lap. The search is confined to `--bounds`, which
should be the stable region found by the sweeps: the optimiser has no notion
of safety beyond the watchdog.

    python3 -m px4_auto_tuning.bayes_tuner --sitl --speed 4 --trials 25

A trial stopped by the watchdog or flown with too much oscillation is kept
in the model with a penalty, so the optimiser learns to stay away from it.
"""

import math

import numpy as np

from .online_tuner import OnlineTuner, make_parser, run

DEFAULT_BOUNDS = 'MPC_XY_P=0.6:3.5,MPC_XY_VEL_P_ACC=1.2:9,MPC_XY_VEL_I_ACC=0.2:3'


def expected_improvement(x_seen, y_seen, x_new, length=0.25, noise=0.05):
    """EI (for minimisation) of a zero-mean RBF Gaussian process on standardised scores."""
    mean, std = np.mean(y_seen), np.std(y_seen) + 1e-9
    y = (np.asarray(y_seen) - mean) / std

    def kernel(a, b):
        d2 = ((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)
        return np.exp(-0.5 * d2 / length ** 2)

    k = kernel(x_seen, x_seen) + noise * np.eye(len(x_seen))
    k_new = kernel(x_new, x_seen)
    solve = np.linalg.solve(k, np.column_stack([y, k_new.T]))
    mu = k_new @ solve[:, 0]
    var = np.clip(1.0 - np.einsum('ij,ji->i', k_new, solve[:, 1:]), 1e-9, None)
    sigma = np.sqrt(var)
    z = (y.min() - mu) / sigma
    cdf = 0.5 * (1.0 + np.vectorize(math.erf)(z / math.sqrt(2.0)))
    pdf = np.exp(-0.5 * z * z) / math.sqrt(2.0 * math.pi)
    return (y.min() - mu) * cdf + sigma * pdf


class BayesTuner(OnlineTuner):

    def __init__(self, args):
        super().__init__(args)
        self.bounds = {}

        for item in args.bounds.split(','):
            name, span = item.split('=')
            low, high = (float(v) for v in span.split(':'))
            self.bounds[name] = (math.log(low), math.log(high))

        self.rng = np.random.default_rng(args.seed)
        self.seen_x = []
        self.seen_y = []

    # Gains are searched on a log scale, mapped to the unit cube
    def _to_unit(self, gains):
        return np.array([(math.log(gains[n]) - self.bounds[n][0]) / (self.bounds[n][1] - self.bounds[n][0])
                         for n in self.tune.gains])

    def _from_unit(self, x):
        return {n: round(math.exp(self.bounds[n][0] + v * (self.bounds[n][1] - self.bounds[n][0])), 3)
                for n, v in zip(self.tune.gains, x)}

    def _record(self, cost):
        self.seen_x.append(self._to_unit(self.gains))
        self.seen_y.append(cost)

    def _next_candidate(self):
        if len(self.seen_y) >= self.tune.trials:
            return False

        dim = len(self.tune.gains)

        if len(self.seen_y) < self.tune.init_trials:
            x = self.rng.random(dim)

        else:
            pool = self.rng.random((2000, dim))
            x = pool[np.argmax(expected_improvement(np.array(self.seen_x), self.seen_y, pool))]

        self.candidate = ('bayes', None)
        self._apply(self._from_unit(x))
        return True

    def _judge(self, score):
        cost = self._cost(score)

        if score.osc > self.tune.osc_limit:
            self._reject(score, 'oscillation above limit')
            return

        self._record(cost)

        if cost < self.best_score:
            self.best = dict(self.gains)
            self.best_score = cost
            self._log('accept', score, 'best so far')

        else:
            self._log('trial', score)

    def _reject(self, score, note):
        # The lap was not completed or shook too much: tell the model it was bad
        worst = max(self.seen_y + [self._cost(self.baseline)])
        self._record(2.0 * worst)
        self._log('reject', score, note)
        self._apply(self.best)


def main():
    parser = make_parser(__doc__)
    parser.add_argument('--bounds', default=None, help=f'NAME=low:high,... (default from {DEFAULT_BOUNDS})')
    parser.add_argument('--trials', type=int, default=25)
    parser.add_argument('--init-trials', type=int, default=6, help='random trials before the model is used')
    parser.add_argument('--seed', type=int, default=0)
    parser.set_defaults(margin=1.0, out='results/bayes_tuner.csv', events='results/raw/bayes_events.csv')
    args = parser.parse_args()

    if args.bounds is None:
        known = dict(item.split('=') for item in DEFAULT_BOUNDS.split(','))
        args.bounds = ','.join(f'{name}={known[name]}' for name in args.gains.split(','))

    run(args, BayesTuner)


if __name__ == '__main__':
    main()
