"""
Kapalı tur TSP.

n ≤ 12 için Held–Karp (kesin), aksi halde çok başlangıçlı en yakın komşu + 2-opt + Or-opt.
patrol_planner simetrikleştirilmiş Nav2 yol uzunluklarını verir ((d + dᵀ) / 2).
"""

import itertools
from typing import List, Sequence

import numpy as np


def tour_cost(order: Sequence[int], d: np.ndarray) -> float:
    return float(sum(d[order[i], order[(i + 1) % len(order)]] for i in range(len(order))))


def _held_karp(d: np.ndarray) -> List[int]:
    n = len(d)
    full = 1 << n
    dp = np.full((full, n), np.inf)
    parent = np.full((full, n), -1, dtype=np.int64)
    dp[1, 0] = 0.0
    for mask in range(1, full):
        if not mask & 1:
            continue
        for j in range(n):
            if not (mask >> j) & 1 or not np.isfinite(dp[mask, j]):
                continue
            for k in range(n):
                if (mask >> k) & 1:
                    continue
                nm = mask | (1 << k)
                v = dp[mask, j] + d[j, k]
                if v < dp[nm, k]:
                    dp[nm, k] = v
                    parent[nm, k] = j
    last = int(np.argmin(dp[full - 1] + d[:, 0]))
    order, mask = [], full - 1
    while last != -1:
        order.append(last)
        prev = int(parent[mask, last])
        mask &= ~(1 << last)
        last = prev
    return order[::-1]


def _improve(order: List[int], d: np.ndarray) -> List[int]:
    best = tour_cost(order, d)
    n = len(order)
    improved = True
    while improved:
        improved = False
        # 2-opt
        for i, j in itertools.combinations(range(1, n), 2):
            cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
            c = tour_cost(cand, d)
            if c < best - 1e-9:
                order, best, improved = cand, c, True
        # Or-opt: 1..3 uzunluklu parçayı başka yere taşı
        for seg in (1, 2, 3):
            for i in range(1, n - seg + 1):
                piece = order[i:i + seg]
                rest = order[:i] + order[i + seg:]
                for k in range(1, len(rest) + 1):
                    for pc in (piece, piece[::-1]):
                        cand = rest[:k] + pc + rest[k:]
                        c = tour_cost(cand, d)
                        if c < best - 1e-9:
                            order, best, improved = cand, c, True
                            break
                    if improved:
                        break
                if improved:
                    break
            if improved:
                break
    return order


def solve(d: np.ndarray) -> List[int]:
    """Kapalı turda düğüm sırası (0 ile başlar)."""
    n = len(d)
    if n <= 2:
        return list(range(n))
    if n <= 12:
        return _held_karp(d)
    best_order, best = None, np.inf
    for start in range(min(n, 8)):  # çok başlangıç; 2-opt/Or-opt ile yeterli
        order, left = [start], set(range(n)) - {start}
        while left:
            nxt = min(left, key=lambda k: d[order[-1], k])
            order.append(nxt)
            left.remove(nxt)
        order = _improve(order, d)
        c = tour_cost(order, d)
        if c < best:
            best_order, best = order, c
    i0 = best_order.index(0)
    return best_order[i0:] + best_order[:i0]
