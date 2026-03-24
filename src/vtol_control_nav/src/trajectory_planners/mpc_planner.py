"""
배치 MPC (Model Predictive Control) 궤적 플래너.

이중 적분기(double-integrator) 시스템 모델 기반 유한 수평(finite-horizon) LQR/MPC.
각 축(X, Y, Z)을 독립적으로 처리하며 외부 라이브러리(numpy 등) 없이 동작합니다.

시스템 모델 (축별 독립):
    s_{k+1} = A s_k + B u_k
    A = [[1, dt], [0, 1]]       상태: [위치 오차, 속도]
    B = [[dt²/2], [dt]]         입력: 가속도

비용함수 (수평 N 스텝):
    J = Σ_{k=1}^N (w_pos·e_k² + w_vel·v_k²)  +  Σ_{k=0}^{N-1} w_u·u_k²

배치 최적해:
    X = Phi·s₀ + Gamma·U
    U* = -(Gamma^T Q̄ Gamma + R̄)⁻¹ Gamma^T Q̄ Phi · s₀  =  -F·s₀
    u₀ = U*[0]  (첫 번째 가속도만 적용, Receding Horizon)
"""

from __future__ import annotations

from .base import BaseTrajectoryPlanner
from .simple import move_toward


# ── 행렬 연산 헬퍼 (numpy 미사용, 순수 Python) ─────────────────────────


def _zeros(m: int, n: int) -> list[list[float]]:
    return [[0.0] * n for _ in range(m)]


def _eye(n: int) -> list[list[float]]:
    M = _zeros(n, n)
    for i in range(n):
        M[i][i] = 1.0
    return M


def _mat_mul(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    """행렬 곱셈 A (m×k) · B (k×n) → C (m×n)."""
    m, k, n = len(A), len(A[0]), len(B[0])
    C = _zeros(m, n)
    for i in range(m):
        Ai = A[i]
        Ci = C[i]
        for l in range(k):
            a = Ai[l]
            if a == 0.0:
                continue
            Bl = B[l]
            for j in range(n):
                Ci[j] += a * Bl[j]
    return C


def _mat_inv(A: list[list[float]]) -> list[list[float]]:
    """가우스-요르단 소거법(부분 피벗)으로 정방행렬 역행렬 계산."""
    n = len(A)
    M = [A[i][:] + [1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(M[r][col]))
        M[col], M[pivot] = M[pivot], M[col]
        piv_val = M[col][col]
        if abs(piv_val) < 1e-14:
            raise ValueError(
                'MPC 게인 행렬이 특이(singular)합니다. w_u > 0 인지 확인하세요.'
            )
        inv_p = 1.0 / piv_val
        row_c = M[col]
        for j in range(2 * n):
            row_c[j] *= inv_p
        for r in range(n):
            if r == col:
                continue
            f = M[r][col]
            if f == 0.0:
                continue
            row_r = M[r]
            for j in range(2 * n):
                row_r[j] -= f * row_c[j]
    return [[M[i][n + j] for j in range(n)] for i in range(n)]


def _build_mpc_gain(
    N: int,
    dt: float,
    w_pos: float,
    w_vel: float,
    w_u: float,
) -> list[list[float]]:
    """
    배치 MPC 게인 행렬 F (N×2)를 사전 계산합니다.

    반환값 F 를 이용하면 임의 상태 s₀ = [e₀, v₀] 에 대해
        U* = -F · s₀  →  u₀ = U*[0]
    으로 첫 번째 최적 가속도를 O(N) 연산으로 즉시 구할 수 있습니다.
    """
    A = [[1.0, dt], [0.0, 1.0]]
    B = [[0.5 * dt * dt], [dt]]   # (2×1)
    two_N = 2 * N

    # A^1, A^2, ..., A^N 미리 계산
    A_pows: list[list[list[float]]] = []
    Ap = _eye(2)
    for _ in range(N):
        Ap = _mat_mul(A, Ap)
        A_pows.append([row[:] for row in Ap])
    # A_pows[k] = A^{k+1}

    def _AiB(i: int) -> list[list[float]]:
        """A^i · B (2×1)."""
        if i == 0:
            return [row[:] for row in B]
        return _mat_mul(A_pows[i - 1], B)

    # Phi (2N×2):  Phi[2k:2k+2, :] = A^{k+1}
    Phi = _zeros(two_N, 2)
    for k in range(N):
        for r in range(2):
            for c in range(2):
                Phi[2 * k + r][c] = A_pows[k][r][c]

    # Gamma (2N×N):  Gamma[2k:2k+2, j] = A^{k-j} · B  (j ≤ k)
    Gamma = _zeros(two_N, N)
    for k in range(N):
        for j in range(k + 1):
            ab = _AiB(k - j)      # (2×1)
            for r in range(2):
                Gamma[2 * k + r][j] = ab[r][0]

    # Q̄ 대각 원소 (길이 2N):  위치 행 → w_pos,  속도 행 → w_vel
    q_diag = [w_pos if (l % 2 == 0) else w_vel for l in range(two_N)]

    # GᵀQ̄ (N×2N):  GtQ[i][l] = Gamma[l][i] * q_diag[l]
    GtQ = [[Gamma[l][i] * q_diag[l] for l in range(two_N)] for i in range(N)]

    # H = GᵀQ̄·Gamma + R̄  (N×N),  R̄ = w_u · I
    H = _zeros(N, N)
    for i in range(N):
        GtQi = GtQ[i]
        for j in range(N):
            s = 0.0
            for l in range(two_N):
                s += GtQi[l] * Gamma[l][j]
            H[i][j] = s
        H[i][i] += w_u

    # GᵀQ̄·Phi (N×2)
    GtQPhi = _zeros(N, 2)
    for i in range(N):
        GtQi = GtQ[i]
        for c in range(2):
            s = 0.0
            for l in range(two_N):
                s += GtQi[l] * Phi[l][c]
            GtQPhi[i][c] = s

    # F = H⁻¹ · GᵀQ̄·Phi  (N×2)
    return _mat_mul(_mat_inv(H), GtQPhi)


# ── MPCPlanner ────────────────────────────────────────────────────────


class MPCPlanner(BaseTrajectoryPlanner):
    """
    배치 MPC 기반 궤적 플래너 (이중 적분기 모델).

    파라미터:
        max_step_m   : 한 스텝 최대 이동 거리 [m] — 출력 클램핑
        horizon_steps: 예측 수평 N (기본 15)
        dt           : 제어 주기 [s] (기본 0.1 = 10 Hz)
        w_pos        : 위치 오차 가중치
        w_vel        : 속도 감쇠 가중치 (오버슈트 억제)
        w_u          : 가속도 입력 가중치 (제어 에너지 절약, > 0 필수)
    """

    def __init__(
        self,
        max_step_m: float,
        horizon_steps: int,
        dt: float,
        w_pos: float,
        w_vel: float,
        w_u: float,
    ):
        self._max_step_m = max(0.1, float(max_step_m))
        self._N = max(2, int(horizon_steps))
        self._dt = max(0.02, float(dt))
        self._w_pos = max(0.0, float(w_pos))
        self._w_vel = max(0.0, float(w_vel))
        self._w_u = max(1e-6, float(w_u))   # > 0 → H 정칙성 보장

        self._F: list[list[float]] = _build_mpc_gain(
            self._N, self._dt, self._w_pos, self._w_vel, self._w_u
        )

        self._prev_pos: tuple[float, float, float] | None = None
        self._vel: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ── 설정 업데이트 ─────────────────────────────────────────────────

    def update_config(self, **kwargs) -> None:
        changed = False
        spec = [
            ('_max_step_m', 'max_step_m',    lambda v: max(0.1,  float(v))),
            ('_N',          'horizon_steps', lambda v: max(2,    int(v))),
            ('_dt',         'dt',            lambda v: max(0.02, float(v))),
            ('_w_pos',      'w_pos',         lambda v: max(0.0,  float(v))),
            ('_w_vel',      'w_vel',         lambda v: max(0.0,  float(v))),
            ('_w_u',        'w_u',           lambda v: max(1e-6, float(v))),
        ]
        for attr, key, clamp in spec:
            if key in kwargs:
                new_val = clamp(kwargs[key])
                if new_val != getattr(self, attr):
                    setattr(self, attr, new_val)
                    changed = True
        if changed:
            self._F = _build_mpc_gain(
                self._N, self._dt, self._w_pos, self._w_vel, self._w_u
            )

    def reset_segment(self) -> None:
        """새 웨이포인트 세그먼트 시작 시 속도 추정값 초기화."""
        self._prev_pos = None
        self._vel = (0.0, 0.0, 0.0)

    # ── 다음 위치 셋포인트 계산 ────────────────────────────────────────

    def next_setpoint(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """
        현재 위치와 목표 위치를 받아 다음 위치 셋포인트를 반환합니다.

        내부 속도 추정 → 각 축에 대한 MPC 최적 가속도 계산 →
        p_next = p + v·dt + ½·u₀·dt²  →  max_step_m 클램핑.
        """
        # 속도 추정 (연속 호출 간 위치 차분)
        if self._prev_pos is not None:
            dt = self._dt
            self._vel = (
                (current[0] - self._prev_pos[0]) / dt,
                (current[1] - self._prev_pos[1]) / dt,
                (current[2] - self._prev_pos[2]) / dt,
            )
        self._prev_pos = current

        dt = self._dt
        F0, F1 = self._F[0][0], self._F[0][1]   # 첫 행만 사용 (u₀ 계산)

        sp: list[float] = []
        for i in range(3):
            e = current[i] - target[i]   # 위치 오차
            v = self._vel[i]             # 추정 속도
            # U* = -F·s₀  →  u₀ = -(F[0][0]·e + F[0][1]·v)
            u0 = -(F0 * e + F1 * v)
            # 다음 위치: p + v·dt + ½·u₀·dt²
            sp.append(current[i] + v * dt + 0.5 * u0 * dt * dt)

        # max_step_m 클램핑 (안전 한계)
        return move_toward(current, (sp[0], sp[1], sp[2]), self._max_step_m)
