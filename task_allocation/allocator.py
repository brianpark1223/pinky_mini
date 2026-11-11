"""
task_allocation/allocator.py
Pinky 부품 배송 FMS - Cost 기반 작업 배정 (순수 파이썬, ROS 의존성 없음)

팀 README 규칙을 그대로 따릅니다.
  - 지점: STORE(픽업대), A / B(작업대), HOME(수납대, 두 로봇이 공유)
  - 공용 대기열 하나. "비어 있는" 로봇(IDLE, TO_HOME) 중 cost가 가장 낮은 로봇이 맡음
  - 로봇이 수령 완료(WAIT_RECEIVE)를 마치면 대기열 맨 앞 요청을 이어받음 (next_for)
  - 픽업대는 한 번에 한 대 -> 픽업대 대기 시간을 cost에 반영
  - ERROR 로봇은 후보에서 제외

    C_i = w_dist * clip(D_i / D_max) + w_wait * clip(W_i / W_max) + w_battery * (1 - B_i / 100)

    D_i : 신규 요청의 픽업대(STORE)까지 총 주행 거리 [m]
    W_i : 정지·대기 시간 [s] (ArUco 정렬 + 픽업대 대기 + 현재 작업의 남은 사람 대기 추정)
    B_i : 배터리 (README상 배터리 관리가 없으므로 기본 가중치 0)

FMS 컨트롤러가 쓰는 방법
    alloc = Allocator(ZONES, Config())
    assigned = alloc.submit(request, robots)     # [(robot_name, request)] / 빈 리스트면 대기열
    nxt = alloc.next_for("pinky_1")              # 수령 완료 시: Request 또는 None(-> TO_HOME)
    assigned = alloc.dispatch(robots)            # ERROR -> IDLE 등으로 로봇이 비었을 때
    best = alloc.predict_next(robots)            # GUI '담당 로봇 예상'
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
DistFn = Callable[[Point, Point], float]


# ----------------------------------------------------------------------
# 1. 데이터 정의 (상태 이름은 README 상태 머신과 동일)
# ----------------------------------------------------------------------
class State(str, Enum):
    IDLE = "IDLE"
    TO_STORE = "TO_STORE"
    WAIT_PICKUP = "WAIT_PICKUP"
    WAIT_LOAD = "WAIT_LOAD"
    TO_STATION = "TO_STATION"
    WAIT_RECEIVE = "WAIT_RECEIVE"
    TO_HOME = "TO_HOME"
    ERROR = "ERROR"


# 새 작업을 바로 받을 수 있는 "비어 있는" 상태
AVAILABLE = frozenset({State.IDLE, State.TO_HOME})


@dataclass
class Request:
    id: int
    station: str                 # "A" | "B"
    created_at: float            # 요청 시각 (초). 대기열 순서에 사용
    priority: int = 0            # 클수록 먼저 처리


@dataclass
class Robot:
    """FMS 컨트롤러가 매번 넘겨주는 로봇 스냅샷"""
    name: str                            # "pinky_1" / "pinky_2"
    state: State
    pos: Point                           # map 프레임 (x, y) [m]
    task: Optional[Request] = None       # 현재 맡은 작업 (IDLE/TO_HOME/ERROR는 None)
    state_elapsed_s: float = 0.0         # 현재 상태에 들어온 뒤 경과 시간
    battery: Optional[float] = None      # 0 ~ 100, 없으면 None


@dataclass
class Config:
    w_dist: float = 0.6
    w_wait: float = 0.4
    w_battery: float = 0.0               # 배터리 관리가 없으므로 기본 0 (켜려면 > 0)
    d_max: Optional[float] = None        # None이면 지점 좌표로 자동 추정
    w_max_s: Optional[float] = None      # None이면 시간 설정으로 자동 추정
    v: float = 0.2                       # 평균 주행 속도 [m/s] (실측 필요)
    t_align_s: float = 8.0               # ArUco 정렬 시간 [s]
    t_load_s: float = 20.0               # 픽업대 적재 시간 추정 [s]
    t_receive_s: float = 20.0            # 작업대 수령 시간 추정 [s]
    tie_eps: float = 0.01                # cost 차이가 이보다 작으면 동률로 보고 이름 순


@dataclass
class Estimate:
    robot: str
    available: bool      # 지금 바로 새 작업을 받을 수 있는가
    free_in_s: float     # 이 로봇이 비기까지 남은 시간 (available이면 0)
    D: float             # 신규 요청 픽업대까지 총 주행 거리 [m]
    W: float             # 정지·대기 시간 [s]
    eta_s: float         # 픽업대에서 적재 준비가 되기까지 예상 시간 (D / v + W)
    C: float             # cost


def euclid(a: Point, b: Point) -> float:
    """임시 거리 함수. 나중에 Nav2 경로 길이로 교체하려면 같은 형태의 함수를 dist= 로 전달."""
    return math.dist(a, b)


# ----------------------------------------------------------------------
# 2. 배정기
# ----------------------------------------------------------------------
class Allocator:
    def __init__(self, zones: Dict[str, Point], cfg: Optional[Config] = None,
                 dist: DistFn = euclid):
        self.zones = zones
        self.dist = dist
        cfg = cfg or Config()
        if cfg.d_max is None:
            cfg = replace(cfg, d_max=self._estimate_d_max())
        if cfg.w_max_s is None:
            cfg = replace(cfg, w_max_s=2 * cfg.t_align_s + 2 * cfg.t_load_s + cfg.t_receive_s)
        self.cfg = cfg
        self.queue: List[Request] = []   # 공용 대기열 (GUI에서 읽기용)

    # ---- 정규화 기준 -------------------------------------------------
    def _estimate_d_max(self) -> float:
        """가장 긴 경우: 픽업대로 가던 중 새 요청 -> 작업대 배달 후 다시 픽업대"""
        home, store = self.zones["HOME"], self.zones["STORE"]
        worst = max(self.dist(store, self.zones[s]) for s in ("A", "B"))
        return self.dist(home, store) + 2 * worst

    # ---- 로봇이 비기까지 남은 일 ------------------------------------
    def _remaining(self, r: Robot) -> Optional[Tuple[float, float, Point]]:
        """(남은 주행거리 [m], 남은 대기시간 [s], 비는 위치). ERROR는 None.
        케이스 ①~⑤ 설계를 README 상태에 맞춰 옮긴 것:
          TO_STORE/WAIT_PICKUP=① WAIT_LOAD=③ TO_STATION=② WAIT_RECEIVE=④ TO_HOME=⑤"""
        c, z, s = self.cfg, self.zones, r.state
        if s in AVAILABLE:
            return 0.0, 0.0, r.pos
        if s == State.ERROR or r.task is None:
            return None
        store, st = z["STORE"], z[r.task.station]
        if s in (State.TO_STORE, State.WAIT_PICKUP):
            d = self.dist(r.pos, store) + self.dist(store, st)
            w = c.t_align_s + c.t_load_s + c.t_receive_s
            if s == State.WAIT_PICKUP:
                w += c.t_load_s          # 앞 로봇 적재가 끝나길 기다림 (보수적 추정)
            return d, w, st
        if s == State.WAIT_LOAD:
            w = max(0.0, c.t_load_s - r.state_elapsed_s) + c.t_receive_s
            return self.dist(store, st), w, st
        if s == State.TO_STATION:
            return self.dist(r.pos, st), c.t_receive_s, st
        if s == State.WAIT_RECEIVE:
            return 0.0, max(0.0, c.t_receive_s - r.state_elapsed_s), st
        return None

    def _store_free_in(self, me: Robot, robots: Sequence[Robot]) -> float:
        """다른 로봇이 픽업대를 비우기까지 남은 시간 (픽업대는 한 번에 한 대)"""
        c, store = self.cfg, self.zones["STORE"]
        t = 0.0
        for o in robots:
            if o.name == me.name:
                continue
            if o.state == State.WAIT_LOAD:
                t = max(t, max(0.0, c.t_load_s - o.state_elapsed_s))
            elif o.state in (State.TO_STORE, State.WAIT_PICKUP):
                t = max(t, self.dist(o.pos, store) / c.v + c.t_align_s + c.t_load_s)
        return t

    # ---- cost -------------------------------------------------------
    def cost(self, D: float, W: float, battery: Optional[float]) -> float:
        c = self.cfg
        total = (c.w_dist * min(D / c.d_max, 1.0)
                 + c.w_wait * min(W / c.w_max_s, 1.0))
        if c.w_battery and battery is not None:
            total += c.w_battery * (1.0 - battery / 100.0)
        return total

    def estimate(self, r: Robot, robots: Sequence[Robot]) -> Optional[Estimate]:
        rem = self._remaining(r)
        if rem is None:
            return None
        rem_d, rem_w, free_pos = rem
        c = self.cfg
        D = rem_d + self.dist(free_pos, self.zones["STORE"])
        t_arrive = D / c.v + rem_w
        pickup_wait = max(0.0, self._store_free_in(r, robots) - t_arrive)
        W = rem_w + pickup_wait + c.t_align_s
        return Estimate(
            robot=r.name,
            available=r.state in AVAILABLE,
            free_in_s=rem_d / c.v + rem_w,
            D=D, W=W, eta_s=D / c.v + W,
            C=self.cost(D, W, r.battery),
        )

    def estimates(self, robots: Sequence[Robot], only_available: bool = False) -> List[Estimate]:
        out = []
        for r in robots:
            e = self.estimate(r, robots)
            if e is not None and (e.available or not only_available):
                out.append(e)
        return out

    def _pick(self, ests: List[Estimate]) -> Optional[Estimate]:
        """cost 최저. 차이가 tie_eps 이하면 동률로 보고 이름이 앞선 로봇"""
        if not ests:
            return None
        best = min(e.C for e in ests)
        tied = [e for e in ests if e.C - best <= self.cfg.tie_eps]
        return min(tied, key=lambda e: e.robot)

    # ---- 외부 API ---------------------------------------------------
    def dispatch(self, robots: Sequence[Robot]) -> List[Tuple[str, Request]]:
        """대기열 앞에서부터 '비어 있는' 로봇 중 cost 최저 로봇에게 배정"""
        pool = list(robots)
        out: List[Tuple[str, Request]] = []
        while self.queue:
            pick = self._pick(self.estimates(pool, only_available=True))
            if pick is None:
                break
            out.append((pick.robot, self.queue.pop(0)))
            pool = [r for r in pool if r.name != pick.robot]  # 이미 배정된 로봇 제외
        return out

    def submit(self, req: Request, robots: Sequence[Robot]) -> List[Tuple[str, Request]]:
        """새 요청 접수. 바로 배정되면 [(로봇, 요청)], 빈 로봇이 없으면 [] (대기열에 남음)"""
        self.queue.append(req)
        self.queue.sort(key=lambda q: (-q.priority, q.created_at, q.id))
        return self.dispatch(robots)

    def next_for(self, robot_name: str) -> Optional[Request]:
        """로봇이 수령 완료를 마쳤을 때 호출. 대기열 맨 앞 요청을 넘기고, 없으면 None(-> TO_HOME)"""
        return self.queue.pop(0) if self.queue else None

    def predict_next(self, robots: Sequence[Robot]) -> Optional[Estimate]:
        """GUI '담당 로봇 예상': 비어 있는 로봇이 있으면 cost 최저, 없으면 가장 먼저 비는 로봇"""
        ests = self.estimates(robots)
        avail = [e for e in ests if e.available]
        if avail:
            return self._pick(avail)
        if not ests:
            return None
        return min(ests, key=lambda e: (e.free_in_s, e.robot))
