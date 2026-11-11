"""
task_allocation/test_allocator.py
실행:  python3 test_allocator.py      (task_allocation/ 폴더 안에서)
pytest가 있으면 pytest test_allocator.py 도 됩니다.
"""
from dataclasses import replace

from allocator import Allocator, Config, Request, Robot, State

# 예시 좌표 [m]. 실제 값은 config/ 의 지점 좌표로 교체
ZONES = {"HOME": (0.0, 0.0), "STORE": (4.0, 0.0), "A": (7.0, 2.0), "B": (7.0, -2.0)}


def req(i, station="A", t=0.0, prio=0):
    return Request(id=i, station=station, created_at=t, priority=prio)


def names(assigned):
    return [(n, r.id) for n, r in assigned]


def test_both_idle_near_equal_is_tie_then_name_order():
    # 수납대를 공유하므로 대기 위치 차이는 사실상 노이즈 -> 동률, 이름 순
    a = Allocator(ZONES, Config())
    robots = [Robot("pinky_1", State.IDLE, (0.0, 0.0)), Robot("pinky_2", State.IDLE, (0.1, 0.0))]
    assert names(a.submit(req(1), robots)) == [("pinky_1", 1)]


def test_returning_robot_closer_to_store_takes_it():
    # 작업대 A에서 HOME으로 복귀 중 (6, 2) -> STORE까지 2.83m < HOME에서 4m
    a = Allocator(ZONES, Config())
    robots = [Robot("pinky_1", State.IDLE, (0.0, 0.0)), Robot("pinky_2", State.TO_HOME, (6.0, 2.0))]
    assert names(a.submit(req(1), robots)) == [("pinky_2", 1)]


def test_returning_robot_farther_than_home_loses():
    a = Allocator(ZONES, Config())
    robots = [Robot("pinky_1", State.IDLE, (0.0, 0.0)), Robot("pinky_2", State.TO_HOME, (0.5, -3.0))]
    assert names(a.submit(req(1), robots)) == [("pinky_1", 1)]


def test_both_busy_goes_to_queue():
    a = Allocator(ZONES, Config())
    robots = [
        Robot("pinky_1", State.TO_STATION, (5.5, 1.0), task=req(10, "A")),
        Robot("pinky_2", State.WAIT_LOAD, (4.0, 0.0), task=req(11, "B")),
    ]
    assert a.submit(req(1), robots) == []
    assert [q.id for q in a.queue] == [1]


def test_error_robot_excluded():
    a = Allocator(ZONES, Config())
    robots = [Robot("pinky_1", State.ERROR, (0.0, 0.0), task=req(10)), Robot("pinky_2", State.IDLE, (0.0, 0.0))]
    assert names(a.submit(req(1), robots)) == [("pinky_2", 1)]
    # 한 대는 ERROR, 한 대는 바쁨 -> 대기열
    a2 = Allocator(ZONES, Config())
    robots2 = [Robot("pinky_1", State.ERROR, (0.0, 0.0), task=req(10)),
               Robot("pinky_2", State.TO_STATION, (5.0, 1.0), task=req(11))]
    assert a2.submit(req(2), robots2) == []


def test_queue_is_fifo_and_next_for_pops_head():
    a = Allocator(ZONES, Config())
    busy = [Robot("pinky_1", State.TO_STATION, (5.5, 1.0), task=req(10)),
            Robot("pinky_2", State.TO_STATION, (5.5, -1.0), task=req(11, "B"))]
    a.submit(req(1, t=1.0), busy)
    a.submit(req(2, t=2.0), busy)
    assert a.next_for("pinky_1").id == 1      # 수령 완료 -> 가장 오래된 요청
    assert a.next_for("pinky_2").id == 2
    assert a.next_for("pinky_1") is None      # 대기열 없음 -> TO_HOME


def test_priority_beats_arrival_order():
    a = Allocator(ZONES, Config())
    busy = [Robot("pinky_1", State.TO_STATION, (5.5, 1.0), task=req(10)),
            Robot("pinky_2", State.TO_STATION, (5.5, -1.0), task=req(11, "B"))]
    a.submit(req(1, t=1.0), busy)
    a.submit(req(3, t=5.0, prio=5), busy)
    assert a.next_for("pinky_1").id == 3


def test_dispatch_after_error_recovery():
    a = Allocator(ZONES, Config())
    busy = [Robot("pinky_1", State.ERROR, (3.0, 0.0), task=req(10)),
            Robot("pinky_2", State.TO_STATION, (5.0, 1.0), task=req(11))]
    a.submit(req(1), busy)
    # 관리자가 작업 취소 -> pinky_1이 IDLE로 복귀
    recovered = [Robot("pinky_1", State.IDLE, (0.0, 0.0)), busy[1]]
    assert names(a.dispatch(recovered)) == [("pinky_1", 1)]


def test_pickup_wait_is_counted():
    # pinky_1이 픽업대에서 막 적재 시작(60초 걸림), pinky_2는 HOME에서 20초 거리
    cfg = Config(t_load_s=60.0)
    a = Allocator(ZONES, cfg)
    robots = [Robot("pinky_1", State.WAIT_LOAD, (4.0, 0.0), task=req(10), state_elapsed_s=0.0),
              Robot("pinky_2", State.IDLE, (0.0, 0.0))]
    e = next(x for x in a.estimates(robots) if x.robot == "pinky_2")
    assert abs(e.W - (40.0 + cfg.t_align_s)) < 1e-6      # 60 - 20 = 40초 대기 + 정렬


def test_predict_next_picks_robot_that_frees_first():
    a = Allocator(ZONES, Config())
    robots = [Robot("pinky_1", State.WAIT_RECEIVE, (7.0, 2.0), task=req(10, "A"), state_elapsed_s=15.0),
              Robot("pinky_2", State.TO_STORE, (2.0, 0.0), task=req(11, "B"))]
    best = a.predict_next(robots)
    assert best.robot == "pinky_1" and abs(best.free_in_s - 5.0) < 1e-6


def test_busy_distance_matches_old_case_formula():
    # 케이스 ②(TO_STATION): 현재->작업대 + 작업대->STORE
    a = Allocator(ZONES, Config())
    r = Robot("pinky_1", State.TO_STATION, (7.0, 2.0), task=req(10, "A"))
    e = a.estimate(r, [r])
    assert abs(e.D - (0.0 + ((3.0 ** 2 + 2.0 ** 2) ** 0.5))) < 1e-6


def test_battery_term_is_optional():
    robots = [Robot("pinky_1", State.IDLE, (0.0, 0.0), battery=20.0),
              Robot("pinky_2", State.IDLE, (0.0, 0.0), battery=90.0)]
    off = Allocator(ZONES, Config())
    assert names(off.submit(req(1), robots)) == [("pinky_1", 1)]        # 기본: 배터리 무시
    on = Allocator(ZONES, Config(w_battery=0.3))
    assert names(on.submit(req(1), robots)) == [("pinky_2", 1)]


if __name__ == "__main__":
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"[PASS] {name}")
        except AssertionError as ex:
            failed += 1
            print(f"[FAIL] {name} {ex}")
    print("\n모든 테스트 통과" if failed == 0 else f"\n실패 {failed}건")
    raise SystemExit(1 if failed else 0)
