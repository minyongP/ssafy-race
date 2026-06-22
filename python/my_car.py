from DrivingInterface.drive_controller import DrivingController
import math
import csv
import json
import os
from datetime import datetime

before = 0  # 핸들링 스무딩용

class DrivingClient(DrivingController):


    # ========================= 파일명 자동 증가 ========================= #
    def _get_next_log_file(self, log_dir, prefix="log_gpt_v", ext=".csv"):
        """log_dir 안에서 log_gpt_v1.csv, v2... 안 겹치는 다음 파일명 리턴"""
        os.makedirs(log_dir, exist_ok=True)
        v = 1
        while True:
            path = os.path.join(log_dir, f"{prefix}{v}{ext}")
            if not os.path.exists(path):
                return path
            v += 1

    def _calc_best_gap(self, obstacles, ped, half_load_width):
        """장애물 클러스터 기준으로 도로 내 가장 넓은 빈 구간의 (폭, 중앙)을 계산"""
        try:
            left_edge = -float(half_load_width)
            right_edge = float(half_load_width)

            intervals = []
            for o in obstacles or []:
                tm = float(o.get("to_middle", 0.0))
                intervals.append((tm - float(ped), tm + float(ped)))

            if not intervals:
                return None, None

            intervals.sort(key=lambda x: x[0])

            merged = []
            for a, b in intervals:
                if not merged or a > merged[-1][1]:
                    merged.append([a, b])
                else:
                    merged[-1][1] = max(merged[-1][1], b)

            segs = []
            cur = left_edge
            for a, b in merged:
                if a > cur:
                    segs.append((cur, a))
                cur = max(cur, b)
            if cur < right_edge:
                segs.append((cur, right_edge))

            if not segs:
                return None, None

            best = max(segs, key=lambda s: (s[1] - s[0]))
            width = float(best[1] - best[0])
            center = float((best[0] + best[1]) * 0.5)
            return width, center
        except Exception:
            return None, None

    def _calc_free_segments(self, obstacles, ped, half_load_width):
        """
        장애물들을 ped(좌우 여유)로 확장한 interval로 보고,
        도로(-half_load_width ~ +half_load_width) 내 '비어있는 구간'들을 반환합니다.
        Returns: list of (a, b, width, center)
        """
        try:
            left_edge = -float(half_load_width)
            right_edge = float(half_load_width)

            intervals = []
            for o in obstacles or []:
                tm = float(o.get('to_middle', 0.0))
                intervals.append((tm - float(ped), tm + float(ped)))

            cleaned = []
            for a, b in intervals:
                a = max(left_edge, float(a))
                b = min(right_edge, float(b))
                if b > a:
                    cleaned.append((a, b))

            if not cleaned:
                return [(left_edge, right_edge, right_edge - left_edge, (left_edge + right_edge) * 0.5)]

            cleaned.sort(key=lambda x: x[0])
            merged = []
            for a, b in cleaned:
                if not merged or a > merged[-1][1]:
                    merged.append([a, b])
                else:
                    merged[-1][1] = max(merged[-1][1], b)

            segs = []
            cur = left_edge
            for a, b in merged:
                if a > cur:
                    segs.append((cur, a))
                cur = max(cur, b)
            if cur < right_edge:
                segs.append((cur, right_edge))

            out = []
            for a, b in segs:
                if b > a:
                    w = float(b - a)
                    c = float((a + b) * 0.5)
                    out.append((float(a), float(b), w, c))
            return out
        except Exception:
            return []

    def _is_center_free(self, x, segments, margin=0.0):
        """segments(list of (a,b,w,c)) 중 하나라도 (a+margin) <= x <= (b-margin)을 만족하면 True"""
        try:
            xf = float(x)
        except Exception:
            return False
        m = float(margin)
        for a, b, _w, _c in segments or []:
            if xf >= float(a) + m and xf <= float(b) - m:
                return True
        return False

    def _choose_corridor(self, segments, middle, anchor, curve_direction):
        """
        여러 free segment 중 '가장 안전+안정적'인 corridor를 선택합니다.
        - 폭이 넓을수록 선호
        - anchor(과거 루트)와 멀어질수록 페널티 (휘청거림 억제)
        - middle과도 너무 멀어지지 않게
        Returns: (width, center) or (None, None)
        """
        if not segments:
            return None, None
        try:
            mid = float(middle)
        except Exception:
            mid = 0.0
        try:
            anc = float(anchor)
        except Exception:
            anc = mid
        try:
            cd = float(curve_direction)
        except Exception:
            cd = 0.0

        best_score = 1e18
        best = None
        for a, b, w, c in segments:
            # 너무 좁은 구간은 기본적으로 불리(단, 선택지가 없으면 사용)
            width_pen = 0.0
            if w < 2.6:
                width_pen = (2.6 - w) * 4.0

            score = 0.0
            score += abs(c - anc) * 0.55
            score += abs(c - mid) * 0.25
            score -= w * 0.90
            score += width_pen

            # 코너 방향 바이어스: 역방향으로 급이동하는 라인 억제
            if cd > 6.0 and c < mid - 0.6:
                score += 1.8
            elif cd < -6.0 and c > mid + 0.6:
                score += 1.8

            if score < best_score:
                best_score = score
                best = (a, b, w, c)

        if best is None:
            return None, None
        return float(best[2]), float(best[3])

    def _near_obstacle_clear_target(self, target, middle, nearest_ob_dist, nearest_ob_tm, cnt, half_load_width, dense_obs=False):
        if cnt <= 0 or dense_obs or nearest_ob_dist is None or nearest_ob_tm is None:
            return target
        try:
            dist = float(nearest_ob_dist)
            ob_mid = float(nearest_ob_tm)
            mid = float(middle)
            target = float(target)
            lane_limit = max(1.0, float(half_load_width) - 0.8)
        except Exception:
            return target

        if dist > 35.0:
            return target

        side = 1.0 if mid >= ob_mid else -1.0
        clearance = 3.25 if dist <= 25.0 else 3.0
        desired = ob_mid + side * clearance
        if desired > lane_limit:
            desired = lane_limit
        elif desired < -lane_limit:
            desired = -lane_limit

        if side > 0:
            return max(target, desired)
        return min(target, desired)

    def _early_edge_safe_target(self, target, middle, lap_progress, cnt):
        if cnt <= 0:
            return target
        try:
            target = float(target)
            mid = float(middle)
            progress = float(lap_progress)
        except Exception:
            return target

        if progress > 12.0 or abs(mid) < 6.5:
            return target
        edge_target = 5.5
        if mid < 0:
            return max(target, -edge_target)
        return min(target, edge_target)

    def _finish_edge_safe_target(self, target, middle, lap_progress, speed):
        try:
            target = float(target)
            mid = float(middle)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return target

        if progress < 98.5 or spd < 100.0 or abs(mid) < 6.2:
            return target

        edge_target = 4.6
        if mid > 0:
            return min(target, edge_target)
        return max(target, -edge_target)

    def _apply_finish_edge_guard(self, car_controls, middle, lap_progress, speed):
        try:
            mid = float(middle)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return

        if self.accident_step != 0 or progress < 98.5 or spd < 100.0 or abs(mid) < 6.2:
            return

        guard_steer = min(0.55, 0.34 + (abs(mid) - 6.2) * 0.08)
        if mid > 0:
            car_controls.steering = min(car_controls.steering, -guard_steer)
        else:
            car_controls.steering = max(car_controls.steering, guard_steer)

        car_controls.throttle = 0.0
        car_controls.brake = max(car_controls.brake, 0.45 if abs(mid) > 7.4 else 0.35)

    def _apply_finish_heading_guard(self, car_controls, middle, moving_angle, lap_progress, speed):
        try:
            mid = float(middle)
            angle = float(moving_angle)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return

        if self.accident_step != 0 or progress < 98.8 or spd < 95.0 or abs(angle) < 16.0:
            return

        guard_steer = min(0.75, 0.43 + (abs(angle) - 16.0) * 0.018 + max(0.0, abs(mid) - 4.0) * 0.04)
        if angle > 0:
            car_controls.steering = min(car_controls.steering, -guard_steer)
        else:
            car_controls.steering = max(car_controls.steering, guard_steer)

        car_controls.throttle = 0.0
        car_controls.brake = max(car_controls.brake, 0.45 if spd > 115.0 or abs(mid) > 4.5 else 0.35)

    def _high_speed_edge_safe_target(self, target, middle, speed):
        try:
            target = float(target)
            mid = float(middle)
            spd = float(speed)
        except Exception:
            return target

        if spd < 115.0 or abs(mid) < 6.4:
            return target

        edge_target = 5.0 if abs(mid) < 8.0 else 4.4
        if mid > 0:
            return min(target, edge_target)
        return max(target, -edge_target)

    def _map31_obstacle_edge_target(self, target, middle, nearest_ob_dist, nearest_ob_tm, lap_progress, speed):
        try:
            target = float(target)
            mid = float(middle)
            dist = float(nearest_ob_dist)
            ob_mid = float(nearest_ob_tm)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return target

        if not (64.0 <= progress <= 72.0) or spd < 120.0:
            return target

        edge_target = 3.0
        approach_target = 3.2
        if progress <= 69.4 and dist <= 45.0:
            if ob_mid < -2.5 and mid < 2.8:
                return max(target, approach_target)
            if ob_mid > 2.5 and mid > -2.8:
                return min(target, -approach_target)

        if dist > 25.0:
            return target

        if mid > 3.4 and ob_mid < -2.5:
            return min(target, edge_target)
        if mid < -3.4 and ob_mid > 2.5:
            return max(target, -edge_target)
        return target

    def _apply_map31_obstacle_edge_guard(self, car_controls, middle, moving_angle, nearest_ob_dist, nearest_ob_tm, lap_progress, speed):
        try:
            mid = float(middle)
            angle = float(moving_angle)
            dist = float(nearest_ob_dist)
            ob_mid = float(nearest_ob_tm)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return

        if self.accident_step != 0 or not (64.0 <= progress <= 72.0) or spd < 145.0 or dist > 25.0:
            return

        if mid > 3.4 and ob_mid < -2.5:
            guard_steer = min(0.52, 0.28 + max(0.0, mid - 3.4) * 0.09 + max(0.0, angle - 2.0) * 0.012)
            car_controls.steering = min(car_controls.steering, -guard_steer)
        elif mid < -3.4 and ob_mid > 2.5:
            guard_steer = min(0.52, 0.28 + max(0.0, -mid - 3.4) * 0.09 + max(0.0, -angle - 2.0) * 0.012)
            car_controls.steering = max(car_controls.steering, guard_steer)
        else:
            return

        car_controls.throttle = 0.0
        car_controls.brake = max(car_controls.brake, 0.35 if spd > 155.0 else 0.25)

    def _apply_map31_right_edge_exit_guard(self, car_controls, middle, moving_angle, lap_progress, speed):
        try:
            mid = float(middle)
            angle = float(moving_angle)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return

        if self.accident_step != 0 or not (68.8 <= progress <= 70.5) or spd < 105.0 or mid < 6.6:
            return

        guard_steer = min(0.72, 0.55 + max(0.0, mid - 6.6) * 0.055 + max(0.0, angle - 4.0) * 0.010)
        car_controls.steering = min(car_controls.steering, -guard_steer)
        car_controls.throttle = 0.0
        car_controls.brake = max(car_controls.brake, 0.55 if mid > 7.4 or angle > 10.0 else 0.45)

    def _apply_map31_midcourse_heading_guard(self, car_controls, middle, moving_angle, lap_progress, speed):
        try:
            mid = float(middle)
            angle = float(moving_angle)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return

        if self.accident_step != 0 or not (48.5 <= progress <= 50.6) or spd < 110.0 or abs(angle) < 16.0:
            return

        guard_steer = min(0.72, 0.43 + (abs(angle) - 16.0) * 0.018 + max(0.0, abs(mid) - 2.0) * 0.04)
        if angle > 0:
            car_controls.steering = min(car_controls.steering, -guard_steer)
        else:
            car_controls.steering = max(car_controls.steering, guard_steer)

        car_controls.throttle = 0.0
        car_controls.brake = max(car_controls.brake, 0.45 if spd > 125.0 or abs(mid) > 4.5 else 0.35)

    def _apply_map31_s_cut_exit_guard(self, car_controls, middle, moving_angle, nearest_ob_dist, nearest_ob_tm, lap_progress, speed):
        try:
            mid = float(middle)
            angle = float(moving_angle)
            dist = float(nearest_ob_dist)
            ob_mid = float(nearest_ob_tm)
            progress = float(lap_progress)
            spd = float(speed)
        except Exception:
            return

        if self.accident_step != 0 or not (18.4 <= progress <= 20.1) or spd < 90.0:
            return

        rightward_exit = angle > 14.0 and mid > 2.0 and ob_mid < -2.5 and dist <= 18.0
        if not rightward_exit:
            return

        guard_steer = min(0.70, 0.42 + max(0.0, angle - 14.0) * 0.018 + max(0.0, mid - 2.0) * 0.05)
        car_controls.steering = min(car_controls.steering, -guard_steer)
        car_controls.throttle = 0.0
        car_controls.brake = max(car_controls.brake, 0.35 if spd > 115.0 else 0.25)

    def __init__(self):
        # =========================================================== #
        #  Initialization
        # =========================================================== #
        self.is_debug = False
        self.enable_api_control = True 
        super().set_enable_api_control(self.enable_api_control)

        # AEB(자동긴급제동): 타임어택에 불리해서 기본 OFF
        self.enable_aeb = False


        self.track_type = 99
        self.is_accident = False
        self.recovery_count = 0
        self.accident_count = 0
        self.accident_step = 0
        self.uturn_step = 0
        self.uturn_count = 0

        self.prev_target = 0.0

        # [Anti-wobble route memory] (dense obstacle 구간에서 라인 튐/휘청거림 완화)
        self._route_hold_center = 0.0
        self._route_hold_until = 0.0
        self._route_ema = 0.0
        self._route_ema_inited = False

        
        # [타이머] 차량 출발 시간 기록용
        self.start_time = None 

        # ========================= 로그 (Full Spec) =========================
        self._log_dir = os.path.dirname(os.path.abspath(__file__))
        self.debug_log_file = self._get_next_log_file(self._log_dir, prefix="log_gpt_v", ext=".csv")
        self.debug_log_fp = None
        self.debug_log_writer = None
        self._init_csv_logger()
        self.map_log_file = None  # JSON logging disabled for performance
        # ====================================================================

        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # ========================================================== #
        # [타이머 로직] 차량이 움직이기 시작하면 시간 측정 시작
        # ========================================================== #
        if self.start_time is None:
            if sensing_info.speed > 0.1: # 차량이 조금이라도 움직이면 시작
                self.start_time = datetime.now()
        
        elapsed_seconds = 0
        if self.start_time is not None:
            elapsed_seconds = (datetime.now() - self.start_time).total_seconds()

        
        # ========================== 로깅 메타 (매 tick) ========================== #
        # (20초 전/후 모두 동일 포맷으로 CSV에 저장)
        steer_pre_smooth = None
        steer_delta_raw = None
        max_delta = None
        target_speed = None
        half_load_width = None
        tg = 0
        cnt = 0
        nearest_ob_dist = None
        nearest_ob_tm = None
        is_narrow_gap = False
        obs_in_corner_zone_cnt = 0
        collision_risk_obj = False
        is_hard_corner_ahead = False
        middle_add = 0.0
        avoid_gain = 0.0
        # ======================================================================== #

# ========================================================== #
        # [모드 전환] 20초 이전: v10_2 (안정) / 20초 이후: v10_6 (고속)
        # ========================================================== #
        
        if elapsed_seconds < 20.0:
            # #######################################################################
            # [Phase 1] 초기 20초: v10_2 로직 (안정성 중심)
            # #######################################################################

            half_load_width = self.half_road_limit - 1.5 # v10_2 설정
            car_controls.throttle = 1
            car_controls.brake = 0

            middle = sensing_info.to_middle
            spd = sensing_info.speed
            angles = sensing_info.track_forward_angles
            if not angles: angles = [0.0] * 20

            # 최대 속도 제한 (v10_2: 180)
            if spd > 180:
                car_controls.throttle = 0.0
                if spd > 185: car_controls.brake = 0.2

            # 급커브 예측 브레이킹 (v10_2)
            is_hard_corner_ahead = False
            brake_threshold_speed = 1000 
            check_range = []
            
            if spd >= 150: check_range = range(7, min(len(angles), 12)); brake_threshold_speed = 110 
            elif spd >= 130: check_range = range(5, min(len(angles), 8)); brake_threshold_speed = 100 
            elif spd >= 100: check_range = range(4, min(len(angles), 7)); brake_threshold_speed = 90
            elif spd >= 80: check_range = range(3, min(len(angles), 5)); brake_threshold_speed = 80
            
            if check_range:
                for i in check_range:
                    if abs(angles[i]) > 50:
                        is_hard_corner_ahead = True
                        break
            
            if is_hard_corner_ahead and spd > brake_threshold_speed:
                car_controls.throttle = 0
                if spd > brake_threshold_speed + 15: car_controls.brake = 1.0 
                else: car_controls.brake = 0.6 

            # 장애물 회피 (v10_2)
            ob_start, ob_end = 0, 260 
            full_line = [round(i * 0.1, 1) for i in range(-int(half_load_width) * 10, int(half_load_width) * 10)]
            ob_line_near = full_line[:]
            ob_line_all = full_line[:]
            
            cnt = 0
            nearest_ob_dist = 10**9
            collision_risk_obj = None 

            sorted_obstacles = sorted(sensing_info.track_forward_obstacles, key=lambda x: x['dist'])
            is_narrow_gap = False
            for i in range(len(sorted_obstacles) - 1):
                o1 = sorted_obstacles[i]
                o2 = sorted_obstacles[i+1]
                if o1['dist'] < 60 and o2['dist'] < 60:
                    if abs(o1['dist'] - o2['dist']) < 2.0:
                        gap_width = abs(o1['to_middle'] - o2['to_middle'])
                        if 2.8 < gap_width < 5.5:
                            is_narrow_gap = True
                            break
            ped = 1.7 if is_narrow_gap else 2.4 
            if is_narrow_gap and spd > 95: car_controls.throttle = 0.3 

            for obj in sensing_info.track_forward_obstacles:
                ob_dist, ob_middle = obj['dist'], obj['to_middle']
                if ob_start <= ob_dist <= ob_end:
                    idx = int(ob_dist / 10)
                    if idx >= len(angles): idx = len(angles) - 1
                    local_angle = abs(angles[idx])
                    if local_angle > 50 and ob_dist > 120: continue
                    if ob_dist < nearest_ob_dist:
                        nearest_ob_dist = ob_dist
                        nearest_ob_tm = ob_middle
                        if abs(ob_middle - middle) < 2.5: collision_risk_obj = obj 
                    ob_line_all = [i for i in ob_line_all if not ob_middle - ped <= i <= ob_middle + ped]
                    if ob_dist < 70:
                        ob_line_near = [i for i in ob_line_near if not ob_middle - ped <= i <= ob_middle + ped]
                    cnt += 1

            if ob_line_all: target_candidates = ob_line_all
            else: target_candidates = ob_line_near
            if not target_candidates: target_candidates = full_line[:] 

            edge_margin = self.half_road_limit - 1.5 
            tg_idx = 0
            if spd < 50: tg_idx = 0
            elif spd < 120: tg_idx = 1
            elif spd < 180: tg_idx = 2
            else: tg_idx = 3
            if tg_idx >= len(angles): tg_idx = len(angles) - 1
            curve_direction = angles[tg_idx]

            def target_cost(x):
                cost = abs(x - middle) 
                edge_dist = abs(x)
                use_margin = dense_edge_margin if 'dense_edge_margin' in locals() else edge_margin
                if edge_dist > use_margin: cost += (edge_dist - use_margin) ** 2 * 6.0
                if curve_direction > 5 and x < middle - 0.5: cost += 5.0 
                elif curve_direction < -5 and x > middle + 0.5: cost += 5.0
                return cost

            target = min(target_candidates, key=target_cost)
            target = self._near_obstacle_clear_target(
                target,
                middle,
                nearest_ob_dist,
                nearest_ob_tm,
                cnt,
                half_load_width,
            )
            target = self._early_edge_safe_target(
                target,
                middle,
                sensing_info.lap_progress,
                cnt,
            )
            self.target_offset = target

            p = -(middle - target) * 0.07 
            i = p ** 2 * 0.05 if p >= 0 else - p ** 2 * 0.05
            middle_add = 0.5 * p + 0.4 * i
            if cnt == 0: middle_add = 0.0

            avoid_gain = 1.0
            if cnt > 0:
                if nearest_ob_dist <= 25: avoid_gain = 2.2 
                elif nearest_ob_dist <= 40: avoid_gain = 1.8 
                elif nearest_ob_dist <= 60: avoid_gain = 1.4 
                elif nearest_ob_dist <= 90: avoid_gain = 1.1 

            if cnt > 0 and spd > 130 and nearest_ob_dist < 60 and collision_risk_obj:
                car_controls.throttle = 0.0
                avoid_gain *= 1.2 

            if cnt > 0:
                react_dist = max(80.0, min(150.0, spd * 0.8))
                if nearest_ob_dist > react_dist:
                    middle_add = 0.0
                    avoid_gain = 1.0
                    cnt = 0

            # [BUG FIX] tg 변수 정의 추가 (v10_2 로직)
            if spd < 50 and sensing_info.lap_progress > 1: tg = 0
            elif spd < 120: tg = 1
            elif spd < 180: tg = 2
            else: tg = 3
            if tg >= len(angles): tg = len(angles) - 1
            if tg < 0: tg = 0

            # Steering v10_2
            if abs(angles[tg]) < 55:
                if cnt == 0:
                    if spd < 70: base_factor = 60
                    elif spd < 120: base_factor = 85 
                    else: base_factor = 100 
                    car_controls.steering = (angles[tg] - sensing_info.moving_angle) / base_factor
 
                    # [Edge Recovery] When we're far from center, strengthen center correction (reduce wall riding time)
 
                    center_div = 80.0
 
                    if abs(middle) > (self.half_road_limit - 3.0):
 
                        center_div = 45.0
 
                    elif abs(middle) > (self.half_road_limit - 4.5):
 
                        center_div = 60.0
 
                    car_controls.steering -= (middle / center_div)
                else:
                    if nearest_ob_dist < 25 and collision_risk_obj: base_steer_factor = 60 
                    elif spd < 70: base_steer_factor = 60
                    else: base_steer_factor = 90
                    set_steering = (angles[tg] - sensing_info.moving_angle) / base_steer_factor
                    car_controls.steering = set_steering

                    final_add = (middle_add * avoid_gain)


                    # [Wall-aware damping] Don't push further into the wall when already near the edge

                    if abs(middle) > (self.half_road_limit - 2.2):
                        if middle * final_add > 0:
                            final_add *= 0.2


                    car_controls.steering += final_add
            else:
                k = spd if spd >= 60 else 60
                if angles[tg] < 0:
                    r = self.half_road_limit - 1.25 + middle
                    beta = - math.pi * k * 0.1 / r
                    car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if angles[tg] > -60 else -1
                else:
                    r = self.half_road_limit - 1.25 - middle
                    beta = math.pi * k * 0.1 / r
                    car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if angles[tg] < 60 else 1
                if spd > 90: 
                    car_controls.throttle = -1
                    car_controls.brake = 1
                if cnt > 0:
                    curve_scale = 0.40
                    if nearest_ob_dist <= 45: curve_scale = 0.65
                    car_controls.steering += (middle_add * avoid_gain) * curve_scale
                    if car_controls.steering > 1: car_controls.steering = 1
                    elif car_controls.steering < -1: car_controls.steering = -1

            ang_idx = int(spd // 20)
            if ang_idx >= len(angles): ang_idx = len(angles) - 1
            if not is_hard_corner_ahead and cnt == 0 and (abs(angles[ang_idx]) > 45 or abs(middle) > 10) and spd > 120:
                car_controls.throttle = 0
                car_controls.brake = 0.3
            if spd > 180 and abs(angles[-1]) > 10:
                car_controls.throttle = -0.5
                car_controls.brake = 1
            close_obstacle_active = (
                cnt > 0
                and nearest_ob_dist is not None
                and nearest_ob_dist <= 12
            )
            if spd < 5 and not close_obstacle_active and self.accident_step == 0:
                car_controls.steering = 0

            if abs(sensing_info.moving_angle) > 30 and spd > 80:
                car_controls.throttle = 0.0
                car_controls.brake = max(car_controls.brake, 1.0)

            #aeb사용안함
            # if collision_risk_obj: 
            #     need_aeb = False
            #     if spd >= 120 and nearest_ob_dist < 60: need_aeb = True
            #     elif spd >= 80 and nearest_ob_dist < 40: need_aeb = True
            #     elif nearest_ob_dist < 18: need_aeb = True
            #     if need_aeb:
            #         car_controls.throttle = -1
            #         car_controls.brake = 1.0 

            # Target Speed v10_2
            if self.accident_step == 0 and not is_hard_corner_ahead: 
                abs_ang = abs(angles[tg]) if angles else 0.0
                if abs_ang < 3: base_target = 210
                elif abs_ang < 7: base_target = 190
                elif abs_ang < 15: base_target = 160
                elif abs_ang < 25: base_target = 140
                elif abs_ang < 35: base_target = 120
                else: base_target = 100
                offset = abs(middle)
                if offset > 8: base_target = min(base_target, 110)
                target_speed = max(100, base_target)
                if car_controls.brake < 0.1: 
                    if spd < 180: 
                        if spd < target_speed - 5: car_controls.throttle = 1.0
                        elif spd > target_speed + 5: car_controls.throttle = 0.0; car_controls.brake = 0.5
                        else: car_controls.throttle = 0.7
                    else: car_controls.throttle = 0.0

            close_risk_brake = (
                self.accident_step == 0
                and cnt > 0
                and bool(collision_risk_obj)
                and nearest_ob_dist is not None
                and nearest_ob_dist <= 18.0
                and spd > 45.0
            )
            if close_risk_brake:
                car_controls.throttle = 0.0
                car_controls.brake = max(car_controls.brake, 0.65 if nearest_ob_dist <= 8.0 else 0.45)

        else:
            # #######################################################################
            # [Phase 2] 20초 이후: v10_6 로직 (고속/최적화) + 튜닝
            # #######################################################################

            half_load_width = self.half_road_limit - 1.2 # v10_6 설정
            car_controls.throttle = 1
            car_controls.brake = 0

            middle = sensing_info.to_middle
            spd = sensing_info.speed
            angles = sensing_info.track_forward_angles or [0.0] * 20
            obstacles = sensing_info.track_forward_obstacles

            # [v10_6] 최대 속도 제한 (post-20s only, safety/penalty-aware)
            MAX_SPEED = 170.0
            if spd > MAX_SPEED:
                car_controls.throttle = 0.0
                # bleed speed smoothly; stronger if far above limit
                if spd > MAX_SPEED + 7.0:
                    car_controls.brake = max(car_controls.brake, 0.25)
                elif spd > MAX_SPEED + 3.0:
                    car_controls.brake = max(car_controls.brake, 0.12)

            # [v10_6 튜닝] 급커브 예측: 시야 확장 적용 (12 -> 15)
            is_hard_corner_ahead = False
            brake_threshold_speed = 1000 
            check_range = []
            
            if spd >= 150: 
                # [수정] 기존 range(7, 12) -> range(7, 15)로 30m 확장
                check_range = range(7, min(len(angles), 15))
                brake_threshold_speed = 110 
            elif spd >= 130: check_range = range(5, min(len(angles), 8)); brake_threshold_speed = 100 
            elif spd >= 100: check_range = range(4, min(len(angles), 7)); brake_threshold_speed = 90
            elif spd >= 80: check_range = range(3, min(len(angles), 5)); brake_threshold_speed = 80
            
            if check_range:
                for i in check_range:
                    if abs(angles[i]) > 50:
                        is_hard_corner_ahead = True
                        break
            
            if is_hard_corner_ahead and spd > brake_threshold_speed:
                car_controls.throttle = 0
                if float(sensing_info.lap_progress) >= 88.0 and spd <= 105.0:
                    car_controls.brake = 0.3
                elif spd > brake_threshold_speed + 15: car_controls.brake = 1.0 
                else: car_controls.brake = 0.6 

            # [v10_6] 장애물 회피
            ob_start, ob_end = 0, 260 
            full_line = [round(i * 0.1, 1) for i in range(-int(half_load_width) * 10, int(half_load_width) * 10)]
            ob_line_near = full_line[:]
            ob_line_all = full_line[:]
            
            cnt = 0
            nearest_ob_dist = 10**9
            collision_risk_obj = None 

            sorted_obstacles = sorted(sensing_info.track_forward_obstacles, key=lambda x: x['dist'])
            is_narrow_gap = False
            for i in range(len(sorted_obstacles) - 1):
                o1 = sorted_obstacles[i]
                o2 = sorted_obstacles[i+1]
                if o1['dist'] < 60 and o2['dist'] < 60:
                    if abs(o1['dist'] - o2['dist']) < 2.0:
                        gap_width = abs(o1['to_middle'] - o2['to_middle'])
                        if 2.8 < gap_width < 5.5:
                            is_narrow_gap = True
                            break
            ped = 2.1 if is_narrow_gap else 2.5 
            if is_narrow_gap and spd > 95: car_controls.throttle = 0.3 

            obs_in_corner_zone_cnt = 0
            for obj in obstacles:
                if 0 <= obj['dist'] <= 25: obs_in_corner_zone_cnt += 1

            for obj in obstacles:
                ob_dist, ob_middle = obj['dist'], obj['to_middle']
                if ob_start <= ob_dist <= ob_end:
                    idx = int(ob_dist / 10)
                    if idx >= len(angles): idx = len(angles) - 1
                    if abs(angles[idx]) > 50 and ob_dist > 120: continue
                    if ob_dist < nearest_ob_dist:
                        nearest_ob_dist = ob_dist
                        nearest_ob_tm = ob_middle
                        if abs(ob_middle - middle) < 2.5: collision_risk_obj = obj 
                    ob_line_all = [i for i in ob_line_all if not ob_middle - ped <= i <= ob_middle + ped]
                    if ob_dist < 70:
                        ob_line_near = [i for i in ob_line_near if not ob_middle - ped <= i <= ob_middle + ped]
                    cnt += 1

            if ob_line_all: target_candidates = ob_line_all
            else: target_candidates = ob_line_near
            if not target_candidates: target_candidates = full_line[:] 

            edge_margin = self.half_road_limit - 1.5 
            tg_idx = 0
            if spd < 50: tg_idx = 0
            elif spd < 120: tg_idx = 1
            elif spd < 180: tg_idx = 2
            else: tg_idx = 3
            if tg_idx >= len(angles): tg_idx = len(angles) - 1
            curve_direction = angles[tg_idx]

            # ================= Dense obstacle mode (후반 다중 장애물 구간 보완) =================
            # - obs_cnt가 높거나(>=8), 60m 이내 양쪽에 장애물이 동시에 많으면 '가장 넓은 갭'을 더 강하게 선호
            obs_cnt_60 = 0
            has_left = False
            has_right = False
            for _o in obstacles:
                try:
                    _d = float(_o.get('dist', 0.0))
                    if _d <= 60.0:
                        obs_cnt_60 += 1
                        _tm = float(_o.get('to_middle', 0.0))
                        if _tm < middle:
                            has_left = True
                        elif _tm > middle:
                            has_right = True
                except Exception:
                    continue

            # ===== Dense obstacle awareness =====
            obs_cnt_80 = 0
            if obstacles:
                for _o in obstacles:
                    try:
                        _d = float(_o.get('dist', 1e9))
                    except Exception:
                        continue
                    if 0.0 < _d <= 80.0:
                        obs_cnt_80 += 1

            dense_obs = False
            # (1) 매우 많은 장애물
            if cnt >= 6:
                dense_obs = True
            # (2) 80m 이내에 장애물이 촘촘하게 분포
            elif obs_cnt_80 >= 6:
                dense_obs = True
            # (3) 코너 진입 직전(25m 이내)에 장애물이 여러 개
            elif obs_in_corner_zone_cnt >= 3:
                dense_obs = True
            # (4) 60m 이내에 좌/우 모두 장애물이 존재하면서 개수가 충분
            elif obs_cnt_60 >= 4 and has_left and has_right:
                dense_obs = True

            # dense일 때는 벽에 붙는 라인을 조금 더 억제(벽 충돌 감소)
            dense_edge_margin = edge_margin
            if dense_obs:
                dense_edge_margin = self.half_road_limit - 2.7  # 기존보다 안전 여유 증가
                if dense_edge_margin < 7.2:
                    dense_edge_margin = 7.2

            # 가장 넓은 갭(near window) 계산 -> 갭 중앙 후보를 비용에 반영
            gap_center = None
            gap_width = None
            if dense_obs and obstacles:
                try:
                    # nearest 기준 ±15m 윈도우 (주로 '지금 곧 지나갈' 클러스터에 집중)
                    _nearest = min(obstacles, key=lambda x: float(x.get('dist', 1e9)))
                    _nd = float(_nearest.get('dist', 1e9))
                    _window = [o for o in obstacles
                               if float(o.get('dist', 1e9)) <= 85.0 and abs(float(o.get('dist', 1e9)) - _nd) <= 18.0]
                    gap_width, gap_center = self._calc_best_gap(_window, ped, half_load_width)
                except Exception:
                    gap_width, gap_center = None, None

            # ----- Gap center smoothing + next-route (lookahead) -----
            gap_center_ema = None
            if gap_center is not None:
                try:
                    _prev_gc = float(getattr(self, "_gap_center_ema", float(gap_center)))
                except Exception:
                    _prev_gc = float(gap_center)
                a_gc = 0.10 if spd > 130 else 0.18
                gap_center_ema = (1.0 - a_gc) * _prev_gc + a_gc * float(gap_center)
                self._gap_center_ema = gap_center_ema

            far_gap_center = None
            if dense_obs and obstacles:
                try:
                    _far_window = [o for o in obstacles if 90.0 < float(o.get("dist", 1e9)) <= 150.0]
                    if _far_window:
                        _, far_gap_center = self._calc_best_gap(_far_window, max(1.8, float(ped) - 0.2), half_load_width)
                except Exception:
                    far_gap_center = None

            far_gap_center_ema = None
            if far_gap_center is not None:
                try:
                    _prev_fg = float(getattr(self, "_far_gap_center_ema", float(far_gap_center)))
                except Exception:
                    _prev_fg = float(far_gap_center)
                a_fg = 0.08 if spd > 130 else 0.14
                far_gap_center_ema = (1.0 - a_fg) * _prev_fg + a_fg * float(far_gap_center)
                self._far_gap_center_ema = far_gap_center_ema
            # ---------------------------------------------------------

            # dense soft-repulse 리스트(너무 많은 연산 방지: 90m 이내만)
            dense_repulse_obs = []
            if dense_obs:
                for _o in obstacles:
                    try:
                        _d = float(_o.get('dist', 0.0))
                        if _d <= 90.0:
                            dense_repulse_obs.append((float(_d), float(_o.get('to_middle', 0.0))))
                    except Exception:
                        continue

            # ====================================================================================
            # [v10_6 튜닝] target_cost 가중치 변경 (5.0 -> 4.5)
            # ----- Outer-lane preference (only in dense clusters) -----
            outer_pref = None
            outer_weight = 0.0
            if 'dense_obs' in locals() and dense_obs:
                try:
                    _use_margin = dense_edge_margin if 'dense_edge_margin' in locals() else edge_margin
                    outer_mag = min(10.5, float(_use_margin) - 0.4)
                    if outer_mag > 0:
                        # out-in-out: prefer outside of the upcoming turn to increase corner radius
                        if curve_direction > 4.0:
                            outer_pref = -outer_mag
                        elif curve_direction < -4.0:
                            outer_pref = outer_mag
                        else:
                            outer_pref = outer_mag if float(getattr(self, "prev_target", 0.0)) >= 0.0 else -outer_mag

                        outer_weight = 0.10 if abs(float(curve_direction)) < 18.0 else 0.06
                        if cnt >= 8:
                            outer_weight += 0.05
                except Exception:
                    outer_pref = None
                    outer_weight = 0.0
            # ---------------------------------------------------------

            def target_cost(x):
                cost = abs(x - middle) 
                # cost += abs(x - self.prev_target) * 0.5  # disabled: caused edge-locking in obstacle zones 
                edge_dist = abs(x)
                use_margin = dense_edge_margin if 'dense_edge_margin' in locals() else edge_margin
                if edge_dist > use_margin: cost += (edge_dist - use_margin) ** 2 * 6.0

                # [Soft edge guard] corner/dense 상황에서 '마진 안쪽'에서도 벽에 너무 붙는 라인을 억제
                if (is_hard_corner_ahead or abs(curve_direction) > 25 or ('dense_obs' in locals() and dense_obs)):
                    soft_edge = max(6.5, float(use_margin) - 1.4)
                    if abs(float(x)) > soft_edge:
                        cost += ((abs(float(x)) - soft_edge) ** 2) * 3.0

                    # [Corner inner-wall guard] 강한 커브에서 '안쪽 벽' 쪽 라인을 과하게 타지 않도록
                    inner_guard = max(5.5, float(use_margin) - 2.2)
                    if curve_direction > 8 and float(x) > inner_guard:
                        cost += ((float(x) - inner_guard) ** 2) * 9.0
                    elif curve_direction < -8 and float(x) < -inner_guard:
                        cost += ((-float(x) - inner_guard) ** 2) * 9.0

                    # [Outer preference] when obstacles are dense, softly bias to a stable outside lane
                    if outer_pref is not None and outer_weight > 0.0:
                        try:
                            cost += abs(float(x) - float(outer_pref)) * float(outer_weight)
                        except Exception:
                            pass

                
                # [수정] 5.0 -> 4.5
                if cnt == 0:
                    if curve_direction > 5 and x < middle - 0.5: cost += 4.5 
                    elif curve_direction < -5 and x > middle + 0.5: cost += 4.5
                
                # ===== Dense mode extras =====
                if 'dense_obs' in locals() and dense_obs:
                    # (1) 갭 중앙(있으면) 선호: 너무 급격한 타겟 점프를 줄이면서도 통과 가능한 라인 유지
                    if gap_center_ema is not None:
                        # EMA-smoothed gap center (anti-jitter) - keep inside a feasible corridor
                        cost += abs(float(x) - float(gap_center_ema)) * 0.22
                    elif gap_center is not None:
                        cost += abs(float(x) - float(gap_center)) * 0.20

                    # (1b) lookahead gap (next-route) - small weight
                    if far_gap_center_ema is not None:
                        cost += abs(float(x) - float(far_gap_center_ema)) * 0.10
                    elif far_gap_center is not None:
                        cost += abs(float(x) - float(far_gap_center)) * 0.08

                    # (2) 장애물 경계 근접 soft-repulse: ped 바로 바깥을 스치며 부딪히는 경우 방지
                    repulse_safe = float(ped) + (0.55 if spd > 125 else 0.45)
                    for _d, _tm in dense_repulse_obs:
                        lat = abs(float(x) - float(_tm))
                        if lat < repulse_safe:
                            w = min(4.0, max(0.8, 70.0 / (_d + 10.0)))
                            cost += ((repulse_safe - lat) ** 2) * w
                # =============================
                return cost

            target_raw = min(target_candidates, key=target_cost)

            # ----- Corridor / Anti-wobble (multi-obstacle) -----
            route_hold_center = None
            route_ema = None
            if dense_obs and obstacles:
                try:
                    _nearest = min(obstacles, key=lambda x: float(x.get('dist', 1e9)))
                    _nd = float(_nearest.get('dist', 1e9))
                    window_obs = [o for o in obstacles
                                  if float(o.get('dist', 1e9)) <= 85.0 and abs(float(o.get('dist', 1e9)) - _nd) <= 18.0]
                    segs = self._calc_free_segments(window_obs, ped, half_load_width)

                    anchor = self.prev_target
                    if getattr(self, '_route_ema_inited', False):
                        anchor = getattr(self, '_route_ema', self.prev_target)

                    corridor_width, corridor_center = self._choose_corridor(segs, middle, anchor, curve_direction)

                    # gap_center가 있으면 약하게 혼합(좁은 갭 통과 안정성)
                    if gap_center is not None:
                        if corridor_center is None:
                            corridor_center = float(gap_center_ema) if gap_center_ema is not None else float(gap_center)
                        else:
                            # blend near-gap center (EMA if available) and a small lookahead hint
                            _gmix = float(gap_center_ema) if gap_center_ema is not None else float(gap_center)
                            if far_gap_center_ema is not None:
                                _fgmix = float(far_gap_center_ema)
                                corridor_center = 0.62 * float(corridor_center) + 0.28 * _gmix + 0.10 * _fgmix
                            elif far_gap_center is not None:
                                _fgmix = float(far_gap_center)
                                corridor_center = 0.64 * float(corridor_center) + 0.28 * _gmix + 0.08 * _fgmix
                            else:
                                corridor_center = 0.66 * float(corridor_center) + 0.34 * _gmix

                    if corridor_center is not None:
                        # hysteresis: 일정 시간(0.7~0.9s) 동일 corridor 유지
                        hold_margin = float(ped) + (0.45 if spd > 120 else 0.35)
                        hold_valid = True
                        if segs:
                            hold_valid = self._is_center_free(getattr(self, '_route_hold_center', corridor_center), segs, margin=hold_margin)

                        if float(elapsed_seconds) < float(getattr(self, '_route_hold_until', 0.0)) and hold_valid:
                            corridor_center = float(getattr(self, '_route_hold_center', corridor_center))
                        else:
                            # corridor_center가 segment 밖이면 가장 가까운 segment center로 스냅
                            if segs and not self._is_center_free(corridor_center, segs, margin=hold_margin):
                                corridor_center = min(segs, key=lambda s: abs(float(s[3]) - float(corridor_center)))[3]

                            hold_dur = 0.90 if spd > 135 else 0.70
                            self._route_hold_center = float(corridor_center)
                            self._route_hold_until = float(elapsed_seconds) + float(hold_dur)

                        # step-limit + EMA: 타겟 점프 자체를 줄여 휘청거림/벽충돌 감소
                        alpha = 0.86 if spd > 120 else 0.80
                        if not getattr(self, '_route_ema_inited', False):
                            self._route_ema = float(corridor_center)
                            self._route_ema_inited = True
                        else:
                            max_step = 0.65 if spd > 120 else 0.90
                            diff = float(corridor_center) - float(getattr(self, '_route_ema', 0.0))
                            if diff > max_step:
                                corridor_center = float(getattr(self, '_route_ema', 0.0)) + max_step
                            elif diff < -max_step:
                                corridor_center = float(getattr(self, '_route_ema', 0.0)) - max_step

                            # 너무 크게 벌어지면(상황이 바뀜) EMA를 리셋
                            if abs(diff) > 4.0:
                                self._route_ema = float(corridor_center)
                            else:
                                self._route_ema = alpha * float(getattr(self, '_route_ema', 0.0)) + (1.0 - alpha) * float(corridor_center)

                        route_hold_center = float(getattr(self, '_route_hold_center', corridor_center))
                        route_ema = float(getattr(self, '_route_ema', corridor_center))
                except Exception:
                    route_hold_center = None
                    route_ema = None
            # -----------------------------------------------

            target = target_raw
            if route_ema is not None and target_candidates:
                # corridor EMA에 가장 가까운 안전 candidate로 스냅
                target = min(target_candidates, key=lambda x: abs(float(x) - float(route_ema)))
            target = self._near_obstacle_clear_target(
                target,
                middle,
                nearest_ob_dist,
                nearest_ob_tm,
                cnt,
                half_load_width,
                dense_obs=('dense_obs' in locals() and dense_obs),
            )
            target = self._high_speed_edge_safe_target(
                target,
                middle,
                spd,
            )
            target = self._map31_obstacle_edge_target(
                target,
                middle,
                nearest_ob_dist,
                nearest_ob_tm,
                sensing_info.lap_progress,
                spd,
            )

            s_cut_active = (
                16.5 <= float(sensing_info.lap_progress) <= 20.0
                and nearest_ob_tm is not None
                and nearest_ob_dist is not None
                and nearest_ob_dist <= 60.0
                and float(nearest_ob_tm) < -2.0
            )
            if s_cut_active:
                s_cut_min = 1.15
                s_cut_max = min(float(half_load_width) - 1.0, 3.2)
                target = min(max(float(middle), s_cut_min), s_cut_max)

            target = self._finish_edge_safe_target(
                target,
                middle,
                sensing_info.lap_progress,
                spd,
            )

            self.target_offset = target
            self.prev_target = target

            p = -(middle - target) * 0.07 
            i = p ** 2 * 0.05 if p >= 0 else - p ** 2 * 0.05
            middle_add = 0.5 * p + 0.4 * i
            if cnt == 0: middle_add = 0.0

            avoid_gain = 1.0
            if cnt > 0:
                if nearest_ob_dist <= 25: avoid_gain = 2.2 
                elif nearest_ob_dist <= 40: avoid_gain = 1.8 
                elif nearest_ob_dist <= 60: avoid_gain = 1.4 
                elif nearest_ob_dist <= 90: avoid_gain = 1.1 

            if cnt > 0 and spd > 130 and nearest_ob_dist < 60 and collision_risk_obj:
                car_controls.throttle = 0.0
                avoid_gain *= 1.2 

            if cnt > 0:
                react_dist = max(80.0, min(150.0, spd * 0.8))
                if nearest_ob_dist > react_dist:
                    middle_add = 0.0
                    avoid_gain = 1.0
                    cnt = 0

            # Steering v10_6
            if spd < 50 and sensing_info.lap_progress > 1: tg = 0
            elif spd < 120: tg = 1
            elif spd < 180: tg = 2
            else: tg = 3
            if tg >= len(angles): tg = len(angles) - 1
            if tg < 0: tg = 0

            if abs(angles[tg]) < 55:
                if cnt == 0:
                    if spd < 70: base_factor = 60
                    elif spd < 120: base_factor = 85 
                    else: base_factor = 100 
                    car_controls.steering = (angles[tg] - sensing_info.moving_angle) / base_factor
 
                    # [Edge Recovery] When we're far from center, strengthen center correction (reduce wall riding time)
 
                    center_div = 80.0
 
                    if abs(middle) > (self.half_road_limit - 3.0):
 
                        center_div = 45.0
 
                    elif abs(middle) > (self.half_road_limit - 4.5):
 
                        center_div = 60.0
 
                    car_controls.steering -= (middle / center_div)
                else:
                    # heading_err는 항상 정의 (UnboundLocalError 방지)
                    heading_err = (angles[tg] - sensing_info.moving_angle)

                    if (nearest_ob_dist is not None) and (nearest_ob_dist < 25) and collision_risk_obj:
                        base_steer_factor = 60
                    elif spd < 70:
                        base_steer_factor = 60
                    else:
                        base_steer_factor = 90

                    # straight/완만 구간에서 헤딩 오차가 큰데 벽쪽으로 밀리는 케이스(93~94%, 22~24%) 보정
                    if abs(heading_err) > 10 and abs(angles[tg]) < 25 and spd > 120:
                        base_steer_factor = min(base_steer_factor, 65)

                    set_steering = heading_err / base_steer_factor
                    final_add = middle_add * avoid_gain

                    # [Edge recovery in obstacle mode] 장애물 중에도 벽에 붙으면 센터로 끌어당김
                    center_pull = 0.0
                    if abs(middle) > 8.5:
                        center_div_obs = 160.0
                        if abs(middle) > (self.half_road_limit - 4.5):
                            center_div_obs = 110.0
                        if abs(middle) > (self.half_road_limit - 3.2):
                            center_div_obs = 80.0
                        if abs(middle) > (self.half_road_limit - 2.4):
                            center_div_obs = 55.0
                        if is_hard_corner_ahead or abs(curve_direction) > 25:
                            center_div_obs *= 0.8

                        center_pull = -(middle / center_div_obs)

                        # center_pull 방향에 가까운 장애물이 있으면(근거리) pull 약화
                        if (nearest_ob_tm is not None) and (nearest_ob_dist is not None) and (nearest_ob_dist < 35):
                            if middle > 0 and (nearest_ob_tm < middle) and (abs(nearest_ob_tm - middle) < 4.0):
                                center_pull *= 0.35
                            elif middle < 0 and (nearest_ob_tm > middle) and (abs(nearest_ob_tm - middle) < 4.0):
                                center_pull *= 0.35

                    # [Wall-aware damping] Don't push further into the wall when already near the edge
                    if abs(middle) > (self.half_road_limit - 2.2):
                        if middle * final_add > 0:
                            final_add *= 0.2
                    if abs(curve_direction) > 50 and cnt > 0: final_add *= 0.5 
                    car_controls.steering = set_steering + final_add + center_pull
            else:
                k = spd if spd >= 60 else 60
                if angles[tg] < 0:
                    r = self.half_road_limit - 1.25 + middle
                    beta = - math.pi * k * 0.1 / r
                    car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if angles[tg] > -60 else -1
                else:
                    r = self.half_road_limit - 1.25 - middle
                    beta = math.pi * k * 0.1 / r
                    car_controls.steering = (beta - sensing_info.moving_angle * math.pi / 180) if angles[tg] < 60 else 1
                if spd > 90: 
                    car_controls.throttle = -1
                    car_controls.brake = 1
                if cnt > 0:
                    curve_scale = 0.40
                    if nearest_ob_dist <= 45: curve_scale = 0.65
                    car_controls.steering += (middle_add * avoid_gain) * curve_scale
                    if car_controls.steering > 1: car_controls.steering = 1
                    elif car_controls.steering < -1: car_controls.steering = -1

            ang_idx = int(spd // 20)
            if ang_idx >= len(angles): ang_idx = len(angles) - 1
            if not is_hard_corner_ahead and cnt == 0 and (abs(angles[ang_idx]) > 45 or abs(middle) > 10) and spd > 120:
                car_controls.throttle = 0
                car_controls.brake = 0.3
            if spd > 180 and abs(angles[-1]) > 10:
                car_controls.throttle = -0.5
                car_controls.brake = 1
            close_obstacle_active = (
                cnt > 0
                and nearest_ob_dist is not None
                and nearest_ob_dist <= 12
            )
            if spd < 5 and not close_obstacle_active and self.accident_step == 0:
                car_controls.steering = 0

            if abs(sensing_info.moving_angle) > 30 and spd > 80:
                car_controls.throttle = 0.0
                car_controls.brake = max(car_controls.brake, 1.0)

            if self.enable_aeb and collision_risk_obj and cnt == 0: 
                need_aeb = False
                if spd >= 120 and nearest_ob_dist < 60: need_aeb = True
                elif spd >= 80 and nearest_ob_dist < 40: need_aeb = True
                elif nearest_ob_dist < 18: need_aeb = True
                if need_aeb:
                    car_controls.throttle = -1
                    car_controls.brake = 1.0

            # Target Speed v10_6
            if self.accident_step == 0 and not is_hard_corner_ahead: 
                abs_ang = abs(angles[tg]) if angles else 0.0
                if abs_ang < 3: base_target = 170
                elif abs_ang < 7: base_target = 160
                elif abs_ang < 15: base_target = 145
                elif abs_ang < 25: base_target = 130
                elif abs_ang < 35: base_target = 115
                else: base_target = 100
                offset = abs(middle)
                if offset > 8: base_target = min(base_target, 140)
                target_speed = max(90, base_target)
                # hard cap
                target_speed = min(target_speed, 170)
                # reduce target speed when large steering demand (friction-circle style)
                try:
                    _abs_st = abs(float(getattr(car_controls, "steering", 0.0)))
                except Exception:
                    _abs_st = 0.0
                if _abs_st > 0.65 and spd > 115:
                    target_speed = min(target_speed, 125)
                elif _abs_st > 0.55 and spd > 125:
                    target_speed = min(target_speed, 135)
                if obs_in_corner_zone_cnt >= 3: target_speed -= 20 
                target_speed = max(target_speed, 60)
                # [Edge risk speed cap] 벽 근접 + 코너/장애물 구간에서 안전마진 확보
                if (is_hard_corner_ahead or abs(curve_direction) > 25 or ('dense_obs' in locals() and dense_obs)):
                    if offset > 10.5:
                        target_speed = min(target_speed, 120)
                    if offset > 11.5:
                        target_speed = min(target_speed, 95)

                if car_controls.brake < 0.1: 
                    if spd < 180: 
                        if spd < target_speed - 5: car_controls.throttle = 1.0
                        elif spd > target_speed + 5:
                            car_controls.throttle = 0.0
                            car_controls.brake = 0.35 if float(sensing_info.lap_progress) >= 30.0 else 0.5
                        else: car_controls.throttle = 0.7
                    else: car_controls.throttle = 0.0

                if s_cut_active and self.accident_step == 0:
                    car_controls.throttle = 1.0
                    car_controls.brake = 0.0
                    if car_controls.steering > 0.22:
                        car_controls.steering = 0.22
                    elif car_controls.steering < -0.12:
                        car_controls.steering = -0.12

                high_speed_edge = spd > 120.0 and abs(float(middle)) > 7.0
                mid_speed_edge = spd > 80.0 and abs(float(middle)) > 8.6
                edge_guard_active = (
                    self.accident_step == 0
                    and not s_cut_active
                    and (high_speed_edge or mid_speed_edge)
                )
                if edge_guard_active:
                    edge_base = 7.0 if high_speed_edge else 8.6
                    guard_steer = min(0.60, 0.34 + max(0.0, abs(float(middle)) - edge_base) * 0.055)
                    if middle > 0:
                        car_controls.steering = min(car_controls.steering, -guard_steer)
                    else:
                        car_controls.steering = max(car_controls.steering, guard_steer)
                    car_controls.throttle = 0.0
                    car_controls.brake = max(car_controls.brake, 0.45 if abs(float(middle)) > 10.5 else 0.35)

            close_risk_brake = (
                self.accident_step == 0
                and cnt > 0
                and bool(collision_risk_obj)
                and nearest_ob_dist is not None
                and nearest_ob_dist <= 28.0
                and spd > 95.0
            )
            if close_risk_brake:
                car_controls.throttle = 0.0
                car_controls.brake = max(car_controls.brake, 0.55 if nearest_ob_dist <= 12.0 else 0.35)

            self._apply_map31_obstacle_edge_guard(
                car_controls,
                middle,
                sensing_info.moving_angle,
                nearest_ob_dist,
                nearest_ob_tm,
                sensing_info.lap_progress,
                spd,
            )
            self._apply_map31_right_edge_exit_guard(
                car_controls,
                middle,
                sensing_info.moving_angle,
                sensing_info.lap_progress,
                spd,
            )
            self._apply_map31_midcourse_heading_guard(
                car_controls,
                middle,
                sensing_info.moving_angle,
                sensing_info.lap_progress,
                spd,
            )
            self._apply_map31_s_cut_exit_guard(
                car_controls,
                middle,
                sensing_info.moving_angle,
                nearest_ob_dist,
                nearest_ob_tm,
                sensing_info.lap_progress,
                spd,
            )
            self._apply_finish_edge_guard(
                car_controls,
                middle,
                sensing_info.lap_progress,
                spd,
            )
            self._apply_finish_heading_guard(
                car_controls,
                middle,
                sensing_info.moving_angle,
                sensing_info.lap_progress,
                spd,
            )

        # ========================================================== #
        # 공통 처리 (스무딩, 사고 복구, 로그)
        # ========================================================== #
        global before
        max_delta = 0.15
        if cnt > 0:
            if nearest_ob_dist is not None and nearest_ob_dist <= 30: max_delta = 1.0
            elif nearest_ob_dist is not None and nearest_ob_dist <= 50: max_delta = 0.50
            elif nearest_ob_dist is not None and nearest_ob_dist <= 80: max_delta = 0.35
            else: max_delta = 0.25

        collided_now = bool(getattr(sensing_info, "collided", False))
        close_collision = collided_now and (
            (nearest_ob_dist is not None and nearest_ob_dist <= 8.0)
            or abs(spd) <= 25.0
        )
        if close_collision and self.accident_step == 0:
            self.accident_step = 1
            self.accident_count = max(self.accident_count, 9)
            self.recovery_count = 0

        # [Edge Recovery] allow faster steering change near road edge / during recovery
        _mid_now = float(getattr(sensing_info, "to_middle", 0.0))
        if abs(_mid_now) > 9.5:
            max_delta = max(max_delta, 0.35)
        if is_hard_corner_ahead and abs(_mid_now) > 8.0:
            max_delta = max(max_delta, 0.5)
        if spd > 120 and abs(_mid_now) > 7.0:
            max_delta = max(max_delta, 0.35)
        try:
            _progress_now = float(getattr(sensing_info, "lap_progress", 0.0))
            _nearest_dist_now = float(nearest_ob_dist) if nearest_ob_dist is not None else 1e9
            _nearest_mid_now = float(nearest_ob_tm) if nearest_ob_tm is not None else 0.0
            if (
                64.0 <= _progress_now <= 72.0
                and spd > 145
                and _nearest_dist_now <= 25.0
                and ((_mid_now > 3.4 and _nearest_mid_now < -2.5) or (_mid_now < -3.4 and _nearest_mid_now > 2.5))
            ):
                max_delta = max(max_delta, 0.45)
        except Exception:
            pass
        try:
            if (
                48.5 <= float(getattr(sensing_info, "lap_progress", 0.0)) <= 50.6
                and spd > 110
                and abs(float(getattr(sensing_info, "moving_angle", 0.0))) > 16.0
            ):
                max_delta = max(max_delta, 0.6)
        except Exception:
            pass
        try:
            if (
                18.4 <= float(getattr(sensing_info, "lap_progress", 0.0)) <= 20.1
                and spd > 90
                and float(getattr(sensing_info, "moving_angle", 0.0)) > 14.0
                and _mid_now > 2.0
            ):
                max_delta = max(max_delta, 0.6)
        except Exception:
            pass
        try:
            if float(getattr(sensing_info, "lap_progress", 0.0)) >= 98.5 and spd > 100 and abs(_mid_now) > 6.2:
                max_delta = max(max_delta, 0.45)
        except Exception:
            pass
        try:
            if (
                float(getattr(sensing_info, "lap_progress", 0.0)) >= 98.8
                and spd > 95
                and abs(float(getattr(sensing_info, "moving_angle", 0.0))) > 16.0
            ):
                max_delta = max(max_delta, 0.6)
        except Exception:
            pass
        if abs(_mid_now) > 11.2:
            max_delta = 1.0
        if self.accident_step != 0 or self.uturn_count > 5:
            max_delta = 1.0
        steer_pre_smooth = car_controls.steering
        steer_delta_raw = steer_pre_smooth - before
        delta = steer_delta_raw
        if delta > max_delta: car_controls.steering = before + max_delta
        elif delta < -max_delta: car_controls.steering = before - max_delta
        if car_controls.steering > 1:
            car_controls.steering = 1
        elif car_controls.steering < -1:
            car_controls.steering = -1
        before = car_controls.steering

        # 탈출 및 복구
        if spd > 10 and not close_collision:
            self.accident_step = 0
            self.recovery_count = 0
            self.accident_count = 0

        if sensing_info.lap_progress > 0.5 and self.accident_step == 0 and abs(spd) < 1.0:
            self.accident_count += 1

        if self.accident_count > 8: self.accident_step = 1

        if self.accident_step == 1:
            self.recovery_count += 1
            if middle >= 0: car_controls.steering = -1 
            else: car_controls.steering = 1
            car_controls.throttle = -1
            car_controls.brake = 0
            if spd < -20: car_controls.throttle = 0; car_controls.brake = 0.5

        if self.recovery_count > 20:
            self.accident_step = 2
            self.recovery_count = 0
            self.accident_count = 0

        if self.accident_step == 2:
            car_controls.steering = 0
            car_controls.throttle = 1
            car_controls.brake = 1
            if sensing_info.speed > -1:
                self.accident_step = 0
                car_controls.throttle = 1
                car_controls.brake = 0

        if not sensing_info.moving_forward and not (self.accident_count + self.accident_step + self.recovery_count) and spd > 0:
            self.uturn_count += 1
            if not self.uturn_step:
                if middle >= 0: self.uturn_step = 1
                else: self.uturn_step = -1

        if sensing_info.moving_forward:
            self.uturn_count = 0
            self.uturn_step = 0

        if self.uturn_count > 5:
            car_controls.steering = self.uturn_step
            car_controls.throttle = 0.5

        # 로그 저장 (CSV only, JSON/print 제거)
        try:
            mode = "v10_2" if elapsed_seconds < 20 else "v10_6"

            logic_tags = [mode]
            if cnt > 0: logic_tags.append("obstacle")
            if is_narrow_gap: logic_tags.append("narrow_gap")
            if obs_in_corner_zone_cnt > 0: logic_tags.append(f"corner_obs_{obs_in_corner_zone_cnt}")
            if is_hard_corner_ahead: logic_tags.append("hard_corner")
            if self.accident_step or self.recovery_count or self.accident_count: logic_tags.append("recovery")
            if self.uturn_count > 5: logic_tags.append("uturn_active")
            if 'dense_obs' in locals() and dense_obs: logic_tags.append("dense_obs")
            if 's_cut_active' in locals() and s_cut_active: logic_tags.append("s_cut")

            _angles = sensing_info.track_forward_angles or [0.0] * 20
            _tg = int(tg) if tg is not None else 0
            if _tg < 0: _tg = 0
            if _tg > 19: _tg = 19
            _ang_tg = _angles[_tg] if len(_angles) > _tg else 0.0

            logic_basis = ";".join([
                f"spd={sensing_info.speed:.2f}",
                f"mid={sensing_info.to_middle:.2f}",
                f"ang={sensing_info.moving_angle:.2f}",
                f"lp={getattr(sensing_info, 'lap_progress', 0.0):.4f}",
                f"tg={_tg}",
                f"ang_tg={_ang_tg:.2f}",
                f"target={getattr(self, 'target_offset', 0.0):.2f}",
                f"cnt={int(cnt)}",
                f"n_ob={(-1.0 if nearest_ob_dist is None else float(nearest_ob_dist)):.2f}",
                f"ob_tm={(0.0 if nearest_ob_tm is None else float(nearest_ob_tm)):.2f}",
                f"risk={int(bool(collision_risk_obj))}",
                f"max_d={(0.0 if max_delta is None else float(max_delta)):.3f}",
            ])

            # ---- extra debug fields for CSV ----
            self._dbg_dense_obs = bool(dense_obs) if 'dense_obs' in locals() else False
            self._dbg_gap_center = gap_center if 'gap_center' in locals() else None
            self._dbg_gap_width = gap_width if 'gap_width' in locals() else None
            self._dbg_route_hold_center = getattr(self, '_route_hold_center', None)
            self._dbg_route_ema = getattr(self, '_route_ema', None) if getattr(self, '_route_ema_inited', False) else None
            # ------------------------------------

            self._log_tick(
                sensing_info=sensing_info,
                car_controls=car_controls,
                elapsed_s=elapsed_seconds,
                mode=mode,
                tg_idx=_tg,
                target_speed=target_speed,
                half_load_width=half_load_width,
                nearest_ob_dist=nearest_ob_dist,
                nearest_ob_to_middle=nearest_ob_tm,
                obs_cnt=cnt,
                is_narrow_gap=is_narrow_gap,
                obs_in_corner_zone_cnt=obs_in_corner_zone_cnt,
                collision_risk=collision_risk_obj,
                steer_pre=steer_pre_smooth,
                steer_delta=steer_delta_raw,
                steer_rate_limit=max_delta,
                middle_add=middle_add,
                avoid_gain=avoid_gain,
                logic_applied="|".join(logic_tags),
                logic_basis=logic_basis,
            )
        except Exception:
            pass
        
        return car_controls

    # ===================== CSV 로깅 ===================== #
    
    # ===================== CSV 로깅 (GPT v1) ===================== #
    def _init_csv_logger(self):
        try:
            # flush는 매 tick마다 하지 않고, 일정 프레임마다만 수행(부하 절감)
            self._log_frame_count = 0
            self._log_flush_every = 10  # 10 frames ~= 1s (0.1s control loop)

            file_exists = os.path.exists(self.debug_log_file)
            need_header = (not file_exists) or (os.path.getsize(self.debug_log_file) == 0)

            self.debug_log_fp = open(self.debug_log_file, mode="a", newline="", encoding="utf-8")

            angle_headers = [f"ang_{i:02d}" for i in range(20)]
            self._csv_fields = [
                "time", "elapsed_s", "mode",
                "lap_progress",
                "speed", "to_middle", "moving_angle",
                "middle_add", "avoid_gain",
                "steer_pre", "steer", "steer_delta", "steer_rate_limit",
                "throttle", "brake",
                "target_offset", "prev_target",
                "half_road_limit", "half_load_width",
                "tg_idx", "ang_tg",
                "target_speed",
                "obs_cnt", "nearest_ob_dist", "nearest_ob_to_middle",
                "dense_obs", "gap_center", "gap_width", "route_hold_center", "route_ema",
                "is_narrow_gap", "obs_in_corner_zone_cnt",
                "collision_risk",
                "is_accident", "accident_count", "accident_step",
                "recovery_count", "uturn_count", "uturn_step",
                "logic_applied", "logic_basis",
                "all_obstacles", "all_opponents",
            ] + angle_headers

            self.debug_log_writer = csv.DictWriter(self.debug_log_fp, fieldnames=self._csv_fields)
            if need_header:
                self.debug_log_writer.writeheader()
        except Exception:
            self.debug_log_fp = None
            self.debug_log_writer = None

    def _log_tick(
        self,
        sensing_info,
        car_controls,
        *,
        elapsed_s,
        mode,
        tg_idx,
        target_speed,
        half_load_width,
        nearest_ob_dist,
        nearest_ob_to_middle,
        obs_cnt,
        is_narrow_gap,
        obs_in_corner_zone_cnt,
        collision_risk,
        steer_pre,
        steer_delta,
        steer_rate_limit,
        middle_add,
        avoid_gain,
        logic_applied,
        logic_basis,
    ):
        if not self.debug_log_writer:
            return

        t = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        angles = sensing_info.track_forward_angles or []
        if len(angles) < 20:
            angles = angles + [0.0] * (20 - len(angles))
        else:
            angles = angles[:20]

        # 안전하게 인덱스 보정
        try:
            tg_idx_int = int(tg_idx)
        except Exception:
            tg_idx_int = 0
        if tg_idx_int < 0:
            tg_idx_int = 0
        if tg_idx_int > 19:
            tg_idx_int = 19

        # 문자열 필드는 CSV 파싱 편의를 위해 콤마를 세미콜론으로 치환
        all_obs = str(getattr(sensing_info, "track_forward_obstacles", []) or []).replace(",", ";")
        all_opps = str(getattr(sensing_info, "opponent_cars_info", []) or []).replace(",", ";")

        row = {
            "time": t,
            "elapsed_s": round(float(elapsed_s), 3) if elapsed_s is not None else "",
            "mode": mode or "",
            "lap_progress": round(float(getattr(sensing_info, "lap_progress", 0.0)), 4),

            "speed": round(float(getattr(sensing_info, "speed", 0.0)), 2),
            "to_middle": round(float(getattr(sensing_info, "to_middle", 0.0)), 3),
            "moving_angle": round(float(getattr(sensing_info, "moving_angle", 0.0)), 3),
            "middle_add": (round(float(middle_add), 4) if middle_add is not None else ""),
            "avoid_gain": (round(float(avoid_gain), 4) if avoid_gain is not None else ""),

            "steer_pre": (round(float(steer_pre), 4) if steer_pre is not None else ""),
            "steer": round(float(getattr(car_controls, "steering", 0.0)), 4),
            "steer_delta": (round(float(steer_delta), 4) if steer_delta is not None else ""),
            "steer_rate_limit": (round(float(steer_rate_limit), 4) if steer_rate_limit is not None else ""),

            "throttle": round(float(getattr(car_controls, "throttle", 0.0)), 4),
            "brake": round(float(getattr(car_controls, "brake", 0.0)), 4),

            "target_offset": round(float(getattr(self, "target_offset", 0.0)), 3),
            "prev_target": round(float(getattr(self, "prev_target", 0.0)), 3),

            "half_road_limit": round(float(getattr(self, "half_road_limit", 0.0)), 3),
            "half_load_width": (round(float(half_load_width), 3) if half_load_width is not None else ""),

            "tg_idx": tg_idx_int,
            "ang_tg": round(float(angles[tg_idx_int]), 3),

            "target_speed": (round(float(target_speed), 3) if target_speed is not None else ""),

            "obs_cnt": int(obs_cnt) if obs_cnt is not None else 0,
            "nearest_ob_dist": (round(float(nearest_ob_dist), 3) if nearest_ob_dist is not None else ""),
            "nearest_ob_to_middle": (round(float(nearest_ob_to_middle), 3) if nearest_ob_to_middle is not None else ""),

        "dense_obs": int(bool(getattr(self, "_dbg_dense_obs", False))),
        "gap_center": (round(float(getattr(self, "_dbg_gap_center", 0.0)), 3) if getattr(self, "_dbg_gap_center", None) is not None else ""),
        "gap_width": (round(float(getattr(self, "_dbg_gap_width", 0.0)), 3) if getattr(self, "_dbg_gap_width", None) is not None else ""),
        "route_hold_center": (round(float(getattr(self, "_dbg_route_hold_center", 0.0)), 3) if getattr(self, "_dbg_route_hold_center", None) is not None else ""),
        "route_ema": (round(float(getattr(self, "_dbg_route_ema", 0.0)), 3) if getattr(self, "_dbg_route_ema", None) is not None else ""),

            "is_narrow_gap": int(bool(is_narrow_gap)),
            "obs_in_corner_zone_cnt": int(obs_in_corner_zone_cnt) if obs_in_corner_zone_cnt is not None else 0,

            "collision_risk": int(bool(collision_risk)),

            "is_accident": int(bool(getattr(self, "is_accident", False))),
            "accident_count": int(getattr(self, "accident_count", 0)),
            "accident_step": int(getattr(self, "accident_step", 0)),
            "recovery_count": int(getattr(self, "recovery_count", 0)),
            "uturn_count": int(getattr(self, "uturn_count", 0)),
            "uturn_step": int(getattr(self, "uturn_step", 0)),

            "logic_applied": (logic_applied or ""),
            "logic_basis": (logic_basis or ""),

            "all_obstacles": all_obs,
            "all_opponents": all_opps,
        }

        # angle columns
        for i in range(20):
            row[f"ang_{i:02d}"] = round(float(angles[i]), 3)

        self.debug_log_writer.writerow(row)

        # flush throttling
        self._log_frame_count += 1
        if self._log_flush_every and (self._log_frame_count % self._log_flush_every == 0):
            try:
                self.debug_log_fp.flush()
            except Exception:
                pass

    def __del__(self):
        try:
            if self.debug_log_fp:
                self.debug_log_fp.close()
        except Exception:
            pass

    def set_player_name(self):
        return "v33_Dodge_Hybrid_CorridorHold_v9"

if __name__ == '__main__':
    client = DrivingClient()
    return_code = client.run()
    raise SystemExit(return_code)
