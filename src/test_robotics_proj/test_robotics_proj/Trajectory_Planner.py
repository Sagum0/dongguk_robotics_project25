import numpy as np

class CircleFullPathPlanner:
    def __init__(self, start_point, center_point, radius, z_circle, num_move, num_circle):
        """
        start_point: (3,) array, 시작 위치
        center_point: (3,) array, 원 중심 위치 (x, y, z_center)
        radius: float, 원 반지름
        z_circle: float, 원 그릴 때 고정 z값
        num_move: int, 이동 시 경유점 개수
        num_circle: int, 원 그리기 점 개수 (한 바퀴 기준)
        """
        self.start_point = np.array(start_point)
        self.center_point = np.array(center_point)
        self.r = radius
        self.z_circle = z_circle
        self.num_move = num_move
        self.num_circle = num_circle

    def plan(self):
        xs, ys, zs = self.start_point
        x0, y0, zc = self.center_point
        # 1. 시작 → 중심 (z는 중심 zc까지)
        t1 = np.linspace(0, 1, self.num_move)
        move1_x = xs + (x0 - xs) * t1
        move1_y = ys + (y0 - ys) * t1
        move1_z = zs + (zc - zs) * t1
        move1_path = np.vstack((move1_x, move1_y, move1_z)).T

        # 2. 중심 → 원 위 (z 고정 zc)
        angle0 = np.arctan2(move1_y[-1] - y0, move1_x[-1] - x0)
        x_on = x0 + self.r * np.cos(angle0)
        y_on = y0 + self.r * np.sin(angle0)
        t2 = np.linspace(0, 1, self.num_move)
        move2_x = move1_x[-1] + (x_on - move1_x[-1]) * t2
        move2_y = move1_y[-1] + (y_on - move1_y[-1]) * t2
        move2_z = np.full_like(move2_x, zc)
        move2_path = np.vstack((move2_x, move2_y, move2_z)).T

        # 3. 원 위에서 z_circle로 수직 하강
        t3 = np.linspace(0, 1, self.num_move)
        desc_x = np.full(self.num_move, x_on)
        desc_y = np.full(self.num_move, y_on)
        desc_z = zc + (self.z_circle - zc) * t3
        descend_path = np.vstack((desc_x, desc_y, desc_z)).T

        # 4. 원 그리기 (1.2바퀴)
        angle_total = 2 * np.pi * 1.2
        theta = np.linspace(angle0, angle0 + angle_total,
                            int(self.num_circle * 1.2), endpoint=False)
        circle_x = x0 + self.r * np.cos(theta)
        circle_y = y0 + self.r * np.sin(theta)
        circle_z = np.full_like(theta, self.z_circle)
        circle_path = np.vstack((circle_x, circle_y, circle_z)).T

        # 5. 원 끝에서 zc로 수직 상승
        t5 = np.linspace(0, 1, self.num_move)
        last_x, last_y = circle_path[-1, :2]
        asc_x = np.full(self.num_move, last_x)
        asc_y = np.full(self.num_move, last_y)
        asc_z = self.z_circle + (zc - self.z_circle) * t5
        ascend_path = np.vstack((asc_x, asc_y, asc_z)).T

        # 6. 원 끝 → 중심 (z 고정 zc)
        t6 = np.linspace(0, 1, self.num_move)
        final_x = last_x + (x0 - last_x) * t6
        final_y = last_y + (y0 - last_y) * t6
        final_z = np.full(self.num_move, zc)
        final_path = np.vstack((final_x, final_y, final_z)).T

        # 전체 경로 연결
        full_path = np.vstack((
            move1_path,
            move2_path,
            descend_path,
            circle_path,
            ascend_path,
            final_path
        ))
        return full_path
