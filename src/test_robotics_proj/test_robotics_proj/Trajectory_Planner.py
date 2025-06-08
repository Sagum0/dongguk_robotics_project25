import numpy as np

class CircleFullPathPlanner:
    def __init__(self, start_point, center_point, radius, z_circle, num_move, num_circle):
        """
        start_point: (3,) array, 시작 위치
        center_point: (3,) array, 원 중심 위치 (x, y, z_center)
        radius: float, 원 반지름
        z_circle: float, 원 그릴 때 고정 z값
        num_move: int, 중심까지/돌아갈 때 경유점 개수
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

        # 1. 원의 중심으로 이동 (z는 중심 좌표의 z로 맞춤)
        move1_t = np.linspace(0, 1, self.num_move)
        move1_x = xs + (x0 - xs) * move1_t
        move1_y = ys + (y0 - ys) * move1_t
        move1_z = zs + (zc - zs) * move1_t
        move1_path = np.vstack((move1_x, move1_y, move1_z)).T

        # 2. 원 그리기 시작점: 원 위에서 시작하는 각도(angle0) 구하기 (z_circle 고정)
        angle0 = np.arctan2(move1_y[-1] - y0, move1_x[-1] - x0)
        x_on = x0 + self.r * np.cos(angle0)
        y_on = y0 + self.r * np.sin(angle0)
        z_on = self.z_circle

        # 2-1. 중심에서 원 위로 이동 (z_circle 고정)
        move2_t = np.linspace(0, 1, self.num_move)
        move2_x = move1_x[-1] + (x_on - move1_x[-1]) * move2_t
        move2_y = move1_y[-1] + (y_on - move1_y[-1]) * move2_t
        move2_z = move1_z[-1] + (z_on - move1_z[-1]) * move2_t
        move2_path = np.vstack((move2_x, move2_y, move2_z)).T

        # 3. 원 그리기 (10% 더, 즉 1.1바퀴)
        angle_total = 2 * np.pi * 1.2
        theta = np.linspace(angle0, angle0 + angle_total, int(self.num_circle * 1.2), endpoint=False)
        x_circle = x0 + self.r * np.cos(theta)
        y_circle = y0 + self.r * np.sin(theta)
        z_circle = np.full_like(theta, self.z_circle)
        circle_path = np.vstack((x_circle, y_circle, z_circle)).T

        # 4. 원 위에서 다시 중심으로 이동 (z_circle → zc로 복귀)
        move3_t = np.linspace(0, 1, self.num_move)
        move3_x = circle_path[-1, 0] + (x0 - circle_path[-1, 0]) * move3_t
        move3_y = circle_path[-1, 1] + (y0 - circle_path[-1, 1]) * move3_t
        move3_z = circle_path[-1, 2] + (zc - circle_path[-1, 2]) * move3_t
        move3_path = np.vstack((move3_x, move3_y, move3_z)).T

        # 전체 경로 연결
        full_path = np.vstack((move1_path, move2_path, circle_path, move3_path))
        return full_path
