import numpy as np

def generate_circle_path(x0, y0, z0, radius, num_circle):
    
    total_angle = 2 * np.pi * 1.1

    # 2. num_circle 만큼 등간격 theta 생성 (끝점 미포함)
    theta = np.linspace(0, total_angle, num_circle, endpoint=False)

    # 3. x, y 계산
    x = x0 + radius * np.cos(theta)
    y = y0 + radius * np.sin(theta)

    # 4. z는 중심점 z0으로 고정
    z = np.full(num_circle, z0)

    # 5. (N,3) 배열로 합치기
    path = np.vstack((x, y, z)).T
    return path