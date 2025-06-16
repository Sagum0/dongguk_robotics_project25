import numpy as np

def generate_circle_path(x0, y0, z0, radius, num_circle):
    
    total_angle = 2 * np.pi * 1.02

    # 2. num_circle 만큼 등간격 theta 생성 (끝점 미포함)
    theta = np.linspace(0, total_angle, num_circle, endpoint=False)

    if radius < 25.0: # 반지름 20mm 이하 (5는 offset 값)
        minor_ratio = 1.0
    elif radius >= 25.0 and radius <= 35.0:
        minor_ratio = 0.97
    else:
        minor_ratio = 0.95

    # 단반경 계산 5%
    x_semi_minor_axis = minor_ratio * radius
    
    # 3. x, y 계산
    x = x0 + x_semi_minor_axis * np.cos(theta)
    y = y0 + radius * np.sin(theta)

    # 4. z는 중심점 z0으로 고정
    z = np.full(num_circle, z0)

    # 5. (N,3) 배열로 합치기
    path = np.vstack((x, y, z)).T
    return path