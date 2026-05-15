import numpy as np
import matplotlib.pyplot as plt

def visualize_trajectory_2d():
    # 1. 파라미터 설정 (로봇 설정값과 동일)
    center_x, center_y, center_z = 0.5, 0.0, 0.5
    radius = 0.12
    num_steps = 36

    # 2. 좌표 계산
    thetas = np.linspace(0, 2 * np.pi, num_steps + 1)
    x_coords = np.full_like(thetas, center_x)
    y_coords = center_y + radius * np.cos(thetas)
    z_coords = center_z + radius * np.sin(thetas)

    # 3. 2D 멀티 플롯 시각화
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # 왼쪽: 정면 뷰 (YZ 평면 - 실제 원이 그려지는 곳)
    ax1.plot(y_coords, z_coords, 'b-', label='Planned Path', alpha=0.6)
    ax1.scatter(y_coords, z_coords, c='r', s=20, label='Waypoints')
    ax1.set_title('Front View (Y-Z Plane)')
    ax1.set_xlabel('Y (Horizontal)')
    ax1.set_ylabel('Z (Height)')
    ax1.axis('equal') # 원이 찌그러지지 않게 설정
    ax1.grid(True)
    ax1.legend()

    # 오른쪽: 측면 뷰 (X-Z 평면 - 깊이 확인)
    ax2.plot(x_coords, z_coords, 'g-', label='Depth Check')
    ax2.scatter(x_coords, z_coords, c='r', s=20)
    ax2.set_title('Side View (X-Z Plane)')
    ax2.set_xlabel('X (Depth)')
    ax2.set_ylabel('Z (Height)')
    ax2.set_xlim(0.3, 0.7)
    ax2.set_ylim(0.2, 0.8)
    ax2.grid(True)

    plt.tight_layout()
    print("✅ 시각화 완료! 창을 확인하세요.")
    plt.show()

if __name__ == "__main__":
    visualize_trajectory_2d()