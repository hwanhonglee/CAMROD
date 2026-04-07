python3 -c "
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

# 1. 원본 데이터 로드
traj_df = pd.read_csv('traj_lidar_xyz_yaw.csv')
gnss_df = pd.read_csv('gnss_local_.csv')
# 타임스탬프 원본 (개수 맞춤용)
with open('traj_lidar_flexcloud.txt', 'r') as f:
    orig_ts = [line.split()[0] for line in f if line.strip()][:len(gnss_df)]

# 2. GNSS 변환 (7열: TS, x, y, z, std_x, std_y, std_z)
# gnss_local_.csv 구조: x, y, z, yaw(실제로는 std 역할을 하는 값 2)
with open('gnss_local_flexcloud_7_col.txt', 'w') as f:
    for i in range(len(gnss_df)):
        row = gnss_df.iloc[i]
        # x, y, z, std_x, std_y, std_z (std값은 원본의 마지막 열 2를 기반으로 임의 설정)
        line = f'{orig_ts[i]} {row[0]:.10f} {row[1]:.10f} {row[2]:.10f} 0.0100000000 0.0100000000 0.0400000000'
        f.write(line + '\n')

# 3. Trajectory 변환 (13열: TS, r11~r33, tx, ty, tz)
with open('traj_lidar_flexcloud_13_col.txt', 'w') as f:
    for i in range(len(gnss_df)):
        t_row = traj_df.iloc[i]
        tx, ty, tz, yaw = t_row[0], t_row[1], t_row[2], t_row[3]
        
        # Yaw(Z-axis rotation)를 회전 행렬로 변환
        rot = R.from_euler('z', yaw).as_matrix()
        
        # 13열 구성: TS, r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz
        kitti = [
            float(orig_ts[i]),
            rot[0,0], rot[0,1], rot[0,2], tx,
            rot[1,0], rot[1,1], rot[1,2], ty,
            rot[2,0], rot[2,1], rot[2,2], tz
        ]
        f.write(' '.join(f'{x:.10f}' for x in kitti) + '\n')

print('성공: gnss_local_flexcloud_7_col.txt 및 traj_lidar_flexcloud_13_col.txt 생성 완료')
"
