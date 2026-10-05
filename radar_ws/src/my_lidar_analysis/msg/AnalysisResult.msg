# 每秒统计结果的汇总消息（节点1 发布，节点2 订阅）

# 总行程（累计，单位：米）
float64 total_distance

# 最大速度（累计，单位：米/秒）
float64 max_speed

# 帧率（这一秒收到的帧数，单位：Hz）
int32 cloud_hz
int32 imu_hz
int32 odom_hz
int32 tf_hz

# 点云点数统计
float64 avg_points
uint32 min_points
uint32 max_points

# IMU 均值（下标 0-2 是加速度 xyz，3-5 是角速度 xyz）
float64[6] imu_mean
# IMU 噪声标准差 σ（下标含义同上）
float64[6] imu_stddev
