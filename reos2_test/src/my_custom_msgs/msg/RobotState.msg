# 简单差分驱动机器人的实时状态
float32 vx            # 前进速度 (m/s)，正为前进
float32 wz            # 角速度 (rad/s)，正为左转
float32 left_wheel    # 左轮角速度 (rad/s)
float32 right_wheel   # 右轮角速度 (rad/s)
float32 battery       # 电池剩余电量 (%)
uint8   status        # 状态：0=待机 1=运行 2=低电量告警
bool    alarm         # 是否正在报警（行人紧急制动 / 驱动器故障等）
uint8   fault         # 故障标志位：bit0=低压 bit1=过热 bit2=堵转 bit3=驱动器异常，0=无故障
