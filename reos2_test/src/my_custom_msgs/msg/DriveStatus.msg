# 电机驱动器上报的健康/异常状态（由驱动器/传感器节点发布）
float32 voltage       # 母线电压 (V)
float32 temperature   # 驱动器/电机温度 (℃)
bool    stall         # 是否堵转（有转速指令但实际转速接近 0）
bool    driver_fault  # 驱动器是否报故障（过流/过压/通讯异常等）
bool    valid         # 数据是否有效（false 表示无有效读数）
