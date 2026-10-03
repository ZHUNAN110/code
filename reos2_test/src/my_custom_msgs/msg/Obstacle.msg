# 前方障碍物信息
uint8   type       # 类型：0=台阶(小,可跨越) 1=大型障碍(刚体) 2=行人
float32 distance   # 到障碍物的距离 (m)
float32 height     # 障碍物高度 (m)
bool    valid      # 数据是否有效（false 表示无有效读数）
# —— 刚体障碍物的"空洞"信息（type=1 且 has_gap=true 时有效）——
bool    has_gap    # 障碍物中间是否有可穿过的空洞/缺口（如门框、拱门）
float32 gap_width  # 空洞宽度 (m)，has_gap=true 时有效
