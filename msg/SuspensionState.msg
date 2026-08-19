# 悬架系统状态消息
# 单位：位移 m，速度 m/s，加速度 m/s^2
# 后续可扩展更多字段（如悬架动行程、轮胎载荷等）

# 采样时刻（相对节点启动的时间，s）
float64 time

# 当前路面高度输入（m）
float64 road_height

# 簧上质量位移（m）
float64 xs
# 簧下质量位移（m）
float64 xus
# 簧上质量速度（m/s）
float64 xs_dot
# 簧下质量速度（m/s）
float64 xus_dot

# 车身（簧上）加速度（m/s^2）
float64 body_accel
