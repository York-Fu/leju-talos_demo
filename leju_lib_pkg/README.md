# 通用仓库包

## 用法

1. catkin_make
2. source devel/setup.bash
3. import lejufunc 或 from lejufunc import *


## 库介绍

### client_action

#### def action(act_frames)

> 执行连续的动作

Args:

- act_frames：该动作的所有关键帧
  - 类型为数组，数组内为元组
  - 该元组共有三个数据
    1. 22个关节角度的元组
    2. 上一关键帧到当前关键帧的运行时间
    3. 该动作结束后的等待时间