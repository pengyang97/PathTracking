import numpy as np
import math
import matplotlib.pyplot as plt

k  = 0.5   # 增益参数
Kp = 1.0   # 速度P控制器系数
dt = 0.1   # 时间间隔，单位：s(和博文图片中d(t)不是一个，注意区分)
L  = 3.0   # 车辆轴距，单位：m

class VehicleState:
    """定义一个车辆的类"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """车辆的属性初始化"""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

 
def update(state, a, delta):
    """根据车辆的运动学模型，定义车辆的状态更新函数"""
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    return state

 
def PControl(target, current):
    """P速度控制器纵向控制"""
    a = Kp * (target - current)
    return a


def stanley_control(state, cx, cy, ch, pind):
    """Stanley算法横向控制"""
    ind = calc_target_index(state, cx, cy) # 找到距离车辆前轮最近的路点

    if pind >= ind: # 如果车辆这次搜索到的最近目标路点小于上一次搜索的最小目标路点，需要把这次的搜索路点设为前一次搜索的，保证车辆是向前行驶的
        ind = pind

    # 如果找到最近的路点索引超过了目标路点的索引，则最近的路点即为最后一个路点
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
        th = ch[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        th = ch[-1]
        ind = len(cx) - 1

    # 计算横向误差（这里规定了参考轨迹在车辆的左侧时横向误差为正，反之为负）
    if ((state.x - tx) * th - (state.y - ty)) > 0: # 这里的判断语句在坐标系中画一下就可以直观的理解
        error = abs(math.sqrt((state.x - tx) ** 2 + (state.y - ty) ** 2))
    else:
        error = -abs(math.sqrt((state.x - tx) ** 2 + (state.y - ty) ** 2))
    delta = ch[ind] - state.yaw + math.atan2(k * error, state.v)

    #  限制车轮转角 [-30, 30]
    if delta > np.pi / 6.0:
        delta = np.pi / 6.0
    elif delta < - np.pi / 6.0:
        delta = - np.pi / 6.0
    return delta, ind


def calc_target_index(state, cx, cy):
    """搜索最临近的路点，并返回他的索引号"""
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    # zip() 函数用于将可迭代的对象作为参数，将对象中对应的元素打包成一个个元组，然后返回由这些元组组成的列表
    # 如果各个迭代器的元素个数不一致，则返回列表长度与最短的对象相同，利用 * 号操作符，可以将元组解压为列表。
    ind = d.index(min(d))

    return ind



def main():
    """主函数部分"""
    #  设置目标路点（这里设置了一条直线）
    cx = np.arange(0, 50, 1) # 车辆的X坐标
    cy = [0 * ix for ix in cx] # 车辆的Y坐标
    ch = [0 * ix for ix in cx] # 车辆的航向角
    target_speed = 5.0 / 3.6  # [m/s]目标车速
    T = 200.0  # 最大模拟时间
    # 设置车辆的初始状态
    state = VehicleState(x=-0.0, y=-3.0, yaw=-0.0, v=0.0)
    lastIndex = len(cx) - 1 # 最后一个路点的索引号49
    time = 0.0
    # 定义几个用来存放仿真时车辆状态以及速度时间的列表
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:
        ai = PControl(target_speed, state.v)
        di, target_ind = stanley_control(state, cx, cy, ch, target_ind)
        state = update(state, ai, di)
        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        plt.cla() # Clear axis即清除当前图形中的当前活动轴。其他轴不受影响。
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "go", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
        plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
