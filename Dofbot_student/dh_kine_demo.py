import roboticstoolbox as rtb
import numpy as np
import math
import matplotlib.pyplot as plt
import os
# os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = r".conda\envs\robotics\Lib\site-packages\PyQt5\Qt5\plugins\platforms"
pi = 3.1415926          # 定义pi常数

l1 = 0.1045             # 定义第一连杆长度
l2 = 0.08285            # 定义第三连杆长度
l3 = 0.08285            # 定义第四连杆长度
l4 = 0.12842            # 定义第五连杆长度

# student version
# 用改进DH参数发表示机器人正运动学
# TODO: modify the dh param
dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a = 0, alpha=0, d=l1, offset=0),
        rtb.RevoluteMDH(a = 0, alpha=-pi/2, d=0, offset=-pi/2),
        rtb.RevoluteMDH(a = l2, alpha=0, d=0, offset=0),
        rtb.RevoluteMDH(a = l3, alpha=0, d=0, offset=pi/2),
        rtb.RevoluteMDH(a = 0, alpha=pi/2, d=l4, offset=0)
    ]
)

# 输出机器人DH参数矩阵
print(dofbot)


'''
Part1 给出一下关节姿态时的机械臂正运动学解，并附上仿真结果
0.(demo) [0., pi/3, pi/4, pi/5, 0.]
1.[pi/2, pi/5, pi/5, pi/5, pi]
2.[pi/3, pi/4, -pi/3, -pi/4, pi/2]
3.[-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
'''

# part1 demo
fkine_input0 = [0., pi/3, pi/4, pi/5, 0.]
fkine_input0 = [0., 0., 0., 0., 0.]
fkine_result0 = dofbot.fkine(fkine_input0)
print(fkine_result0)
dofbot.plot(q=fkine_input0, block=True)

# part1-1
fkine_input1 = [pi/2, pi/5, pi/5, pi/5, pi]
fkine_result1 = dofbot.fkine(fkine_input1)
print(fkine_result1)
dofbot.plot(q=fkine_input1, block=True)
# part1-2
fkine_input2 = [pi/3, pi/4, -pi/3, -pi/4, pi/2]
fkine_result2 = dofbot.fkine(fkine_input2)
print(fkine_result2)
dofbot.plot(q=fkine_input2, block=True)

# part1-3
fkine_input3 = [-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
fkine_result3 = dofbot.fkine(fkine_input3)
print(fkine_result3)
dofbot.plot(q=fkine_input3, block=True)

'''
Part1 给出一下关节姿态时的机械臂逆运动学解，并附上仿真结果
0.(demo) 
    [
        [-1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., -1., -0.1],
        [0., 0., 0., 1.]
    ]
1.
    [
        [1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.1],
        [0., 0., 0., 1.]
    ]
2.
    [
        [cos(pi/3), 0., -sin(pi/3), 0.2,],
        [0., 1., 0., 0.],
        [sin(pi/3), 0., cos(pi/3)., 0.2],
        [0., 0., 0., 1.]
    ]
3.
    [
        [-0.866, -0.25, -0.433, -0.03704,],
        [0.5, -0.433, -0.75, -0.06415],
        [0., -0.866, 0.5, 0.3073],
        [0., 0., 0., 1.]
    ]
'''

#part2 demo
target_pos0 = np.array([
    [-1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., -1., -0.1],
    [0., 0., 0., 1.]
])
ikine_result0 = dofbot.ik_LM(target_pos0)[0]
print("ikine: ", np.array(ikine_result0))
dofbot.plot(q=ikine_result0, block=True)


target_pos1 = np.array([
    [1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., 1., 0.1],
    [0., 0., 0., 1.]
])
ikine_result1 = dofbot.ik_LM(target_pos1)[0]
print("ikine: ", np.array(ikine_result1))
dofbot.plot(q=ikine_result1, block=True)


#part2-2
target_pos2 = np.array([
    [math.cos(-pi/3), 0., -math.sin(-pi/3), 0.2,],
    [0., 1., 0., 0.],
    [math.sin(-pi/3), 0., math.cos(-pi/3), 0.2],
    [0., 0., 0., 1.]
])
ikine_result2 = dofbot.ik_LM(target_pos2)[0]
print("ikine: ", np.array(ikine_result2))
dofbot.plot(q=ikine_result2, block=True)


#part2-3
target_pos3 = np.array([
    [-0.866, -0.25, -0.433, -0.03704,],
    [0.5, -0.433, -0.75, -0.06415],
    [0., -0.866, 0.5, 0.3073],
    [0., 0., 0., 1.]
])
ikine_result3 = dofbot.ik_LM(target_pos3)[0]
print("ikine: ", np.array(ikine_result3))
dofbot.plot(q=ikine_result3, block=True)

X=[],Y=[],Z=[]
for theta_1 in range(-180,181,360//10):
    for theta_2 in range(-90,91,180//5):
        for theta_3 in range(-150,151,300//5):
            for theta_4 in range(-100,101,200//5):
                for theta_5 in range(-180,181,360//10):
                    q = np.array([theta_1, theta_2, theta_3, theta_4, theta_5])*pi/180
                    fkine = dofbot.fkine(q)
                    X.append(fkine.A[0][3])
                    Y.append(fkine.A[1][3])
                    Z.append(fkine.A[2][3])                
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X,Y,Z)
plt.show()
