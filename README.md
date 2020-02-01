# ncrl_motion_prediction_and_robust_tracking
## Introduction
This is the implementation of the simulation of tracking moving target under occluded situation in the gazebo environment 
## Simulation
1. Set master & IP
master- gazebo
2. Open px4 in Gazebo
`roslaunch drone_rl mavros_posix_sitl.launch`
3. Open yolo (把圖片丟到神經網路)
```
cd ~/Desktop/YOLO/car 
py video_node.py v1 --topic /drone/camera/image_raw 
```
4. 開車子控制器
```
cd ~/Desktop/Michael/yolo/agent_controller
py turtlebot3_teleop_key
```
(鍵盤控制-等速:wsad ,等角速:qe, 等加速度:ikjl, 跳出:space)
5. 開角度濾波器
```
roscd ukf_estimate/scripts/
python movingavg_filt.py
```
6. 開飛機控制器
`roslaunch ukf_estimate target_estimate_angle.launch`
steps 1.開ukf 2.開ibvs 
(3:ukf activate, 2:ibvs activate, 1:手動模式-w +x方向, -a +y方向, -s 原地降落, -l 關掉螺旋槳)
** 開完ukf後先確認是否收斂，確認收斂後再開ibvs **
7. 開qp
```
roscd ukf_estimate/scripts/
python target_trajectory_qp_3deg.py -r 80 -l 600 -w 0.1 
```
(r:loop rate, l:window lengh, w:regulator weight)
