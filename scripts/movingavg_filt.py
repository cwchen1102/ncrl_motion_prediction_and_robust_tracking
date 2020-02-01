import rospy
import collections
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

window_length = 80
angles = collections.deque(maxlen=window_length)
angle_filt = 3.1415926/2
box2 = 0
count = 1

def callback(box):
    global angle_filt, count, box2
    
    if box.data[0] != -1:
        if abs(angle_filt - box.data[5]) > 3.1415926:                   #box.data[5] or box.data[6] depends on the version of YOLO
            angle_sub = 2*3.1415926 - abs(angle_filt - box.data[5])
        else:
            angle_sub = abs(angle_filt - box.data[5])

        if angle_sub < 3.1415926/4:
            angles.append(box.data[5])
		
        angle_sum = 0
        if count >= window_length:
            for angle in angles:
                angle_sum = angle_sum + angle
            angle_filt = angle_sum/window_length
        else:
            angle_filt = box.data[5]
            count = count + 1

        box2 = box.data[5]
    
def listener():
    
    rospy.init_node('angle_filter', anonymous=True)

    rospy.Subscriber("YOLO/box", Float32MultiArray, callback)
    
    pub = rospy.Publisher('car_angle_avg', Float64, queue_size=1)

    rate = rospy.Rate(80)

    while not rospy.is_shutdown():
        print('angle_filt')
        print(angle_filt/3.1415926*180)
        print('box_data')
        print(box2/3.1415926*180)
        pub.publish(angle_filt)
        rate.sleep()

if __name__ == '__main__':
    listener()
