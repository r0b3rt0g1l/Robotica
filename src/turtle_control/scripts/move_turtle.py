#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def mover_tortuga():
    rospy.init_node('mover_tortuga', anonymous=False)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)  # 2 Hz
   
    cmd = Twist()  
    tiempo_actual=rospy.get_time()
    tiempo_anterior=tiempo_actual
    


    while not rospy.is_shutdown():
        
        tiempo_actual=rospy.get_time()
        if tiempo_actual-tiempo_anterior>4:
            cmd.linear.x = 1.0    # velocidad hacia adelante

            cmd.angular.z = 1.0   # velocidad de giro  
            tiempo_anterior=tiempo_actual    
        elif tiempo_actual-tiempo_anterior>2:
            cmd.linear.x = 1.0    # velocidad hacia adelante
            cmd.angular.z = 0.0   # velocidad de giro  
        
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        mover_tortuga()
    except rospy.ROSInterruptException:
        pass