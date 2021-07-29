#!/usr/bin/env python
import rospy
import time
import threading
from std_msgs.msg import Bool, Empty, Float32
#from nav_msgs.msg import *
from geometry_msgs.msg import Twist

#TIENE BOTON DE PANICO, DEPLOY PROBE, AJUSTAR VELOCIDAD Y RESET ODOMETRY
#ROBOCOL

class Poses_Publish(object):
    def __init__(self):
        super(Poses_Publish, self).__init__()
        
        self.opciones = True
        
        print('Starting robocol_managment node...')
        rospy.init_node('robocol_managment')
        # Publishers
        self.pubVel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pubResetOdom = rospy.Publisher('zed2/reset_odometry', Empty, queue_size=1)
        self.pubProbe = rospy.Publisher('probe_deployment_unit/drop', Empty, queue_size=1)
        self.pub_flagAuto = rospy.Publisher('Robocol/MotionControl/flag_autonomo', Bool, queue_size=1)
        self.pub_flagPanic = rospy.Publisher('Robocol/MotionControl/flag_panic', Bool, queue_size=1)
        self.pub_kp = rospy.Publisher('Robocol/MotionControl/kp', Float32, queue_size=1)

        rospy.on_shutdown(self.kill)
        print('')
        x = threading.Thread(target=self.thread_function)
        x.start()


    def thread_function(self):
        while self.opciones:
            print("Choose an option:")
            print(" P: Panic Button")
            print(" D: To deploy probe.")
            print(' V: To adjust velocity.')
            print(" R: To reset odometry.")
            op = input(' > ')
            print(op)
            if op == "P" or op == 'p':
                print(' PANIK BUTTON')
                flag_panic = True
                self.pub_flagPanic.publish(flag_panic)
                print('  Stopping Control Node')
                flag_autonomo = False
                self.pub_flagAuto.publish(flag_autonomo)
                print('  Stopped Robot.')
                vel_robot = Twist()
                vel_robot.linear.x = 0
                vel_robot.linear.y = 0
                vel_robot.angular.z = 0
                self.pubVel.publish(vel_robot)
                print('Press P again to resume.')
                a = input(' > ')
                if a == 'P' or a == 'p':
                    print(' ENABLING MOVEMENT')
                    flag_panic = False
                    self.pub_flagPanic.publish(flag_panic)
                    #flag_autonomo = True
                    #self.pub_flagAuto.publish(flag_autonomo)
                
            elif op == "D" or op == 'd':
                print(" Probe droped")
                self.pubProbe.publish()

            elif op == 'V' or op == 'v':
                print(' Input value to reduce or increase velocity. Check Status for actual velocity')
                print(' Input 0 (zero) to disable...')
                kp = input(' > ')  
                self.pub_kp.publish(kp) 
                print(' VELOCITY ADJUSTED')

            elif op == "R" or op == 'r':
                print(' Odometry reseted')
                self.pubResetOdom.publish()
            else:
                print(' COMMAND NOT RECOGNIZED.')
            print('')
        print('Closing thread...')

    def kill(self):
        print("\nKilling node...")
        self.opciones = False
        print('Press Enter to end...')
        

def main():
    try:
        poses_Publish = Poses_Publish()

        time.sleep(1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # poses_Publish.pub_test()
            rate.sleep()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()