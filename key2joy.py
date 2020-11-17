import os
import pygame
from pygame.locals import *
import time
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def main():
    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 438, 344)
    screen = pygame.display.set_mode(window_size.size)
    path = os.path.dirname(os.path.abspath(__file__))
    print(path)
    img = pygame.image.load(path+"/figs/key2joy.png")

    # initialize ros publisher
    twist_pub = rospy.Publisher('keyboard/twist', Twist, queue_size=10)
    joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
    rospy.init_node('key2joy')
    rate = rospy.Rate(50)

    # init joy message
    joy_ = Joy()    
    joy_.header.frame_id = 'world'
    for i in range(8):
      joy_.axes.append(0.0)
    for i in range(11):
      joy_.buttons.append(0)

    while not rospy.is_shutdown():
        rate.sleep()
        screen.blit(img, (1,1))
        pygame.display.flip()

        for event in pygame.event.get():
            # ---------------------- key dowm message ----------------------
            
            if event.type == KEYDOWN:
                if event.key == pygame.K_w:
                    # print 'up'
                    joy_.axes[1] = 1.0
                if event.key == pygame.K_s:
                    # print 'down'
                    joy_.axes[1] = -1.0
                if event.key == pygame.K_a:
                    # print 'left'
                    joy_.axes[0] = -1.0
                if event.key == pygame.K_d:
                    # print 'right'
                    joy_.axes[0] = 1.0
                joy_pub.publish(joy_)

            # when keyup, reset wind velocity
            elif event.type == pygame.KEYUP:
                # wind direction
                if event.key == pygame.K_w:
                    joy_.axes[1] = 0.0
                if event.key == pygame.K_s:
                    joy_.axes[1] = -0.0
                if event.key == pygame.K_a:
                    joy_.axes[0] = 0.0
                if event.key == pygame.K_d:
                    joy_.axes[0] = -0.0
                joy_pub.publish(joy_)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
