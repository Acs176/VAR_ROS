#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal():
    # Crea un objeto de tipo "PoseStamped"
    goal = PoseStamped()

    # Establece la posición del objetivo (cambiar valores según el mapa)
    goal.pose.position.x = 1.0
    goal.pose.position.y = 2.0
    goal.pose.position.z = 0.0

    # Establece la orientación del objetivo (cambiar valores según el mapa)
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    # Crea un objeto de tipo "Publisher" para publicar el objetivo
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Espera a que se establezca la conexión del Publisher
    rospy.sleep(1)

    # Publica el objetivo
    goal_pub.publish(goal)

if __name__ == '__main__':
    try:
        # Inicializa el nodo
        rospy.init_node('my_nav_goal_publisher', anonymous=True)

        # Publica el objetivo una vez cada segundo
        while not rospy.is_shutdown():
            publish_goal()
            rospy.sleep(1)

    except rospy.ROSInterruptException:
            pass

