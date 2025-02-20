#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import threading
import queue
import math

def move_turtle_to(pub, x, y, current_x, current_y, rate):
    """
    Mueve la tortuga hacia la posición (x, y) y ajusta la dirección para
    seguir la forma del corazón.
    """
    move_cmd = Twist()
    
    # Calculamos la diferencia de posición
    dx = x - current_x
    dy = y - current_y

    # Calculamos el ángulo hacia el objetivo (en radianes)
    angle_to_target = math.atan2(dy, dx)

    # Ajustamos la orientación de la tortuga para que apunte hacia el objetivo
    move_cmd.angular.z = angle_to_target  # Control de la rotación
    move_cmd.linear.x = 0.1  # Velocidad lineal (ajusta para que no sea demasiado rápido)

    # Publicamos el comando de movimiento
    pub.publish(move_cmd)
    
    # Esperamos un poco para que la tortuga se mueva
    rate.sleep()

    return x, y  # Retornamos la nueva posición

def draw_heart():
    rospy.init_node('turtle_heart', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(30)  # 30 Hz

    t = 0.0
    current_x, current_y = 0.0, 0.0  # Posición inicial

    while not rospy.is_shutdown():
        # Parametric equations for the heart shape
        x = 16 * math.sin(t)**3
        y = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
        
        # Escalamos las coordenadas para que la tortuga tenga un espacio adecuado en la ventana
        x *= 0.1  # Ajuste para la escala
        y *= 0.1  # Ajuste para la escala

        # Mueve la tortuga hacia la posición calculada (x, y)
        current_x, current_y = move_turtle_to(pub, x, y, current_x, current_y, rate)

        # Incrementamos el valor de t para que se dibuje el corazón
        t += 0.05
        if t > 2 * math.pi:
            t = 0.0

        rate.sleep()


if __name__ == '__main__':
    try:
        draw_heart()
    except rospy.ROSInterruptException:
        pass