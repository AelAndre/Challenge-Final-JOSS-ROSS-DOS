import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declaración de parámetros configurables para el control PID
        self.declare_parameter('kp', 0.04409)
        self.declare_parameter('ki', 0.2808)
        self.declare_parameter('kd', 0.0000173)
        self.declare_parameter('sample_time', 0.1)

        # Inicialización de ganancias y tiempo de muestreo
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.sample_time = self.get_parameter('sample_time').value

        # Variables de estado del sistema y acumuladores del control
        self.set_point = 0.0
        self.motor_y = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Configuración de tópicos de publicación y suscripción
        self.control_pub = self.create_publisher(Float32, '/motor_input_u', 10)
        self.sp_sub = self.create_subscription(Float32, '/set_point', self.sp_cb, 10)
        self.motor_sub = self.create_subscription(Float32, '/motor_output', self.motor_cb, 10)

        # Timer para la ejecución periódica del lazo de control
        self.timer = self.create_timer(self.sample_time, self.timer_cb)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Nodo de Control PID inicializado - Challenge Final')

    def sp_cb(self, msg):
        """Actualiza el valor de referencia (setpoint) deseado."""
        self.set_point = float(msg.data)

    def motor_cb(self, msg):
        """Recibe la retroalimentación de la velocidad actual del motor."""
        self.motor_y = float(msg.data)

    def timer_cb(self):
        """Calcula y publica la acción de control basada en el error actual."""
        error = self.set_point - self.motor_y
        dt = self.sample_time

        # Cálculo de componentes integral y derivativa
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        # Ley de control PID
        u = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Saturación de la señal de salida para compatibilidad con el hardware
        u_saturated = float(np.clip(u, -1.0, 1.0))

        # Publicación del comando de control y actualización del error previo
        msg_out = Float32(data=u_saturated)
        self.control_pub.publish(msg_out)
        self.prev_error = error

    def parameters_callback(self, params):
        """Permite la actualización dinámica de parámetros durante la ejecución."""
        for param in params:
            if param.name == 'kp': self.kp = param.value
            elif param.name == 'ki': self.ki = param.value
            elif param.name == 'kd': self.kd = param.value
            elif param.name == 'sample_time':
                self.sample_time = param.value
                self.timer.cancel()
                self.timer = self.create_timer(self.sample_time, self.timer_cb)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()