import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Parámetros de configuración de la señal de referencia
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('wave_type', 'sine')

        self.signal_publisher = self.create_publisher(Float32, '/set_point', 10)

        # Frecuencia de publicación de 10Hz
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.get_logger().info('Generador de SetPoint iniciado - Challenge Final')

    def timer_cb(self):
        """Genera y publica diferentes tipos de señales periódicas según el parámetro."""
        amp = self.get_parameter('amplitude').value
        omega = self.get_parameter('omega').value
        wave_type = self.get_parameter('wave_type').value

        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9

        # Lógica de generación de formas de onda
        if wave_type == 'sine':
            valor = amp * np.sin(omega * elapsed_time)
        elif wave_type == 'square':
            valor = float(amp * np.sign(np.sin(omega * elapsed_time)))
        elif wave_type == 'triangular':
            valor = float((2 * amp / np.pi) * np.arcsin(np.sin(omega * elapsed_time)))
        else:
            valor = amp * np.sin(omega * elapsed_time)

        self.signal_msg.data = float(valor)
        self.signal_publisher.publish(self.signal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SetPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()