from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        # Mensaje de bienvenida al sistema
        LogInfo(msg="Iniciando Challenge Final..."),
        
        # Nodo Generador de Setpoint (Referencia)
        Node(
            package='motor_control',
            executable='set_point',
            name='sp_gen',
            parameters=[{
                'amplitude': 2.0,
                'omega': 1.0,
                'wave_type': 'sine'
            }]
        ),
        
        # Nodo Controlador PID (Procesamiento)
        Node(
            package='motor_control',
            executable='ctrl',
            name='controller_node',
            parameters=[{
                'kp': 0.93,
                'ki': 1.4,
                'kd': 0.0,
                'sample_time': 0.01
            }]
        ),

        # Nodo de Visualización (rqt_plot)
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='plotter',
            arguments=[
                '/set_point/data', 
                '/motor_output/data', 
                '/motor_input_u/data'
            ],
        )
    ])