import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from create_msgs.msg import Battery
import socket
from flask import Flask, request, jsonify
from flask_cors import CORS

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        # Publicador no tópico cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'diff_cont/cmd_vel', 10)

        # Subscreve ao tópico battery
        self.battery_sub = self.create_subscription(Battery, 'diff_cont/battery', self.battery_callback, 10)
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_pot = 0.0

        # Inicia o servidor HTTP
        self.app = Flask(__name__)
        CORS(self.app)

        @self.app.route('/atuadores', methods=['PUT'])
        def set_velocity():
            data = request.get_json()
            if 'linear' in data and 'angular' in data:
                self.publish_velocity(data['linear'], data['angular'])
                return jsonify({"message": "Mensagem cmd_vel atualizada"}), 200
            return jsonify({"error": "Dados inválidos"}), 400

        @self.app.route('/sensores', methods=['GET'])
        def get_battery():
            return jsonify({"Tensao": self.battery_voltage, "Corrente": self.battery_current, "Potencia": self.battery_pot}), 200

        # Obtém e imprime o IP da máquina
        ip_address = self.get_ip()
        self.get_logger().info(f"Nó iniciado no IP: {ip_address}")

        # Roda o servidor Flask em uma thread separada
        from threading import Thread
        self.server_thread = Thread(target=self.app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
        self.server_thread.daemon = True
        self.server_thread.start()

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear['x'])
        msg.linear.y = float(linear['y'])
        msg.linear.z = float(linear['z'])
        msg.angular.x = float(angular['x'])
        msg.angular.y = float(angular['y'])
        msg.angular.z = float(angular['z'])
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f"Velocidade publicada - Linear: {linear}, Angular: {angular}")

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage*(10**(-3)) #Tensão em Volts (V)
        self.battery_current = msg.current*(10**(-3)) #Amperagem em Amperes (A)
        self.battery_pot = self.battery_voltage*self.battery_current #Potencia em Watts (W)
        self.get_logger().info(f"Dados atualizados: Tensao={self.battery_voltage} V, {self.battery_current} A")

    def get_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "127.0.0.1"

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
