import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from create_msgs.msg import Battery
import socket
import json
import threading

HOST = '192.168.137.239'
PORT = 5000

class HttpRosNode(Node):
    def __init__(self):
        super().__init__('http_ros_node')
        self.array_robots = ['Roomba', 'OmniBot', 'Turtlesim']
        
        self.cmd_vel_pub_Roomba = self.create_publisher(Twist, '/diff_cont/cmd_vel', 10)
        self.cmd_vel_pub_OmniBot = self.create_publisher(Twist, '/omnibot/cmd_vel', 10)
        self.cmd_vel_pub_Turtlesim = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.battery_data = Battery()
        self.create_subscription(Battery, 'battery_state', self.battery_callback, 10)

        # Inicia o servidor HTTP em uma thread separada
        thread = threading.Thread(target=self.start_http_server, daemon=True)
        thread.start()
        self.get_logger().info(f'Servidor HTTP iniciado em {HOST}:{PORT}')

    def battery_callback(self, msg):
        self.battery_data = msg

    def start_http_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(10)
            while True:
                conn, _ = s.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(conn,), daemon=True)
                client_thread.start()

    def handle_client(self, conn):
        conn.settimeout(5.0)  # encerra conexões lentas
        try:
            request = conn.recv(1024).decode()
            if not request:
                return

            linha_req = request.splitlines()[0]
            metodo, caminho, *_ = linha_req.split()

            if metodo == 'GET' and caminho == '/sensores':
                self.get_logger().info('Requisição GET recebida.')
                corpo = json.dumps({
                    "voltage": self.battery_data.voltage / 1000, # Conversão de mV para V
                    "current": self.battery_data.current / 1000, # Conversão de mA para A
                    "charge": self.battery_data.charge / 1000, # Conversão de mAh para Ah
                    "max_id_robo": len(self.array_robots)
                })
                resposta = (
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/json\r\n"
                    f"Content-Length: {len(corpo)}\r\n"
                    "\r\n"
                    f"{corpo}"
                )
                conn.sendall(resposta.encode())

            elif metodo == 'PUT' and caminho == '/atuadores':
                self.get_logger().info('Requisição PUT recebida.')
                partes = request.split("\r\n\r\n", 1)
                if len(partes) == 2:
                    corpo_json = partes[1]
                    try:
                        dados = json.loads(corpo_json)
                        twist = Twist()
                        id_robo = dados.get('robo_id', 0)
                        linear = dados.get('linear', {})
                        angular = dados.get('angular', {})
                        
                        twist.linear.x = float(linear.get('x', 0.0))
                        twist.linear.y = float(linear.get('y', 0.0))
                        twist.linear.z = float(linear.get('z', 0.0))
                        twist.angular.x = float(angular.get('x', 0.0))
                        twist.angular.y = float(angular.get('y', 0.0))
                        twist.angular.z = float(angular.get('z', 0.0))
                        
                        if (id_robo <= len(self.array_robots)):
                            self.get_logger().info(f'Controlando: {self.array_robots[id_robo-1]}')
                        else:
                            self.get_logger().info(f'Modo de debug ativado!')
                        if (id_robo == 1):
                            self.cmd_vel_pub_Roomba.publish(twist)
                        elif (id_robo == 2):
                            self.cmd_vel_pub_OmniBot.publish(twist)
                        elif (id_robo == 3):
                            self.cmd_vel_pub_Turtlesim.publish(twist)
                        else:
                            self.get_logger().info(f'data: {dados}')

                        corpo = json.dumps({"status": "ok"})
                        resposta = (
                            "HTTP/1.1 200 OK\r\n"
                            "Content-Type: application/json\r\n"
                            f"Content-Length: {len(corpo)}\r\n"
                            "\r\n"
                            f"{corpo}"
                        )
                        conn.sendall(resposta.encode())
                    except json.JSONDecodeError:
                        conn.sendall(b"HTTP/1.1 400 Bad Request\r\n\r\n")
                else:
                    conn.sendall(b"HTTP/1.1 400 Bad Request\r\n\r\n")
            else:
                conn.sendall(b"HTTP/1.1 404 Not Found\r\n\r\n")
        except socket.timeout:
            self.get_logger().warn('Conexão expirada por inatividade.')
        except Exception as e:
            self.get_logger().error(f'Erro ao lidar com cliente: {e}')
        finally:
            conn.close()


def main(args=None):
    rclpy.init(args=args)
    node = HttpRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
