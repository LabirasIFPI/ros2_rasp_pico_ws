#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "create_msgs/msg/battery.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "microhttpd.h"

#include <nlohmann/json.hpp>  // Incluindo a biblioteca para JSON

using namespace std::chrono_literals;
using namespace std;

class RobotControlNode : public rclcpp::Node
{
public:
    RobotControlNode()
    : Node("robot_control_node")
    {
        // Publicador no tópico cmd_vel
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("diff_cont/cmd_vel", 10);
        
        // Subscreve ao tópico de bateria
        battery_sub_ = this->create_subscription<create_msgs::msg::Battery>(
            "diff_cont/battery", 10, std::bind(&RobotControlNode::battery_callback, this, std::placeholders::_1));
        
        // Inicializa os dados da bateria
        battery_voltage_ = 0.0;
        battery_current_ = 0.0;
        battery_power_ = 0.0;
        
        // Inicia o servidor HTTP
        start_http_server();
    }

    void start_http_server()
    {
        // Cria um servidor HTTP na porta 5000
        server_ = MHD_start_daemon(MHD_USE_THREAD_PER_CONNECTION, 5000, NULL, NULL, &RobotControlNode::http_request_callback, this, MHD_OPTION_END);
        if (server_ == NULL)
        {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível iniciar o servidor HTTP");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Servidor HTTP iniciado na porta 5000");
        }
    }

    // Callback de recebimento dos dados de bateria
    void battery_callback(const create_msgs::msg::Battery::SharedPtr msg)
    {
        battery_voltage_ = msg->voltage * 1e-3;  // Tensão em Volts
        battery_current_ = msg->current * 1e-3;  // Corrente em Amperes
        battery_power_ = battery_voltage_ * battery_current_;  // Potência em Watts
    }

    // Função que trata as requisições HTTP
    static MHD_Result http_request_callback(void *cls, struct MHD_Connection *connection, const char *url, const char *method,
                                            const char *version, const char *upload_data, size_t *upload_data_size, void **con_cls)
    {
        RobotControlNode *node = static_cast<RobotControlNode *>(cls);


        RCLCPP_INFO(node->get_logger(), "%s", upload_data);
        // Apenas resposta para o endpoint "/sensores"
        if (strcmp(url, "/sensores") == 0)
        {
            // Cria um JSON com os dados da bateria
            std::ostringstream json_stream;
            json_stream << "{"
                        << "\"Tensao\": " << node->battery_voltage_ << ", "
                        << "\"Corrente\": " << node->battery_current_ << ", "
                        << "\"Potencia\": " << node->battery_power_
                        << "}";

            std::string json_data = json_stream.str();

            struct MHD_Response *response = MHD_create_response_from_buffer(json_data.size(), (void *)json_data.c_str(), MHD_RESPMEM_MUST_COPY);
            int ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
            MHD_destroy_response(response);

            return (MHD_Result)ret;
        }

        // Para requisições PUT no endpoint "/atuadores"
        else if ((strcmp(url, "/atuadores") == 0) && (strcmp(method, "PUT") == 0))
        {
            // Se o upload_data não for NULL, processa os dados da requisição PUT
            if (upload_data != NULL)
            {
                RCLCPP_INFO(node->get_logger(), "PUT");
                // Cria um stringstream para ler os dados JSON
                std::istringstream json_stream(upload_data);
                nlohmann::json json_data;
                json_stream >> json_data;

                // Verifica se as chaves 'linear' e 'angular' existem no JSON
                if (json_data.contains("linear") && json_data.contains("angular"))
                {
                  RCLCPP_INFO(node->get_logger(), "PUT2");
                    // Cria a mensagem de Twist
                    geometry_msgs::msg::Twist msg;

                    msg.linear.x = json_data["linear"]["x"];
                    msg.linear.y = json_data["linear"]["y"];
                    msg.linear.z = json_data["linear"]["z"];
                    msg.angular.x = json_data["angular"]["x"];
                    msg.angular.y = json_data["angular"]["y"];
                    msg.angular.z = json_data["angular"]["z"];

                    // Publica a mensagem no tópico cmd_vel
                    node->cmd_vel_publisher_->publish(msg);

                    // Responde com sucesso
                    std::string response_message = "{\"message\": \"Velocidade atualizada com sucesso!\"}";
                    struct MHD_Response *response = MHD_create_response_from_buffer(response_message.size(), (void *)response_message.c_str(), MHD_RESPMEM_MUST_COPY);
                    int ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
                    MHD_destroy_response(response);

                    return (MHD_Result)ret;
                }
                else
                {
                    // Se as chaves não existirem, retorna erro
                    std::string error_message = "{\"error\": \"Dados inválidos, 'linear' e 'angular' são obrigatórios.\"}";
                    struct MHD_Response *response = MHD_create_response_from_buffer(error_message.size(), (void *)error_message.c_str(), MHD_RESPMEM_MUST_COPY);
                    int ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
                    MHD_destroy_response(response);

                    return (MHD_Result)ret;
                }
            }
            else
            {
                // Se não houver dados no corpo da requisição, retorna erro
                std::string error_message = "{\"error\": \"Dados não fornecidos\"}";
                struct MHD_Response *response = MHD_create_response_from_buffer(error_message.size(), (void *)error_message.c_str(), MHD_RESPMEM_MUST_COPY);
                int ret = MHD_queue_response(connection, MHD_HTTP_BAD_REQUEST, response);
                MHD_destroy_response(response);

                return (MHD_Result)ret;
            }
        }

        // Para outras requisições, retorna 404
        return MHD_NO;
    }

    ~RobotControlNode()
    {
        if (server_ != NULL)
        {
            MHD_stop_daemon(server_);
        }
    }

private:
    rclcpp::Subscription<create_msgs::msg::Battery>::SharedPtr battery_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    double battery_voltage_, battery_current_, battery_power_;
    struct MHD_Daemon *server_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlNode>());
    rclcpp::shutdown();
    return 0;
}
