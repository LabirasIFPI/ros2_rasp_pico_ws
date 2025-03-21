# Controlador ROS 2

Este repositório contém o código-fonte para um workspace ROS 2 contendo um pacote com um nó (node) que executa um servidor HTTP apto a receber e publicar informações de cmd_vel através de um método PUT, e retornar informações acerca de dados da bateria por meio de um método GET.

## Requisitos
- Ubuntu 22.04
- ROS 2 Humble Hawksbill

## Funcionalidades

### Requisições HTTP aceitas
- Requisição GET para coleta dos dados de bateria (`/sensores`)
- Requisição PUT para envio de mensagens cmd_vel (`/atuadores`)

## Configuração e Uso

1. Clone este repositório:
   ```
   git clone https://github.com/LabirasIFPI/ros2_rasp_pico_ws.git
   ```

2. Execute o comando a seguir no terminal dentro do diretório da pasta do repositório:
   ```
   colcon build
   ```

3. Para executar o nó ROS 2, execute dentro do diretório da pasta do repositório:
   ```
   source install/setup.bash
   ros2 run server_control server_control
   ```

## Possíveis dificuldades
Caso o seu node ROS 2 não execute corretamente, verifique se executou os comandos nos caminhos corretos.



## Direitos de Uso
Este projeto é disponibilizado para leitura e citação em outros trabalhos. Caso utilize este projeto como referência, por favor, forneça os devidos créditos ao autor.

## Autor
Desenvolvido por Paulo Roberto Araújo Leal.
