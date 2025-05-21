#!/bin/bash
set -e

echo "Configurando ambiente ROS..."
source /opt/ros/humble/setup.bash

echo "Navegando para o workspace..."
cd /workspace/automation_ws

echo "Compilando o workspace..."
colcon build

# Se o workspace foi compilado com sucesso, carregue-o
if [ -f "./install/setup.bash" ]; then
  echo "Carregando o workspace compilado..."
  source ./install/setup.bash
  
  echo "Iniciando o launch file..."
  ros2 launch automation_robot_launch_pkg automation_robot_launch.launch.py
else
  echo "ERRO: A compilação do workspace falhou ou o arquivo setup.bash não foi encontrado!"
  exit 1
fi

# Execute qualquer comando passado como argumento
exec "$@"