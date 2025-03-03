#!/bin/bash

# Проверка аргументов
if [ "$1" = "python" ]; then
    NODE_TYPE="ros_hoverboard_uart_node.py"
else
    NODE_TYPE="hoverboard_node"
fi

# Источник ROS2 и рабочего пространства
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Запуск узла
ros2 run hoverboard_driver_usart2 $NODE_TYPE