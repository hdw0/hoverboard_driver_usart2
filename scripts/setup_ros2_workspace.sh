#!/bin/bash
#source /opt/ros/humble/setup.bash
#sudo chmod +x ./scripts/setup_ros2_workspace.sh
#chmod +x ./scripts/setup_ros2_workspace.sh
#colcon build
#source install/setup.bash
#```

#!/bin/bash

# Проверка наличия ROS2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS2 Humble не установлен"
    exit 1
fi

# Источник ROS2
source /opt/ros/humble/setup.bash

# Установка зависимостей
sudo apt-get update
sudo apt-get install -y \
    libserial-dev \
    libboost-all-dev \
    python3-pip

# Python зависимости
pip3 install pyserial

# Сборка пакета
cd ~/ros2_ws
colcon build --packages-select hoverboard_driver_usart2

# Источник setup.bash
source ~/ros2_ws/install/setup.bash

echo "Настройка рабочего пространства завершена"
