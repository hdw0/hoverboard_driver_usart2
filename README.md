# Hoverboard Driver USART2

ROS2 драйвер для управления ховербордом через UART интерфейс.

## Описание
Этот пакет предоставляет ROS2 узел для управления ховербордом через последовательный порт. 
Поддерживает:
- Управление скоростью через топик cmd_vel
- Публикацию одометрии в топик odom
- Трансформации TF
- Обратную связь о состоянии устройства

## Установка и настройка

1. Установите ROS2 Humble:
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```

2. Установите необходимые зависимости:
    ```bash
    sudo apt-get update
    sudo apt-get install -y \
        libserial-dev \
        libboost-all-dev \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        python3-pip
    
    # Python зависимости
    pip3 install pyserial
    ```

3. Склонируйте репозиторий в рабочее пространство ROS2:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone <URL репозитория>
    cd ~/ros2_ws
    ```

4. Добавьте права на выполнение скриптов:
    ```bash
    chmod +x scripts/setup_ros2_workspace.sh
    chmod +x scripts/run_hoverboard_node.sh
    ```

5. Запустите скрипт настройки рабочего пространства:
    ```bash
    ./scripts/setup_ros2_workspace.sh
    ```

## Запуск узла

Для запуска узла hoverboard выполните:
```bash
./scripts/run_hoverboard_node.sh
```

## Изменение рабочего порта

Для изменения рабочего порта используйте параметр `serial_port`:
```bash
ros2 run hoverboard_driver_usart2 hoverboard_node --ros-args -p serial_port:=/dev/ttyUSB1
```

## Настройка для WSL2

1. Скачайте и установите [usbipd-win](https://github.com/dorssel/usbipd-win/releases)

2. Установите необходимые пакеты в WSL2:
    ```bash
    sudo apt install linux-tools-generic hwdata
    sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*-generic/usbip 20
    ```

3. В Windows PowerShell (от администратора) выполните:
    ```powershell
    # Посмотреть список доступных USB устройств
    usbipd wsl list

    # Подключить нужный COM-порт (замените BUSID на ID вашего устройства)
    usbipd wsl attach --busid <BUSID>
    ```

4. В WSL2 проверьте, что устройство появилось:
    ```bash
    ls /dev/ttyUSB*
    # или
    ls /dev/ttyACM*
    ```

5. Добавьте текущего пользователя в группу dialout для доступа к COM-порту:
    ```bash
    sudo usermod -a -G dialout $USER
    # Перезапустите терминал или выполните
    newgrp dialout
    ```

## Структура проекта

```
ros2_ws/
├── src/
│   └── hoverboard_driver_usart2/
│       ├── src/
│       │   └── hoverboard_node.cpp
│       ├── cmake/
│       │   └── serial-config.cmake
│       ├── CMakeLists.txt
│       └── package.xml
├── scripts/
│   ├── setup_ros2_workspace.sh
│   └── run_hoverboard_node.sh
└── .vscode/
    ├── settings.json
    └── launch.json
```

## Использование

После установки вы можете:
1. Управлять ховербордом, публикуя сообщения в топик cmd_vel:
    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
      x: 0.5
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"
    ```

2. Просматривать одометрию:
    ```bash
    ros2 topic echo /odom
    ```

3. Просматривать обратную связь:
    ```bash
    ros2 topic echo /hoverboard_feedback
    ```

## Лицензия
MIT License

## Автор
[Ваше имя]

## Contributing
Pull requests приветствуются. Для существенных изменений, пожалуйста, создайте issue для обсуждения предлагаемых изменений.