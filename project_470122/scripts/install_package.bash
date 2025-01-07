#!/bin/bash

# Список необходимых пакетов
PACKAGES=(
    python3-rosdep
    ros-noetic-gazebo-ros-pkgs
    ros-noetic-rqt-controller-manager
    ros-noetic-controller-manager
    ros-noetic-gazebo-ros-control
    ros-noetic-ros-controllers
    ros-noetic-ros-control
    ros-noetic-hector-slam
    ros-noetic-navigation
    ros-noetic-laser-geometry
    ros-noetic-laser-filters
    ros-noetic-sensor-msgs
    ros-noetic-geometry-msgs
    ros-noetic-gmapping
    ros-noetic-rviz
    ros-noetic-tf
    ros-noetic-tf2-ros
    ros-noetic-tf2-tools
    ros-noetic-dwa-local-planner
    ros-noetic-global-planner
    ros-noetic-robot-localization
)

# Флаг для проверки, нужны ли установки
NEEDS_INSTALL=false

# Функция для проверки установки пакета
check_package_installed() {
    dpkg -l | grep -q "$1" > /dev/null 2>&1
    return $?
}

echo "Проверка необходимых пакетов для стека навигации..."

# Проверяем каждый пакет из списка
for PACKAGE in "${PACKAGES[@]}"; do
    if check_package_installed "$PACKAGE"; then
        echo "✅ $PACKAGE уже установлен."
    else
        echo "❌ $PACKAGE не установлен."
        NEEDS_INSTALL=true
    fi
done

# Устанавливаем недостающие пакеты
if $NEEDS_INSTALL; then
    echo "Обновление списка пакетов..."
    sudo apt update > /dev/null 2>&1

    echo "Установка недостающих пакетов..."
    for PACKAGE in "${PACKAGES[@]}"; do
        if ! check_package_installed "$PACKAGE"; then
            sudo apt install -y "$PACKAGE" > /dev/null 2>&1
            if [ $? -eq 0 ]; then
                echo "✅ Успешно установлен: $PACKAGE"
            else
                echo "❌ Ошибка при установке: $PACKAGE"
            fi
        fi
    done
else
    echo "Все необходимые пакеты уже установлены!"
fi
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash