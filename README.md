# state_lattice_planner
# Przedmiot: Metody i Algorytmy Planowania Ruchu
# Projekt: Implementacja plannera typu state-lattice dla samochodu(C++)
# Prowadzący: dr inż. Tomasz Gawron
## Skład grupy
* Klaudia Sagat
* Michał Heit
* Patryk Gawron


## Opis projektu

Projekt przedstawia planner typu state-lattice dla samochodu. Etapy tworzenia projektu:
* Przygotowanie prymitywów ruchu na podstawie wielomianów,
* Implementacja procedury wyszukiwania podobnej do A*,
* Optymalizacja procedury sprawdzania kolizji i obsługi zbioru węzłów otwartych.



## Build:
```bash
mkdir -p catkin_ws

cd ~/catkin_ws

sudo apt-get install ros-noetic-moveit-commander ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-moveit-ros-planning-interface ros-noetic-moveit-planners-ompl ros-noetic-joint-trajectory-controller ros-noetic-tf-conversions ros-noetic-ur-client-library ros-noetic-industrial-robot-status-interface ros-noetic-position-controllers ros-noetic-robot-state-publisher ros-noetic-tf2-tools ros-noetic-moveit-simple-controller-manager

git clone https://github.com/patrykGawron/state_lattice_planner.git

catkin_make

source devel/setup.bash

chmod +x ~/catkin_ws/src/my_robot_world/scripts/*
```

## Visualisation Launch:
```bash
roslaunch my_robot_world myRobot.launch
```
