## Требования:

* Ubuntu 20.04
* Графическая карта Nvidia
* Установленый docker
* Установленный пакет nvidia-docker2
* VSCode с установленым рсширением Docker (ms-azuretools.vscode-docker) и Remote-Container (ms-vscode-remote.remote-containers)


## Docker

Следуем [инструкции по установки](https://docs.docker.com/engine/install/ubuntu/) и [действия после успешной установки](https://docs.docker.com/engine/install/linux-postinstall/)

Далее ставим [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker). Это необходимо для работы видеокатры в контейнере, в противном случае gazebo может работать не стабильно или вообще не запуститься.


## VSCode 

[Инструкция по установки.](https://code.visualstudio.com/Download) После установки зайти в менеджер расширений и установить требуемые плагины.


## Развертывание Puzzle

Открывем VSCode и клонируем репозиторий:
```
git clone --recurse-submodules https://github.com/RossRich/Puzzle.git
```
Если репозиторий был склонирован без параметра --recurse-submodules, клонируем все подмодули:

```
git submodule update --init --recursive
```
*Если все прошло успешно, то VSCode предложит открыть проект в docker контейнере.*

## Сборка и запуск Puzzle

Когда контейнер соберется и откроется, в терминале выполняем сборку проекта:
```
catkin build
```
После сборки обновляем пути для ROS и запускаем симуляцию, вводим в терминале:
```
source devel/setup.bash
roslaunch px4 multi_uav_mavros_sitl.launch
```
