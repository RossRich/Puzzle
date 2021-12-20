## Требования:

* Ubuntu 20.04
* Графическая карта Nvidia
* Установленный docker
* Установленный пакет nvidia-docker2
* VSCode с установленным расширением Docker (ms-azuretools.vscode-docker) и Remote-Container (ms-vscode-remote.remote-containers)


## Docker

Следуем [инструкции по установке](https://docs.docker.com/engine/install/ubuntu/) и [действиям после успешной установки](https://docs.docker.com/engine/install/linux-postinstall/)

Далее ставим [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker). Это необходимо для работы видеокарты в контейнере. В противном случае gazebo может работать не стабильно.


## VSCode 

[Инструкция по установке.](https://code.visualstudio.com/Download) После установки зайти в менеджер расширений и установить требуемые плагины.


## Развертывание Puzzle

Открываем VSCode и клонируем репозиторий:
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
Прописываем пути окружения:

* Для Gazebo:
```
source /use/share/gazebo.bash
```
* для px4
```
source ./workspaces/Puzzle/src/PX4-Autopilot/Tools/setup_gazebo.bash /workspaces/Puzzle/src/PX4-Autopilot /workspaces/Puzzle/src/PX4-Autopilot/build/px4_sitl_default
```
* для puzzle
```
export GAZEBO_MODEL_PATH=/workspaces/Puzzle/src/PxSitl/gazebo_sitl/models:$GAZEBO_MODEL_PATH
```
* для ROS
```
source devel/setup.bash
```
----
Запуск симулятора:
```
roslaunch PxSitl iris_puzzle.launch
```
## Ошибки

>[Err] [RenderEngine.cc:749] Can't open display: :0
[Wrn] [RenderEngine.cc:89] Unable to create X window. Rendering will be disabled
[Wrn] [RenderEngine.cc:292] Cannot initialize render engine since render path type is NONE. Ignore this warning ifrendering has been turned off on purpose.
No protocol specified
[Wrn] [GuiIface.cc:120] could not connect to display :0

Вводим в терминале (на хосте, т.е. не в контейнере)
```
xhost +local:root
```
----
>gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:734: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.

Вводим в когда загрузится контейнер
```
source /usr/share/gazebo/setup.sh
```
---