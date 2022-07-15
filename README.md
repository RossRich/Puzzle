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
`
## Сборка Px4
Более подробно о сборке автопилота можно прочитать на [:link: оффициальном сайте](https://docs.px4.io/master/en/dev_setup/building_px4.html#px4-make-build-targets). В [:link:другом разделе](https://docs.px4.io/master/en/simulation/gazebo.html#gazebo-simulation) можно найти информацию о сборке автопилота для симулятора.
Перед сборкой пакета ROS необходимо сорбать автопилот:
```
(cd src/PX4-Autopilot && make px4_sitl_default gazebo)
```
Если все собралось успешно, то запуститься симуляция

## Сборка плагинов для Gazebo
В симуляторе используются функции требующие плагины. Их необходимо собрать:
```
(cd src/puzzle_gazebo/plugins && mkdir build)
(cd src/puzzle_gazebo/plugins/build && cmake ../ && make -j8)
```

## Сборка и запуск Puzzle

Когда контейнер соберется и откроется, в терминале выполняем сборку проекта:
```
catkin build
```
Прописываем пути окружения:

* Для Gazebo:
```
source /usr/share/gazebo/setup.bash
```
* для px4
```
source /workspaces/Puzzle/src/PX4-Autopilot/Tools/setup_gazebo.bash /workspaces/Puzzle/src/PX4-Autopilot /workspaces/Puzzle/src/PX4-Autopilot/build/px4_sitl_default
```
* для puzzle
```
export GAZEBO_MODEL_PATH=/workspaces/Puzzle/src/puzzle_gazebo/models:$GAZEBO_MODEL_PATH
```
```
export GAZEBO_PLUGIN_PATH=/workspaces/Puzzle/src/puzzle_gazebo/plugins/build:$GAZEBO_PLUGIN_PATH
```
* для ROS
```
source devel/setup.bash
```
----
Запуск симулятора:
```
roslaunch puzzle_gazebo launcher.launch
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
При сборки контейнера выдаёт ошибку подключения к интернету или не загружаются пакеты через менеджер apt. Для решения проблеммы необходимо перезагрузить Docker. Вводим в терминале (на хосте, т.е. не в контейнере)

```
sudo systemctl restart docker
```
----
>Gazebo долго открывается и выдает предупреждение: [Wrn] [ModelDatabase.cc:340] Getting models from[http://models.gazebosim.org/]. This may take a few seconds.

Отсутствуют необходимые модели и Gazebo пытается загрузить их. Все мадели добавлены в репозиторий. Необходимо прописать: 
```
source /workspaces/Puzzle/src/PX4-Autopilot/Tools/setup_gazebo.bash /workspaces/Puzzle/src/PX4-Autopilot /workspaces/Puzzle/src/PX4-Autopilot/build/px4_sitl_default
```
----
>gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:734: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.

Вводим в когда загрузится контейнер
```
source /usr/share/gazebo/setup.sh
```
---
>[Err] [Plugin.hh:178] Failed to load plugin libYPGunPlugin.so: libYPGunPlugin.so: cannot open shared object file: No such file or directory
[Err] [Plugin.hh:178] Failed to load plugin libGUIShootPlugin.so: libGUIShootPlugin.so: cannot open shared object file: No such file or directory
[Err] [MainWindow.cc:2092] Unable to create gui overlay plugin with filename[libGUIShootPlugin.so]

Необходимо собрать плагины для Gazebo
```
(cd src/puzzle_gazebo/plugins && mkdir build)
(cd src/puzzle_gazebo/plugins/build && cmake ../ && make -j8)
```
---
>[ WARN] [1656580307.684461415]: OGRE EXCEPTION(3:RenderingAPIException): Unable to create a suitable GLXContext in GLXContext::GLXContext at /build/ogre-1.9-B6QkmW/ogre-1.9-1.9.0+dfsg1/RenderSystems/GL/src/GLX/OgreGLXContext.cpp (line 61) 

В контейнере с ubuntu 18.04 и  ROS melodic не запускается RVIZ. Для решения еобходимо добавить Dockerfile следующих строк:
```
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
   ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
```
---
>[ERROR] [1657465485.402691263]: An exception has been thrown: Failed to open scan_element ..... Last Error: Permission denied

Отсутствуют разрешения для udev. На хосте находясь в папке **Puzzle/.devcontainer**, необходимо выполнить :
```
sudo chmod +x scripts/*.sh && ./scripts/setup_udev_rules.sh
```