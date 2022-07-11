
## Сборка контейнера для jetson nano 

Открываем терминал в директории Puzzle и запускаем сборку контейнера командой:
```
docker buildx build --platform linux/arm64 -f .prodcontainer/jetson.Dockerfile --network=host -t prod/puzzle_nano:latest .
```
---
>[ERROR] [1657465485.402691263]: An exception has been thrown: Failed to open scan_element ..... Last Error: Permission denied

Отсутствуют разрешения для udev. На хосте находясь в папке **Puzzle/.prodcontainer**, необходимо выполнить :
```
sudo chmod +x scripts/*.sh && ./scripts/setup_udev_rules.sh
```