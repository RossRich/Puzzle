
## Сборка контейнера для jetson nano 

Открываем терминал в дериктории puzzle и запускаем сборку контейнера командой:
```
docker buildx build --platform linux/arm64 -f .prodcontainer/jetson.Dockerfile --network=host -t prod/puzzle_nano:latest .
```
