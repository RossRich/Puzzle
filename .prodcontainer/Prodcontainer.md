
## Сборка контейнера для jetsen nano

Открываем терминал в дериктории puzzle и запускаем сборку контейнера командой:
```
docker buildx build --platform linux/arm64 -f .prodcontainer/jetson.Dockerfile --network=host -t devdotnetorg/alpine-ssh:aarch64 .
```
