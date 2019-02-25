#!/usr/bin/env bash

docker exec -it $(docker ps -qf "ancestor=mpb") bash