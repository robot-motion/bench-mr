---
layout: default
title:  "Using Docker"
date:   2021-01-04 13:20:59 +0100
parent: "Installation"
---

# Using Docker

1. Build the Docker image
    ```bash
    docker build -t mpb .
    ```

2. Run the image to be able to access the Jupyter Lab instance on port 8888 in your browser from where you can run and evaluate benchmarks:
    ```bash
    docker run -p 8888:8888 -it mpb
    ```
   Optionally, you can mount your local `mpb` copy to its respective folder inside the docker via
   ```bash
   docker run -p 8888:8888 -v $(pwd):/root/code/mpb -it mpb
   # use %cd% in place of $(pwd) on Windows
   ```
   Now you can edit files from outside the docker and use this container to build and run the experiments.

   You can connect multiple times to this same running docker, for example if you want to access it from multiple shell instances via
   ```bash
   docker exec -it $(docker ps -qf "ancestor=mpb") bash
   ```
   Alternatively, run the provided script `./docker_connect.sh` that executes this command.