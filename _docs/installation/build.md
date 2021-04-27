---
layout: default
title:  "Build Instructions"
date:   2021-01-04 13:20:59 +0100
parent: "Installation"
---

# Build Instructions
1.  Check out the submodules
    ```bash
    git submodule init && git submodule update
    ```
    
2.  Create build and log folders
    ```bash
    mkdir build
    ```

3.  Build project
    ```bash
    cd build
    cmake ..
    cmake --build . -- -j4
    ```
    If you see an error during the `cmake ..` command that Qt or one of the Qt modules could
    not be found, you can ignore this message as this dependency is optional.