# RACS2 (ROS and cFS System 2) Example - Case.X

## Overview

This file contains a description of the RACS2 Example. The following movements can be observed in this Example.
01. The Example Publisher node(app) throws a message to RACS2 Bridge (Server).
02. The received message is transmitted by the RACS2 Bridge (Server) to the opposite RACS2 Bridge (Client).

## Directory List

- cFS/
  - apps/run_app
    - The Example Publisher for cFS.
  - sample_defs
    - The Example build settings for cFS.

## Setup

### Premise

- cFS must be cloned. If not, do the following:
  ```
  git clone https://github.com/nasa/cFS.git
  cd [cFS project path]
  git submodule init
  git submodule update
  ```

- RACS2 Bridge must be cloned. If not, do the following:
  ```
  git clone https://github.com/jaxa/racs2_bridge.git
  ```

- The RACS2 Bridge Server and Client must be up and connected.


### Procedure

- Preparation of execution environment on the cFS side.

  - If the cFS version is not 6.7.0a, go to the top of the cFS project directory and do the following:
    ```
    git checkout v6.7.0a
    git submodule init
    git submodule update
    ```

  - Go to the top of the cFS project directory and execute the following build command.
    ```
    cp cfe/cmake/Makefile.sample Makefile
    cp -r cfe/cmake/sample_defs sample_defs
    ```

  - Place cFS bridge and example directories in the cFS execution environment.
    ```
    cp -pr  racs2_bridge/cFS/Bridge/Client_C/apps/racs2_bridge_client [cFS project path]/apps/
    cp -pr  racs2_bridge/Example/Case.1/cFS/apps/run_app [cFS project path]/apps/
    cp -pr  racs2_bridge/Example/Case.1/cFS/sample_defs/* [cFS project path]/sample_defs/
    ```

  - Edit L.205 of "[cFS project path]/sample_defs/default_osconfig.h" as follows,
    ```
    #define OSAL_DEBUG_PERMISSIVE_MODE
    ```

  - Go to the top of the cFS project directory and execute the following build command.
    ```
    make prep
    make
    make install
    ```

- Start the bridge nodes.
  ```
  cd [ROS2 project path]
  source install/setup.bash
  ros2 run bridge_py_s bridge_py_s_node  --ros-args --params-file ./src/bridge_py_s/config/params.yaml
  ```

- Start the cFS applications, talker.
  ```
  cd [cFS project path]/build/exe/cpu1
  ./core-cpu1
  ```

- Check the messages that have been published and subscribed.

## How to exchange messages

- See `Document/HowToExchangeMessages.md`.

