# ProtoBuffers

## Overview

This directory contains Protocol Buffers (protobuf) related files. The following files are included.

- [README.md](README.md): This file.
- RACS2Bridge_std_msgs
    - [RACS2Bridge_std_msgs.proto](RACS2Bridge_std_msgs.proto): Protocol Buffers file for standard ROS2 messages used in the RACS2 bridge.
    - C source and header files generated from `RACS2Bridge_std_msgs.proto`, generated using the protoc compiler,

        ```bash
        protc --c_out=. RACS2Bridge_std_msgs.proto
        ```

        - [RACS2Bridge_std_msgs.pb-c.c](RACS2Bridge_std_msgs.pb-c.c)
        - [RACS2Bridge_std_msgs.pb-c.h](RACS2Bridge_std_msgs.pb-c.h)

    - Python source file generated from `RACS2Bridge_std_msgs.proto`, generated using the protoc compiler,

        ```bash
        protoc --python_out=. RACS2Bridge_std_msgs.proto
        ```
    
        - [RACS2Brdige_std_msgs_pb2.py](RACS2Brdige_std_msgs_pb2.py)

- RACS2Bridge_geometry_msgs
    - [RACS2Bridge_geometry_msgs.proto](RACS2Bridge_geometry_msgs.proto): Protocol Buffers file for geometry messages used in the RACS2 bridge.
    - C source and header files generated from `RACS2Bridge_geometry_msgs.proto`, generated using the protoc compiler,

        ```bash
        protc --c_out=. RACS2Bridge_geometry_msgs.proto
        ```

        - [RACS2Bridge_geometry_msgs.pb-c.c](RACS2Bridge_geometry_msgs.pb-c.c)
        - [RACS2Bridge_geometry_msgs.pb-c.h](RACS2Bridge_geometry_msgs.pb-c.h)
        
    - Python source file generated from `RACS2Bridge_geometry_msgs.proto`, generated using the protoc compiler,

        ```bash
        protoc --python_out=. RACS2Bridge_geometry_msgs.proto
        ```
    
        - [RACS2Brdige_geometry_msgs_pb2.py](RACS2Brdige_geometry_msgs_pb2.py)
