# PKL
Preferred Kinematic Library

[![CircleCI](https://circleci.com/gh/pfnet/pkl.svg?style=svg)](https://circleci.com/gh/pfnet/pkl)

## Requirements

- Eigen3
- cmake >= 2.8
- google benchmark (optional)
- google test (optional)

## Installation

Refer to the docker files under the `docker/` directory.

## Running the benchmark

```
docker build -t pkl_benchmark -f docker/<docker_file>
docker run -t pkl_benchmark "/root/benchmark/kdl/pkl_benchmark"
```

Where `docker_file` is one of

- `Dockerfile_ubuntu_gcc`
- `Dockerfile_ubuntu_clang`
- `Dockerfile_arch_gcc`

## Planned features

- catkin packaging
- MoveIt! plugin support
