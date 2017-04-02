# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Usage:

```
  /UnscentedKF [OPTION...] positional parameters

  -h, --help        Print help
  -i, --input arg   Input File
  -o, --output arg  Output file
  -r, --radar       use only radar data
  -l, --lidar       use only lidar data

  Example: UnscentedKF -i filename.txt -o outputName.txt
```