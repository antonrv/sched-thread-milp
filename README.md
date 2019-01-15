# Sched-Thread-MILP project

This is a implementation of a Mixed-Integer Linear Program using Gurobi as the backend solver. It returns solutions for a threadable-task scheduling problem instances under time or energy optimization and optionally power budget constraints.

## Prerequisites

**C++ compiler**

**CMake** ( https://cmake.org/ )

**GUROBI** software ( http://www.gurobi.com/index ) with a valid license.

## Compile and test example

```sh
export GUROBI_HOME=[YOUR_GUROBI_PATH] # e.g. /opt/gurobi810/linux64

# In root folder
mkdir build/
cd build
cmake ../
make
ctest
```

All 8 tests should pass.

## Use cases

After building, the binary build/bin/stm will accept the following arguments.

**Parameters**

```sh
--n=<INTEGER>           # Number of tasks
--h=<INTEGER>           # Number of threading configurations
--c=<INTEGER>           # Number of processors
--optimize=<STRING>     # time or energy (default to time)
--idlepower=<FLOAT>     # Idle power (optional)
--maxpower=<FLOAT>      # Maximum power (optional)
--input=<STRING>        # Filename storing previous gurobi solution (optional)
--bigm=<FLOAT>          # Upper bound for the makespan
--inorder=<BOOL>        # To force in-order execution
--threads=<INTEGER>     # Number of CPU threads for solving
--timeout<INTEGER>      # Solver timeout in seconds
```

**Matrices**

* *Delay* matrix that stores the time (float) required for a task running with a threading configuration.
* *Power* matrix that stores the power (float) required for a task running with a threading configuration.
* *Capacity* matrix that stores the number of cores (int) required for a task running with a threading configuration.
* *Precedence* matrix stores (bool) in each column the sucessors for a given row.

Matrix elements must be separated by comma (see examples in test/).

```sh
--delay=<FILENAME>      # N rows (# tasks) X H columns (# threading configurations)
--power=<FILENAME>      # N rows X H columns
--capacity=<FILENAME>      # N rows X H columns
--preced=<FILENAME>      # N rows X N columns
```

## Author
* Antón Rey

## License
This project is licensed under the MIT License.

## Acknowledgments
ArTeCS group - Universidad Complutense de Madrid.
