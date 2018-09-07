# SDPTrajectoryGeneration
This repository is related with the author's master thesis titled "Redundancy control through SDP Lexicographic Optimization".

Based on [Peter Corke Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox) for Matlab, it contains a framework to generate trajectories for a redundant robot. Each task has its own priority and which will be taken into account when solving the trajectory.

Some tasks are already implemented in a 4-link planar manipulator: Pose tracking task (position and orientation), joint limits task as well as an obstacle avoidance task.

A nice exercise to do is to play with the task priorities to check their effect on the trajectory resolution.

## Software dependencies

### SDPA
You can do this procedure inside a folder of your choice. In order to ease the procedure, we start from the ${HOME} folder.

- Create main folder: `cd ${HOME} && mkdir sdpa`
* **OpenBLAS**  
    - Download: `cd sdpa && git clone https://github.com/xianyi/OpenBLAS.git && cd OpenBLAS`
    - Before compiling OpenBLAS, disable the multithread options in Makefile.rule (inside of OpenBLAS folder): Uncomment `USE_THREAD = 0`  
    - Compile and install depending on your system architecture:
        * x32: `make BINARY=32 CC=gcc FC=gfortran USE_OPENMP=0 NO_CBLAS=1 NO_WARMUP=1 libs netlib`    
        * x64: `make BINARY=64 CC=gcc FC=gfortran USE_OPENMP=0 NO_CBLAS=1 NO_WARMUP=1 libs netlib`
    
* **SDPA**
    - Download: `cd sdpa && wget https://sourceforge.net/projects/sdpa/files/sdpa/sdpa_7.3.8.tar.gz --no-check-certificate && tar -xvzf sdpa_7.3.8.tar.gz && rm sdpa_7.3.8.tar.gz`
    - Compile: 
        
        `cd sdpa_7.3.8 && export CC=gcc && export CXX=g++ && export FC=gfortran && export CFLAGS="-funroll-all-loops" && export CXXFLAGS="-funroll-all-loops" && export FFLAGS="-funroll-all-loops"`
        
        `./configure --prefix=$PWD/../ --with-blas="$PWD/../OpenBLAS/libopenblas.a" --with-lapack="$PWD/../OpenBLAS/libopenblas.a"`
        
        `make && sudo make install`
