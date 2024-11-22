# hummanArm

## by: Ali Faisal Yousef

c++ library for kinematics (forward and inverse) for 5 DOF robotics arm,
and python simulation for the robot

## requirements:

## cmake (add it to path in environoment variables for windows)

## gcc compiler in "C:/msys64/ucrt64/bin/gcc.exe" (for windows only), or set it in CMakeLists.txt

## for linux build:

```
mkdir build && cd build
cmake ..
make
sudo make install
```

## for windows build:

open terminal as administrator, go to the repo directory and run:

```
mkdir build; cd build
cmake -G "MinGW Makefiles" ..
cmake --build .
cmake --install .
```

## for testing, install some python packages:

```
pip3 install numpy ikpy 'ikpy[plot]'
```

## for docs see /tests dir.
