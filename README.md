# To build

* need docker and nvidia-docker2

* fetch submodules   
$ git submodule update --init --recursive

* In the top-level project directory, run   
$ docker build -t aidrive .    
After the step, you will see an aidrive image in    
$docker image ls

* source the setup script   
$ source setup.sh   

* prefix any command with a dkb   
$ mkdir build   
$ cd build   
$ dkb cmake ..   
$ dkb make -j 12   
$ dkb ./apps/main
$ dkb ./src/newton/TrajectoryOptimizerTests