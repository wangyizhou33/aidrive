# To build

* fetch submodules   
$ git submodule update --init --recursive
* install ceres under ${aidrive}/3rdparty/ceres/ceres-solver/build/install   
Installation instruction can be found here
http://ceres-solver.org/installation.html

* In the top-level project directory, run   
$ docker build -t aidrive .    
After the step, you will see an aidrive image in    
$docker image ls

* source the setup script   
$ source setup.sh   
$ lunch

* prefix any command with a dkb   
$ dkb cmake ..   
$ dkb make -j 12