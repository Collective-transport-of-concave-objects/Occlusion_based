TO RUN THE OCCLUSION BASED COLLECTIVE TRANSPORT EXPERIMENTS:

Download the repository and replace the argos3-kheperaiv/src/testing folder by the testing folder in this repository

Then follow the following commands:

$ cd argos3-kheperaiv
$ mkdir build
$ cd build 
$ cmake -DCMAKE_BUILD_TYPE=Release ../src
$ make

$ cd ..
$ argos3 -c src/testing/convex_env.argos

