This repository aims to rerun the tagged code that was used for [Valentin](https://vhartmann.com/)'s paper ["Towards computing low-makespan solutions for multi-arm multi-task planning problems"](https://arxiv.org/abs/2305.17527). A writeup of the content can be found in Valentin's [blog](https://vhartmann.com/low-makespan-tamp/) and the repo that is in active development is [here](https://github.com/vhartman/24-data-gen/tree/master). 

# Installation 
![sys-vesrion](https://img.shields.io/badge/Ubuntu-20.04-blue)

The code depends on the code of [rai](https://github.com/vhartman/rai), [rai-robotModels](https://github.com/vhartman/rai-robotModels) and [rai-manip](https://github.com/vhartman/rai-manip).

```
mkdir your_folder
cd your_folder

git clone https://github.com/vhartman/rai.git
cd rai
# The following two commands depend on the config.mk -- see below
make -j1 printUbuntuAll    # for your information: what the next step will install
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt

# Install pybind
sudo apt install --yes python3-dev python3 python3-pip
python3 -m pip install --user numpy pybind11 pybind11-stubgen

# TO AVOID "displayTrajectory" error, comment out lines 59-63 in rai/rai/ry/ry-LGP_Tree.cpp
make -j4
make -j4 tests bin  # (optional) 
make runTests      # (optional) compile and run the essential tests

cd ..
git clone https://github.com/yuezhezhang/rai-robotModels.git
git clone https://github.com/yuezhezhang/rai-manip.git 
git clone https://github.com/yuezhezhang/valentin_robot_stippling.git

# Install other dependencies
sudo apt install libspdlog-dev
sudo apt install libfmt-dev
```

The folder structure should be:
```
├── your_folder
│   ├── rai
│   ├── rai-robotModels
│   ├── rai-manip
│   ├── valentin_robot_stippling
```

Please change the rai path from `rai-fork` to `rai` in the Makefile in `rai-manip/PlanningSubroutines` and `rai-manip/Manip`.

Compilation then works with
```
cd your_folder/valentin_robot_stippling
make -j4
```
Execution goes with
```
./x.exe -mode single_arm -save_video false
```

To save the video, just specify `-save_video true` in the command line, the ppms will then be saved in the `video` folder.

To transform the saved ppms into an mp4, run in terminal:
```
cd video
ffmpeg -framerate 20 -i filename%04d.ppm -c:v libx264 -crf 25 -vf "scale=400:400,format=yuv420p" -movflags +faststart videoname.mp4
```

To transform the mp4 into a high-quality gif, run:
```
ffmpeg -i videoname.mp4 pngname%04d.png
gifski -o videoname.gif pngname*.png
```
>[!NOTE]
The default working branch of `rai` is `assembly`, you can switch to branch `changes` for faster computation, but the switch will introduce some unnnecessary outputs of types and frames of the configurations, to disable this, you can comment the lines 44-45 in `rai/rai/Geo/fclInterface.cpp`. These outputs are related to the function `komo.setModel(C, true)` and the variable `FCL` in `rai/config.mk`.

# Demos

| Random Search | Greedy Search |
|---|---|
<img src="./video/bin_picking/random_search.gif" alt="005" style="zoom: 70%;" /> | <img src="./video/bin_picking/greedy_search.gif" alt="004" style="zoom: 70%;" /> |


