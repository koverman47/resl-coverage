# resl-coverage
## Instruction
Find `arl-unity-ros` folder and clone this repository into `src` and switch to `Prob_map` branch

```bash
# under arl-unity-ros/
cd src
git clone https://github.com/koverman47/resl-coverage.git
cd resl-coverage
# Switch to Prob_map branch
git checkout Prob_map
```

Back to the `arl-unity-ros` folder and rebuild it.

```bash
cd ../..
catkin build
```

Run the Prob_map test:

```bash
roscd resl_coverage/
# start 2 targets and 3 trackers
rosrun resl_coverage resl 2 3
```

You should get a running Unity window showing 3 drones and 2 vehicles. Maybe you need to move the camera to see all of them.

Start a new terminal:

```bash
roscd resl_coverage/
# this line below will start the tracker and pop up some windows showing the real time probability map
roslaunch resl_coverage resl_simulation_liftoff.launch
```
## File structure:
```
arl-unity-ros
    arl-unity-ros
        └── ...
    external
        └── ...
    resl-coverage
        README.md
        resl-coverage
                ├── CMakeLists.txt
                ├── package.xml
                ├── resl    (Use this to start the simulator)
                ├── launch
                │   └── resl_simulation_liftoff.launch
                ├── logs
                │   └── ...
                ├── msg
                │   ├── MultiStateEstimate.msg
                │   ├── ProbMapData.msg     (New msg type of prob map data)
                │   ├── ShareableMeasInfo.msg   (Unused)
                │   └── StateEstimate.msg
                ├── scripts
                │   └── ...
                ├── src
                │   ├── archive
                │   │   └── ...
                │   ├── monitor_fov.py      (Kirby's Kalman tracker, I am using it as a reference)
                │   ├── monitor_prob_map.py     (Zhilong's probabilty map tracker)
                │   ├── prob_map.py     (Zhilong's prob map lib)
                │   └── utils
                │       └── ...
                └── srv
                    ├── MeasShare.srv       (Old prob map service)
                    └── ...
```
