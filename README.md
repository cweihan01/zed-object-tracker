# ZED Object Tracker

## Prerequisites

- ROS 2 Humble
- Python 3.10

## Running a ROS Bag

1. Copy the bag into the container
```bash
docker cp <local-bag-dir> <container>:/home/camera_workspace/rosbags
```

2. Play the bag
```bash
cd /home/camera_workspace/rosbags
ros2 bag play <bag-name> --loop
```

## Building and Launching the Tracker

1. Clone this repo
```bash
cd /home/camera_workspace/src
git clone https://github.com/cweihan01/zed-object-tracker.git
```

2. Clone the [ByteTrack repo](https://github.com/ifzhang/ByteTrack)
```bash
cd /home/camera_workspace/src
git clone https://github.com/ifzhang/ByteTrack.git
cd ByteTrack
pip3 install -r requirements.txt
python3 setup.py develop
pip3 install cython
pip3 install 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
pip3 install cython_bbox
```

Fixes to common issues:
- Update `ByteTrack/requirements.txt` with the following versions:
```bash
onnx==1.15.0
onnxruntime==1.12.0
```

- Update `ByteTrack/setup.py` with the following:
```python
# Comment out lines 50 and 51, and replace with the following line,
# or create an empty README.md in the root directory

# with open("README.md", "r") as f:
#     long_description = f.read()
long_description = ""
```

3. Build the workspace
```bash
cd ~/camera_workspace
colcon build --symlink-install
source install/setup.bash
```

4. Run the node
```bash
ros2 run zed-object-tracker detect
```
