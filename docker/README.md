<!--
 * @Author: blank1 448913821@qq.com
 * @Date: 2026-01-05 16:37:34
 * @LastEditors: blank1 448913821@qq.com
 * @LastEditTime: 2026-02-06 16:10:00
 * @FilePath: \ddt_ros2_control\docker\README.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
## DDT_TiTa_ros2_docker
### Download the docker image
```bash
docker pull registry.cn-hangzhou.aliyuncs.com/ddt_robot/run_ddt_tita:v1.0
```

### start the container

```bash
export ddt_ros2_ws_path=path/above/your/project
sudo docker run -v "$ddt_ros2_ws_path":/mnt/dev -w /mnt/dev --rm  --gpus all --net=host --privileged -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1  -e CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda -it registry.cn-hangzhou.aliyuncs.com/ddt_robot/run_ddt_tita:v1.0
```
### Test

The docker have pre-compiled all the necessary reliance and can be used directly. Please refer to [ddt_ros2_control](https://github.com/DDTRobot/ddt_ros2_control) for further information.

The following tests should pass.

####  webots

```bash
webots
```
The webots GUI should show up.
##### If you meet "xcb" error :
Run
```
xhost +
```

#### ROS 2 :
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

### If you any package is missing (e.g. pinocchio), try :
```bash
apt update
apt install ros-humble-pinocchio
#Then delete the build file and build again
```

### convert the onnx to .engine
Run the command in the docker, rather than the host.
```
cd /mnt/dev
/usr/src/tensorrt/bin/trtexec --onnx=your_onnx.onnx --saveEngine=your_engine.engine
```
### Remember to change your engine file path under the path of docker, e.g. :
```
# /mnt/dev is the path in your docker, not int the host
cuda_test_ = std::make_shared<CudaTest>("/mnt/dev/model_gn.engine");
```
