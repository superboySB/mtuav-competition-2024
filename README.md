# mtuav-competition-2024
第二届低空经济智能飞行管理挑战赛 性能赛（BIT-LINC）

## 环境配置
需要一台比较好的本地电脑,计算时间的同时，最好也要加上小车和飞机的状态来判断

### docker
与去年一样，需要有docker支持，这里会自动拉取镜像
```sh
cd scripts
chmod +x race_images.sh
./race_images.sh
```
创建自定义局域网，并查看子网络详细信息
```sh
docker network create --subnet=192.168.100.0/24 race_net
docker network ls
docker network inspect race_net
```
### 启动sdk（重新启动sdk）
启动SDK服务，遇到权限问题可以`sudo mkdir -p /home/race_log`，然后建立局域网互联的container
```sh
cd scripts
chmod +x start_race.sh
./start_race.sh
```
这样会启动三个官方无人机、无人车、平台准备的container（建议初始化），然后再启动用户container（可重复利用）
```sh
docker run -d -p 8888:8888 --name race_user_sdk_container \
	--network race_net --ip 192.168.100.4 \
	-e ROS_MASTER_URI=http://192.168.100.4:11311 \
	-e ROS_IP=192.168.100.4 \
	-v /home/race_log/user_log:/home/race_log \
	-v /etc/localtime:/etc/localtime:ro \
	-v /etc/timezone:/etc/timezone:ro \
	uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:user
```
相当于四个Docker容器。根据比赛的现状，比赛现在决定公开线上比赛6个卸货点相应地[config文件](docs/config_online)，解压之后是各docker线上配置文件，大家有需要可以参考[教程](docs/线下SDK中修改小车飞机数量和订单位置的办法.pdf)，自行下载并在本地替换。
```sh
docker cp docs/config_online/car/config.json race_car_sdk_container:/car_log/config.json
docker cp docs/config_online/drone/drone.json race_drone_sdk_container:/config/drone.json
docker cp docs/config_online/scene/scene.config race_scene_sdk_container:/evaluator/config/scene.config
docker cp docs/config_online/user/config.json race_user_sdk_container:/config/config.json
```
重启镜像
```sh
cd scripts
chmod +x restart_race.sh
./restart_race.sh
```

### 关闭sdk（重启前释放资源）
如果需要关闭SDK服务，则运行下列内容关闭并清除三个容器，后续重新新建进行初始化
```sh
cd scripts
chmod +x stop_race.sh
./stop_race.sh
```

## 单机版本的代码开发
我们进入user容器开搞
```sh
docker exec -it race_user_sdk_container bash
```
确保`rosnode list`包含`/competition_msg_handler_node`和`/map_client_node`。然后建议后续直接用vscode插件在容器里面开发吧。

【可选】对于可视化平台，如果目前手头的服务器不支持公开ip的话，可能需要本机转发一下，例如：
```sh
ssh -L 8888:localhost:8888 -p 17003 ps@36.189.234.178
```
然后可视化的时候注意要导入`/home/sdk_for_user/map_utmm/`里面的本地地图才行。

### 容器内开发过程
在`race_user_sdk_container`容器内的`/root/`目录拉自己的最新代码
```sh
cd ~
git clone https://github.com/superboySB/mtuav-competition-2024
cd mtuav-competition-2024
```
下面是一个基于python的简单demo，不包含复杂的算法设计和效率提升，是对竞赛SDK的简易流程示教，demo可以一直送完配置文件中的订单量。
```sh
# 编译
catkin_make

# 激活环境
source devel/setup.bash

# 运行
rosrun race_demo demo.py
```
在解决了转发问题以后，可以在`http://<server-ip>:8888/`里面看到可视化与console基本是较低延迟同步执行的，其中`server-ip`就是你可以内网直连的电脑ip（如`172.16.0.108`），如果使用terminial转发的话就应该是`localhost`。注意注意！！如果是为了提交代码，我们只做好编译就可以到下一步了。

## 本地测试run.sh && 在线提交docker镜像
首先，确保`race_user_sdk_container`的`/home/`目录里面有比赛代码，并且比赛程序的启动方式已经写入到了`run.sh`文件中，自己测试工程worksapce下面的`/home/mtuav-competition-2024/run.sh`是否可以跑通,如果可以的话就直接替换掉比赛要求位置的同名文件
```sh
cp ~/mtuav-competition-2024/run.sh /home/run.sh
bash /home/run.sh
```
运行完如果发现也没问题，那别人启动docker就会自动执行比赛程序。接下来执行以下指令
```sh
docker commit race_user_sdk_container race_user:{tag}
```
其中tag可以自己定义为版本号之类的玩意。然后登录腾讯云docker服务
```sh
docker login uav-challenge.tencentcloudcr.com --username 'tcr$user' --password gXWWpxhO9igRnXzYYV58UexxS1Gw8VQY
```
根据比赛方给的appkey和secretKey，我们内部使用
```sh
docker tag race_user:{tag} uav-challenge.tencentcloudcr.com/uav_challenge_2024/3b0859ed3c9d2fd4d7f2a618b85ca413:{tag}

docker push uav-challenge.tencentcloudcr.com/uav_challenge_2024/3b0859ed3c9d2fd4d7f2a618b85ca413:{tag}
```
其中tag可以自己定义。然后，在本地（非container内部）运行提交官方给的脚本
```sh
cd docker_submit_tool

./submit.sh submit uav-challenge.tencentcloudcr.com/uav_challenge_2024/3b0859ed3c9d2fd4d7f2a618b85ca413:{tag}
```
如果没什么报错应该就认为是成功提交了一次镜像。然后可以查看自己在线提交的模型的动态与成绩
```sh
./submit.sh query
```

## 初赛笔记
记录一些比较关键的信息，莫要做的太复杂：
- [ ] 目前车机协同作业区没有障碍物

- [ ] 初赛中，下单时间、订单是否超时都不考虑


- [ ] 换电和上货的time_est不是手动设置的，飞机会自己完成，状态会变成上货中、换电中。指令需要你发，仿真机仿真这个过程。无人机在小车上才能完成换电或者上货的操作，充电操作**不需要**无人机和小车解绑。

- [ ] 正式比赛会给6个小车、30个无人机，所以现在这个settings可能需要自己结合[文档](docs/线下SDK中修改小车飞机数量和订单位置的办法.pdf)修改一下来调试，这个比赛从来不能在测试集训练的

- [ ] 正式比赛中的config文件不会提供用于调试（地图可能也会换？）

- [ ] 小车要在航空作业区才能接飞机降落，在图中绿色区域内，任何一点都可以进行航空作业。需要注意的是，小车的位置不要太靠近边界，因为有可能会被误判出界，一般保持0.5米左右的距离就行

- [ ] 目前还没有像去年那样加入悬停功能，到达route的最后一个点（end_pos）以后会立马发送一个landing type的航点（其实去年大家基本也没用上这个功能，直接全局先做map就好了，中间冲突等待时间都可以在地面预测），所以一定要保证最后一个点是卸货点。

- [ ] 比赛的时候卸货点有6个，不会提前告知位置，只能从订单里提取他们的位置。

- [ ] 目前，飞机收到起飞指令会自动起飞到距离起飞点高3m的位置，然后自动加速到选手设定的速度去往下一个点，并且在距离飞行下一个点较近距离时，飞机会线性减速，来平稳到达选手所设定的目标点。在飞机飞完所有点，收到降落指令后，会自动降落。但是不能直接起点小车、终点订单，还是需要指定中间waypoint来避障，这和去年基本是一样的。

- [ ] 无人车的安全距离3米，无人机的安全距离5米。注意：降落到距离很近的位置也会判定为相撞，也就是降落的时候降落点不能有其他无人机。

- [ ] 地面坐标系z轴向下，读取z轴范围是-222到0，越往上值越小，可以认为乘上-1就是海拔。0不一定是地面，可能是地面以下，还是要根据体素的distance判断。

- [ ] 体素的分辨率是2米，障碍物是有一个个2米的立方体组成的，这就是在规划位置的时候最好不要用整偶数，因为浮点数精度问题，可能会遇到比较诡异的bug（和去年一致）

- [ ] 飞机起飞时，需要在距离起飞点（x，y）半径10米范围内，完成起飞，飞到60米以上，120以下。飞机飞行中，保持在[60,120]米之间。我的起飞点和第一个途经点可以不是垂直的，只要起飞点和第一个途经点的xy坐标在一个半径为10的圆内，但实际上还是先垂直起飞、然后到玩家的第一个途径点。（所以第一个点最好还是垂直上去的呀）

- [ ] 系统是异步执行的，py版本的demo输出，你可以看一下具体代码，它打印的是下一个即将执行的状态。比如说现在发送了飞机航线指令，那么当前状态会立马变为下一个release cargo，所以你会看到飞机在执行航线，但是打印输出对不上，知道了这个逻辑，你应该就可以理解了。建议自己写一个输出打印，订阅一下/panoramic_info话题，打印出你关心的飞机 小车 订单事实状态，可以参考C++版demo

- [ ] 目前为止，如果按照视频教程，提交的是未修改过的demo版本，那么得分为0是合理的。demo里的航线高度为-145m，未通过航线校验，导致小车载着飞机到航空作业区后，飞机无法起飞送单，最终得分为0。我们检查的目的是为了查看大家的0分是因为没成功执行程序，还是说 成功发送了 上货 小车移动这些指令，只是飞机无法执行航线。如果是后者情况，说明大家提交的镜像是没有问题的，后续可提交自己修改过的demo程序，查看结果。

- [ ] 考虑到初赛难度的问题，我们现在暂时不放开轨迹跟踪的方法。同时考虑到您提到的问题，我们会在决赛阶段放开，提升一些难度，选手到时候，可以使用轨迹跟踪

- [ ] 线下和线上查询体素的方式完全一致（for_py/ros service）。bin文件内容不会变，**所谓地图发生变化，只是飞机 小车，还有送餐点等会变化**，这些通过订阅全景信息可以拿到

- [ ] 订单在一开始会全部给到，线上是500单，想送哪个送哪个。最好用心跳中的订单信息，因为订单的状态会发生变化（user_pkg::PanoramicInfo），比如配送中、配送完成等这些状态变化。

- [ ] 小车只能在航空作业区接收无人机，接收完之后可以在商家机场内的任何位置停留（包括航空作业区或者地勤作业区），只是别产生碰撞就行

- [ ] 如果一号飞机从一号小车起飞回来用二号小车接，**不需要在起飞时unbind**一号飞机和一号车吗，因为起飞的时候会自动解绑（就不再有联系了），不需要手动处理，看一下用几号小车接收就行了。

- [X] 我们发现同学们提交的任务跑出来的结果，大部分都是0分，原因可能是航线超高了。一开始为了降低大家调试的难度，我们给大家提供的sdk航线高度限制是150米，Python的demo航线高度是145米，但是线上的飞机航线限高120米，所以大家直接提交demo，跑出来的分数为0分。同学们要自己规划，降低航线高度，线上的卸货点和SDK中是不一致的，需要大家在代码中动态规划航线。降低航线高度后，注意避开地图障碍物。

- [ ] 如果第一个点的类型是takeoff，飞机就会垂直起飞离地3m（takeoff和land类型的时候，设置的点是没有用的）

- [ ] 在飞机上货和换电的时候，不要操作小车和飞机, 上货 换电都需要10s 可以通过飞机状态来判断

- [ ] 裁判系统对小车停靠位置的判定条件更加宽松，只有小车位置在目标点0.5m以内，裁判系统就会判定小车停靠到位了，所以通常不需要担心小车运动误差导致停不到目标位置，收无人机的判断也是0.5米

- [ ] 飞机降落以后从flying状态 变为ready状态 才能抛餐  此时是在地面上  不会掉电  只有flying状态 才会掉电

- [ ] 对于小车yaw角，选手指定的情况下用选手设置的值，选手不设置或设置为0则yaw角为朝着路线前进的方向

## Ref
- 这里有去年的悲催经历：https://github.com/superboySB/mtuav-competition
