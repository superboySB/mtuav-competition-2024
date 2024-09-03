# mtuav-competition-2024
第二届低空经济智能飞行管理挑战赛 性能赛（BIT-LINC）

## 环境配置
我们默认手头是一个windows笔记本电脑，可以连接一些服务器来做这个比赛。（mac和linux图形界面本机可以自己研究，此外从配置上看好像没有允许使用cuda-enabled的算法实现？）

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
docker run -itd -p 8888:8888 --name race_user_sdk_container \
	--network race_net --ip 192.168.100.4 \
	-e ROS_MASTER_URI=http://192.168.100.4:11311 \
	-e ROS_IP=192.168.100.4 \
	-v /home/race_log/user_log:/home/race_log \
	-v /etc/localtime:/etc/localtime:ro \
	-v /etc/timezone:/etc/timezone:ro \
	uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:user
```
相当于四个Docker容器。

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

【可选】对于可视化平台，如果目前手头的服务器不支持公开ip的话，可能需要本机转发一下
```sh
ssh -L 8888:localhost:8888 -p 17003 ps@36.189.234.178
```
然后注意要导入`/home/sdk_for_user/map_utmm/`里面的本地地图才行。

### 容器内开发过程
在`race_user_sdk_container`容器内的`/home/`目录拉自己的最新代码
```sh
cd /home/
git clone https://github.com/superboySB/mtuav-competition-2024
cd mtuav-competition-2024
```
下面是一个基于python的简单demo，不包含复杂的算法设计和效率提升，是对竞赛SDK的简易流程示教，demo可以一直送完配置文件中的订单量。
```sh
# 编译
catkin_make
source devel/setup.bash

# 运行
rosrun race_demo demo.py
```
在解决了转发问题以后，可以本机在`http://localhost:8888/`里面看到可视化与console基本是较低延迟同步执行的。

## 在线提交镜像
首先，确保`race_user_sdk_container`的`/home/`目录里面有比赛代码，并且比赛程序的启动方式已经写入到了`run.sh`文件中，自己测试工程worksapce下面的`/home/mtuav-competition-2024/run.sh`是否可以跑通,如果可以的话就直接替换掉比赛要求位置的同名文件
```sh
cp /home/mtuav-competition-2024/run.sh /home/run.sh
bash /home/run.sh
```
运行完如果发现也没问题，那别人启动docker就会自动执行比赛程序。接下来执行以下指令
```sh
docker commit race_user_sdk_container race_user:linc-xx
```
其中xx可以自己定义为版本号之类的玩意。然后登录腾讯云docker服务
```sh
docker login uav-challenge.tencentcloudcr.com --username 'tcr$user' --password gXWWpxhO9igRnXzYYV58UexxS1Gw8VQY
```
<!-- 然后提交镜像到docker hub
```sh
docker tag race_user:linc-xx uav-challenge.tencentcloudcr.com/uav_challenge_2024/{appkey}:{tag}
docker push uav-challenge.tencentcloudcr.com/uav_challenge_2024/appkey:tag
``` -->
根据比赛方给的appkey，我们内部使用
```sh
docker tag race_user:linc-xx uav-challenge.tencentcloudcr.com/uav_challenge_2024/3b0859ed3c9d2fd4d7f2a618b85ca413:{tag}
```
<!-- ```sh
docker push uav-challenge.tencentcloudcr.com/uav_challenge_2024/3b0859ed3c9d2fd4d7f2a618b85ca413:{tag}
``` -->
其中tag可以自己定义。然后，在本地运行提交脚本
```sh
cd scripts

./submit.sh submit uav-challenge.tencentcloudcr.com/uav_challenge_2024/3b0859ed3c9d2fd4d7f2a618b85ca413:{tag}
```


## Ref
- 这里有去年的悲催经历：https://github.com/superboySB/mtuav-competition