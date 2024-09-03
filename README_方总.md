# mtuav-competition-2024
第二届低空经济智能飞行管理挑战赛 性能赛（BIT-LINC）

## 配置

## Env
- Docker必须为最新版本，否则网络连接部分命令不能执行
```shell
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
- Docker在登录时可能会有Proxy的问题，需要通过
```shell
sudo find /etc/systemd/system/docker.service.d/ -name '*proxy.conf'
```
找到`proxy.conf`并注释掉有问题的loopback proxy。
- 带有\的docker命令可能会出`invalid reference format`错误，注意换行。

## Ref
- 这里有去年的悲催经历：https://github.com/superboySB/mtuav-competition