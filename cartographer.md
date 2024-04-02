cartographer系统集成

# 背景介绍

# 平台搭建

## 平台目标

​	目标硬件平台是rk3399和rk3588，目标的软件平台为ubuntu20.04。为了方便今后的交叉编译，使用docker作为虚拟机，使用ubuntu 20.04作为系统，使用linux/arm64/v8平台。同时安装本地Gitlab，进行数据保存与GitHab服务器进行同步。每次工作均保存成一个commit进行上传。

## docker配置

### docker安装

在ubuntu20.04安装docker

```
sudo apt-get update 
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
service docker start
```

### 容器配置

选择20.04的镜像，arm64v8的平台。

```
docker run -itd --name slam -p 11311:11311 --platform linux/arm64/v8 ubuntu:20.04 /bin/bash
```

使用命令后，会自下载docker文件，如下显示后代表docker成功安装。

```
$ docker run -itd --name slam --platform linux/arm64/v8 ubuntu:20.04 /bin/bash
Unable to find image 'ubuntu:20.04' locally
20.04: Pulling from library/ubuntu
6aae4cfdd5a1: Pull complete 
Digest: sha256:80ef4a44043dec4490506e6cc4289eeda2d106a70148b74b5ae91ee670e9c35d
Status: Downloaded newer image for ubuntu:20.04
78052cf4cba4fc653f3cbf5c088985cb16f03853b385d9f25b07ac27755bd18c
```

通过attach命令进入容器。

```
$ sudo docker attach slam
root@78052cf4cba4:/# 
```

查看服务器架构，确认是arm架构

```
root@78052cf4cba4:/# uname -m
aarch64
```

### 配置修改

```mermaid
graph LR
1(宿主ubuntu20.04:11311)
2(容器ubuntu20.04:11311)
1-->2
```

​	由于我们需要在宿主使用ros1工作容器进行调试，我们需要在生成容器以后调整端口。虽然可以在生成容器时绑定，但是最好掌握事后绑定的方法。

​	首先查询到名为slam的容器ID:78052cf4cba4

```
$ docker ps -a
CONTAINER ID   IMAGE        COMMAND     PORTS     NAMES
78052cf4cba4   ubuntu:20.04 "/bin/bash"           slam
```

​	其次，关闭容器后，编辑/var/lib/docker/containers/78052cf4cba4*/hostconfig.json文件，由于需要使用root权限需要特别注意。

```
$ docker stop slam
$ systemctl stop docker.socket
$ sudo vi /var/lib/docker
```

选择/containers/78052cf4cba4*/hostconfig.json进行修改

将hostconfig.json文件的"PortBindings",改为：

```
"PortBindings":{
	"11311/tcp":[{"HostIp":"","HostPort":"11311"}],
	"80/tcp":[{"HostIp":"","HostPort":"8080"}],
	"22/tcp":[{"HostIp":"","HostPort":"8022"}],
	"443/tcp":[{"HostIp":"","HostPort":"8443"}],
},
```

选择/containers/78052cf4cba4*/config.v2.json进行修改

将config.v2.json文件的"ExposedPorts",改为：

```
"ExposedPorts":{
	"11311/tcp":{},"22/tcp":{},"443/tcp":{},"80/tcp":{}
	},
```

### 基础包安装

从宿主使用attach进入docker

```
$ docker start slam
slam
$ sudo docker attach slam
root@44cf1a96d70f:/# 
```

```
#sudo apt-get update
#apt-get install wget
#sudo wget http://fishros.com/install -O fishros && . fishros
```

选择换源并安装ros1

```
RUN Choose Task:[请输入括号内的数字]
---众多工具，等君来用---
ROS相关:
  [1]:一键安装(推荐):ROS(支持ROS/ROS2,树莓派Jetson)
  [3]:一键安装:rosdep(小鱼的rosdepc,又快又好用)
  [4]:一键配置:ROS环境(快速更新ROS环境设置,自动生成环境选择)
  [9]:一键安装:Cartographer(18 20测试通过,16未测. updateTime 20240125)
  [11]:一键安装:ROS Docker版(支持所有版本ROS/ROS2)
  [16]:一键安装：系统自带ROS (！！警告！！仅供特殊情况下使用)
常用软件:
  [2]:一键安装:github桌面版(小鱼常用的github客户端)
  [6]:一键安装:NodeJS环境
  [7]:一键安装:VsCode开发工具
  [8]:一键安装:Docker
  [10]:一键安装:微信(可以在Linux上使用的微信)
  [12]:一键安装:PlateformIO MicroROS开发环境(支持Fishbot)
  [14]:一键安装:科学上网代理工具
  [15]:一键安装：QQ for Linux
配置工具:
  [5]:一键配置:系统源(更换系统源,支持全版本Ubuntu系统)
  [13]:一键配置:python国内源
[0]:quit
请输入[]内的数字以选择:1

RUN Choose Task:[请输入括号内的数字]
新手或首次安装一定要一定要一定要换源并清理三方源，换源!!!系统默认国外源容易失败!!
[1]:更换系统源再继续安装
[2]:不更换继续安装
[0]:quit
请输入[]内的数字以选择:1

RUN Choose Task:[请输入括号内的数字]
请选择换源方式,如果不知道选什么请选2
[1]:仅更换系统源
[2]:更换系统源并清理第三方源
[0]:quit
请输入[]内的数字以选择:2

RUN Choose Task:[请输入括号内的数字]
请选择你要安装的ROS版本名称(请注意ROS1和ROS2区别):
[1]:noetic(ROS1)
[2]:foxy(ROS2)
[3]:galactic(ROS2)
[4]:rolling(ROS2)
[0]:quit
请输入[]内的数字以选择:1

RUN Choose Task:[请输入括号内的数字]
请选择安装的具体版本(如果不知道怎么选,请选1桌面版):
[1]:noetic(ROS1)桌面版
[2]:noetic(ROS1)基础版(小)
[0]:quit
请输入[]内的数字以选择:2

Please select the geographic area in which you live. Subsequent configuration questions will narrow this down by presenting a list of cities, representing the time zones in which they are located.
6. Asia 
Geographic area: 6

Please select the city or region corresponding to your time zone.
70. Shanghai 
Time zone: 70
```

使用roscore命令判断是否装好ros

```
root@44cf1a96d70f:/# roscore
.. logging to /root/.ros/log/3d872440-f0cd-11ee-bbc2-8df8ab58e450/roslaunch-44cf1a96d70f-21762.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://44cf1a96d70f:40561/
ros_comm version 1.16.0
```

### cartographer安装

官网链接

https://google-cartographer.readthedocs.io/en/latest/

```
#cd ~
#apt-get install git
#git clone https://github.com/cartographer-project/cartographer.git
```

下载后上传到我们申请的github空间

```
root@44cf1a96d70f:~/cartographer# git remote -v
origin	https://github.com/cartographer-project/cartographer.git (fetch)
origin	https://github.com/cartographer-project/cartographer.git (push)
root@44cf1a96d70f:~/cartographer# git remote set-url origin git@github.com:herochiyou/cartographer.git
root@44cf1a96d70f:~/cartographer# git branch -M main
root@44cf1a96d70f:~/cartographer# git push -u origin main
The authenticity of host 'github.com (20.205.243.166)' can't be established.
ECDSA key fingerprint is SHA256:p2QAMXNIC1TJYWeIOttrVc98/R1BUFWu3/LiyKgUfQM.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
Warning: Permanently added 'github.com,20.205.243.166' (ECDSA) to the list of known hosts.
git@github.com: Permission denied (publickey).
fatal: Could not read from remote repository.

Please make sure you have the correct access rights
and the repository exists.
```

这是时候出现没有权限,修改权限后继续上传

```
root@44cf1a96d70f:~/cartographer# git push -u origin main
Enumerating objects: 14728, done.
Counting objects: 100% (14728/14728), done.
Delta compression using up to 20 threads
Compressing objects: 100% (2930/2930), done.
Writing objects: 100% (14728/14728), 5.80 MiB | 1.35 MiB/s, done.
Total 14728 (delta 11731), reused 14728 (delta 11731)
remote: Resolving deltas: 100% (11731/11731), done.
remote: This repository moved. Please use the new location:
remote:   git@github.com:slam-xiay/cartographer.git
To github.com:herochiyou/cartographer.git
 * [new branch]      main -> main
Branch 'main' set up to track remote branch 'main' from 'origin'.
```

### 安装ssh

安装配置ssh

```
$sudo apt-get install openssh-server openssh-client
$sed -i "s/#PubkeyAuthentication/PubkeyAuthentication/g" /etc/ssh/sshd_config
$sed -i "s/#uthorizedKeysFile/uthorizedKeysFile/g" /etc/ssh/sshd_config
$mkdir -p /root/.ssh/
$ssh-keygen
$service ssh restart
```

在宿主生成公钥

```
$ssh-keygen
```

在宿主打印公钥

```
$cat ~/.ssh/id_rsa.pub
```

复制宿主的公钥到容器的/root/.ssh/authorized_keys

通过ssh免秘钥登录主机

```
ssh root@172.17.0.2
```
