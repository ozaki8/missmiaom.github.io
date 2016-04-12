---
layout:     post
title:      "ROS IDE之Eclipse的配置方法"
<<<<<<< HEAD
subtitle:   "\"ROS IDE设置\""
=======
subtitle:   "\"Eclipse的配置方法\""
>>>>>>> origin/master
date:       2016-04-12 12:00:00
author:     "Leiym"
header-img: "img/post-bg-2015.jpg"
tags:
    - ROS
---

## 前言

<<<<<<< HEAD
Hux 的 Blog 就这么开通了。

2015 年，Hux 总算有个地方可以好好写点东西了。

作为一个程序员， Blog 这种轮子要是挂在大众博客程序上就太没意思了。一是觉得大部分 Blog 服务都太丑，二是觉得不能随便定制不好玩。之前因为太懒没有折腾，结果就一直连个写 Blog 的地儿都没有。

在玩了一段时间知乎之后，答题的快感又激起了我开博客的冲动。之前的[个人网站](http://huangxuan.me/portfolio)是作品集形式的（现在集成进来了），并不适合用来写博文，一不做二不休，花一天搞一个吧！

=======
>>>>>>> origin/master
## ROS IDE之Eclipse的配置方法
***
> 这篇博文给大家介绍ROS的重要IDE——Eclipse的配置方法。

***
### 安装eclipse并配置启动文件

不能通过apt-get的方式安装，而需要在官网下载客户端，并做相关配置。下载地址：[eclipse官网下载](http://www.eclipse.org/downloads/packages/eclipse-ide-cc-developers/mars2)。（注意需要选择Eclipse IDE for C/C++ Developers 版本，并根据你的ubuntu系统选择32位或者64位）

下载完成后是没有产生启动图标的，在终端输入eclipse也会提示并没有安装。这时就需要创建一个启动图标，并设置参数。参考：[eclipse启动图标文件配置](http://www.blogs.digitalworlds.net/softwarenotes/?p=54)

完成上述教程后，应该就会生成eclipse图标，点击就可以进入，但这样无法eclipse装载当前环境变量，所以还需要更改一行配置。

- `$sudo gedit /usr/share/applications/eclipse.desktop`

将 *Exec=/opt/eclipse/eclipse* 这一行改为：

- `Exec=bash -i -c /opt/eclipse/eclipse`


### 创建eclipse工程

执行以下命令，记得将*catkin_ws*替换成你想要导入eclipse的工程所在的工作空间名：
1. `$ . ~/catkin_ws/devel/setup.bash`
- `$ cd ~/catkin_ws`
- `$ catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_ECLIPSE_MAKE_ARGUMENTS=-j8 .`

### 将工程导入eclipse

打开eclipse，在导入之前更改一下头文件和内存限制，以免导致像<ros.h>等头文件无法包含。

### 调试运行

### DIY eclipse代码格式以及配色
