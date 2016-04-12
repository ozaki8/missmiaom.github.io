---
layout:     post
title:      "ROS IDE之Eclipse的配置方法"
subtitle:   "\"Eclipse的配置方法\""
date:       2016-04-12 12:00:00
author:     "Leiym"
header-img: "img/post-bg-2015.jpg"
tags:
    - ROS
---


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
