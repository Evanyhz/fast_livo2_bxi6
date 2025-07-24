# 1.部署fast_livo2：
```bash
#在根目录下新建终端，然后运行下列命令：
  mkdir -p ~/fast_livo2_bxi6    #新建文件夹
  cd fast_livo2_bxi6
  git clone https://github.com/Evanyhz/fast_livo2_bxi6
  mv fast_livo2_bxi6 src       #把克隆过来的的文件夹重命名为src
  #然后按 readme3.md 部署, 只需要关注未注释的部分，即2.3节/2.5节/3.0节
```
  
# 2.部署elevation_mapping：
```bash
#新建终端，然后运行下列命令：
  mkdir -p ~/elevation_mapping_bxi6
  cd elevation_mapping_bxi6
  git clone https://github.com/Evanyhz/elevation_mapping_bxi6
  mv elevation_mapping_bxi6 src      
  #然后按照对应的 readme 配置
```

# 3.配置IP:
```bash
首先将雷达的网线连接到电脑：（其中雷达的IP：192.168.2.202，这里配置IP是确保他们处于同一网段）
打开“设置”->选择“网络”->打开有线设置->点击“IPv4”->选择“手动”->地址填“192.168.2.50”，子网掩码“255.255.255.0”->点击“应用”
此时应该显示有线已连接，成功连接雷达
```

# 4.一键启动:
```bash
# 需要提前安装终端复用器：Tmux
  sudo apt update && sudo apt install tmux -y

#进入fast_livo2_bxi6工作空间：
  source /opt/ros/humble/setup.bash && source install/setup.bash 
  cd src/
  ./one_click_launch.sh

#一键关闭
  ./one_click_launch.sh stop

#单独启动：
fast-livo2的启动命令详见instruction.md ；  elevation_mapping的启动命令详见其功能包下的readme.md
```
  
