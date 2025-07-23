# 1.部署fast_livo2：
```bash
  mkdir -p ~/fast_livo2_bxi6    #新建文件夹
  cd fast_livo2_bxi6
  git clone https://github.com/Evanyhz/fast_livo2_bxi6
  mv fast_livo2_bxi6 src       #把克隆过来的的文件夹重命名为src
  #然后按照clone文件夹的 readme3.md 进行部署,只需要关注未注释的部分，2.3节和2.5节
```
  
# 2.部署elevation_mapping：
```bash
  mkdir -p ~/elevation_mapping_bxi6
  cd elevation_mapping_bxi6
  git clone https://github.com/Evanyhz/elevation_mapping_bxi6
  mv elevation_mapping_bxi6 src      
  #然后按照clone文件夹的 readme 进行配置
```

# 3.配置IP:
```bash
首先将雷达的网线连接到电脑：
打开“设置”->选择“网络”->打开有线设置->点击“IPv4”->选择“手动”->地址填“192.168.2.50”，子网掩码“255.255.255.0”->点击“应用”
此时应该显示有线已连接，成功连接雷达
```

# 4.一键启动:
```bash
source install/setup.bash 
cd ~/fast_livo2_bxi6/src
./one_click_launch.sh

#一键关闭
./one_click_launch.sh stop
```
  
