#!/bin/sh

#判断.git目录是否存在
if [ ! -d ".git" ]; then
    git init #初始化
    git remote add origin  http://10.60.66.16:3000/bob.li/agv_controller.git # 增加远端的仓库地址
    git config core.sparsecheckout true # 设置Sparse Checkout 为true
fi

rm -rf .git/info/sparse-checkout

# 将要部分clone的目录相对根目录的路径写入配置文件
echo ".vscode/*" >> .git/info/sparse-checkout 
echo "bash/*" >> .git/info/sparse-checkout 
echo "agv_comm/*" >> .git/info/sparse-checkout 
echo "move_base/*" >> .git/info/sparse-checkout 
echo "doc/*" >> .git/info/sparse-checkout 
echo "cartographer/*" >> .git/info/sparse-checkout 
echo "cartographer_ros/*" >> .git/info/sparse-checkout 
echo "abseil-cpp/*" >> .git/info/sparse-checkout 
echo "geometry2-melodic-devel/*" >> .git/info/sparse-checkout 
echo "launch/*" >> .git/info/sparse-checkout 
echo "navigation-melodic-devel/*" >> .git/info/sparse-checkout 
echo "navigation_msgs-ros1/*" >> .git/info/sparse-checkout 
echo "robot_pose_ekf/*" >> .git/info/sparse-checkout 
echo "scout_ros/*" >> .git/info/sparse-checkout 
echo "sick_scan-master/*" >> .git/info/sparse-checkout 
echo "xsens_ros_mti_driver/*" >> .git/info/sparse-checkout 
echo "agv_chassis/*" >> .git/info/sparse-checkout 
echo "sick_safetyscanners-master/*" >> .git/info/sparse-checkout 

git checkout master #切换主分支，这样新增的目录就能更新下来
git pull origin master #pull下来代码
git branch --set-upstream-to=origin/master master #这样就可以直接执行git pull了
cp -r ./abseil-cpp ../
rm -rf ../abseil-cpp/CATKIN_IGNORE

