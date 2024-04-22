docker start slam
docker exec slam service ssh start

reset
pushd ~/gamma/cartographer/
git add .
git commit -m "modify cartographer"
git push origin master -f
popd

ssh root@172.17.0.2 'killall roslaunch && killall slam_node && killall test_node'
ssh root@172.17.0.2 'cd /root/gamma/src/cartographer/ &&git reset --hard origin/master && git pull origin master && sync'
ssh root@172.17.0.2 'source /opt/ros/noetic/setup.bash && cd /root/gamma/ && catkin_make -DBUILD_SLAM=false -DBUILD_TEST=true -j16'
ssh root@172.17.0.2 'source /root/gamma/devel/setup.bash && roslaunch cartographer test.launch'