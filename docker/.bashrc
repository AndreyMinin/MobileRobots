
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
GLOBAL_ROS_SETUP="/opt/ros/$(rosversion -d)/setup.bash"
echo "Init ros install from $GLOBAL_ROS_SETUP"
source "$GLOBAL_ROS_SETUP"
LOCAL_ROS_SETUP="$HOME/local/workspace/mr_ws/devel/setup.bash"
if test -f "$LOCAL_ROS_SETUP"; then 
    echo "Init local ros workspace from $LOCAL_ROS_SETUP"
#    source "$LOCAL_ROS_SETUP"
else
    echo "Local ros workspace has not build."
    echo "Build rosworspace and source it" 
fi
cd $HOME/local/workspace/mr_ws



