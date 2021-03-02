xhost + local:root

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(sudo xauth nlist :0 | sed -e 's/^..../ffff/')
    # if [ ! -z "$xauth_list" ]
    # then
    #     echo $xauth_list | sudo xauth -f $XAUTH nmerge -
    # else
    #     sudo touch $XAUTH
    # fi
    sudo touch $XAUTH
    sudo chmod a+r $XAUTH
fi