xhost + local:root

XAUTH=/tmp/.docker.xauth
 if [ ! -f $XAUTH ]
 then
    sudo touch $XAUTH
     xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
     if [ ! -z "$xauth_list" ]
     then
        echo $xauth_list | sudo xauth -f $XAUTH nmerge -
     fi
     sudo chmod a+r $XAUTH
 fi