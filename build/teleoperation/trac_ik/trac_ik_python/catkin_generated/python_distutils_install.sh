#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/master/franka_final_ws/src/teleoperation/trac_ik/trac_ik_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/master/franka_final_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/master/franka_final_ws/install/lib/python2.7/dist-packages:/home/master/franka_final_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/master/franka_final_ws/build" \
    "/usr/bin/python2" \
    "/home/master/franka_final_ws/src/teleoperation/trac_ik/trac_ik_python/setup.py" \
    build --build-base "/home/master/franka_final_ws/build/teleoperation/trac_ik/trac_ik_python" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/master/franka_final_ws/install" --install-scripts="/home/master/franka_final_ws/install/bin"
