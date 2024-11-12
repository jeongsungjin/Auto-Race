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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/foscar/Auto-Race-/src/fiducials/aruco_detect"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/foscar/Auto-Race-/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/foscar/Auto-Race-/install/lib/python3/dist-packages:/home/foscar/Auto-Race-/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/foscar/Auto-Race-/build" \
    "/usr/bin/python3" \
    "/home/foscar/Auto-Race-/src/fiducials/aruco_detect/setup.py" \
     \
    build --build-base "/home/foscar/Auto-Race-/build/fiducials/aruco_detect" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/foscar/Auto-Race-/install" --install-scripts="/home/foscar/Auto-Race-/install/bin"