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

echo_and_run cd "/Users/marcoreis/b166er/src/rosserial/rosserial_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/Users/marcoreis/b166er/install/lib/python3.11/site-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/Users/marcoreis/b166er/install/lib/python3.11/site-packages:/Users/marcoreis/b166er/build/rosserial_python/lib/python3.11/site-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/Users/marcoreis/b166er/build/rosserial_python" \
    "/Users/marcoreis/miniforge3/envs/ros_env11/bin/python3.11" \
    "/Users/marcoreis/b166er/src/rosserial/rosserial_python/setup.py" \
     \
    build --build-base "/Users/marcoreis/b166er/build/rosserial_python" \
    install \
    --root="${DESTDIR-/}" \
     --prefix="/Users/marcoreis/b166er/install" --install-scripts="/Users/marcoreis/b166er/install/bin"
