# PlatformIO pre-build script: keep extra_packages/pingpong_interfaces identical to the ROS2
# workspace copy (raspberry/src/pingpong_interfaces). If the messages changed, the micro-ROS library is
# deleted so PlatformIO rebuilds it (takes a few minutes).
import filecmp
import os
import shutil

Import("env")  # noqa: F821  (provided by PlatformIO)

project = env.subst("$PROJECT_DIR")  # noqa: F821
src = os.path.normpath(os.path.join(project, "..", "raspberry", "src", "pingpong_interfaces"))
dst = os.path.join(project, "extra_packages", "pingpong_interfaces")


def same_tree(a, b):
    if not os.path.isdir(b):
        return False
    cmp = filecmp.dircmp(a, b)
    if cmp.left_only or cmp.right_only or cmp.diff_files or cmp.funny_files:
        return False
    return all(same_tree(os.path.join(a, d), os.path.join(b, d)) for d in cmp.common_dirs)


if not same_tree(src, dst):
    print("sync_interfaces: messages changed, copying and forcing a micro-ROS rebuild")
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)
    libmicroros = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"),  # noqa: F821
                               "micro_ros_platformio", "libmicroros")
    shutil.rmtree(libmicroros, ignore_errors=True)
