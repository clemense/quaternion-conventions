# Quaternion Order Conventions
Quaternions are expressions of the form `w + x i + y j + z k`, with `x i + y j + z k` being the vector/imaginary part and `w` the scalar/real part. Math libraries differ in the way they order the vector and real part of a quaternion. This can lead to bothersome debugging sessions. 

The following table shows quaternion implementations and their chosen order:<br>
X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png) (scalar-last format) &nbsp; vs. &nbsp; W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)

| Library / Framework |  Quaternion Format |
| --- | --- |
| [autolab_core](https://berkeleyautomation.github.io/autolab_core/api/dual_quaternion.html#autolab_core.DualQuaternion.qr) |  W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Blender.mathutils](https://docs.blender.org/api/blender_python_api_current/mathutils.html?highlight=vector#mathutils.Quaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Boost](https://www.boost.org/doc/libs/1_71_0/libs/math/example/HSO3.hpp) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Bullet](https://pybullet.org/Bullet/BulletFull/classbtQuaternion.html) / [PyBullet](http://goo.gl/QwJnFX) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [DART](https://github.com/dartsim/dart/blob/5058255853d5b733476fc031b18fb95bdf7d7f5d/python/dartpy/eigen_geometry_pybind.cpp#L225) (uses Eigen::Quaternion)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Drake](https://drake.mit.edu/pydrake/pydrake.common.eigen_geometry.html?highlight=quaternion#pydrake.common.eigen_geometry.Quaternion_[float]) (uses Eigen::Quaternion)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [differentiable-robot-model](https://github.com/facebookresearch/differentiable-robot-model/blob/7e58c1f286a57d48deaafc78bda0e3dedb8e5c4a/differentiable_robot_model/se3_so3_util.py) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [dm_robotics.transformations](https://github.com/deepmind/dm_robotics/tree/main/py/transformations)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Eigen - API](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Eigen - Internal memory ordering](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a3eba7a582f77a8f30525614821d7056f)| X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [FleX](https://developer.nvidia.com/flex) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [iDynTree](https://robotology.github.io/idyntree/classiDynTree_1_1Rotation.html#adcac444f00ca751417f3095b401de86c) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png) | 
| [Isaac Gym](https://developer.nvidia.com/isaac-gym) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [lietorch](https://github.com/princeton-vl/lietorch) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [Klampt](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/_modules/klampt/math/so3.html#quaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [MATLAB](https://www.mathworks.com/help/robotics/ref/quaternion.html) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [MuJoCo](http://mujoco.org/book/modeling.html#COrientation) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Nuklei](http://nuklei.sourceforge.net/doxygen/) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [numpy-quaternions](https://github.com/moble/quaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [OpenRAVE 0.9.0](http://openrave.org/docs/latest_stable/coreapihtml/geometry_8h_source.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [Orocos KDL](http://docs.ros.org/jade/api/orocos_kdl/html/classKDL_1_1Rotation.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [PhysX](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/apireference/files/classPxQuat.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [PyMesh](https://pymesh.readthedocs.io/en/latest/api_misc.html#quaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [pyquaternion](https://github.com/KieranWynn/pyquaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [pyrr](https://pyrr.readthedocs.io/en/latest/_modules/pyrr/quaternion.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [pytorch3d](https://pytorch3d.readthedocs.io/en/latest/_modules/pytorch3d/transforms/rotation_conversions.html#standardize_quaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [pytransform3d](https://rock-learning.github.io/pytransform3d/_apidoc/pytransform3d.rotations.check_quaternion.html#pytransform3d.rotations.check_quaternion) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [quaternions](https://github.com/mjsobrep/quaternions/blob/master/quaternions/quaternion.py) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [RoMa](https://naver.github.io/roma/#main-features) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [ROS](https://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [ROS2 (tf2)](https://github.com/ros2/geometry2/blob/rolling/tf2/include/tf2/LinearMath/QuadWord.h) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [scipy.spatial.transforms](https://docs.scipy.org/doc/scipy/reference/spatial.transform.html#) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [spatialmath-python](https://petercorke.github.io/spatialmath-python/func_quat.html) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [squaternion](https://github.com/MomsFriendlyRobotCompany/squaternion/blob/master/squaternion/squaternion.py) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [The Library for Uniform Deterministic Sequences and Sets of Samples over 2-sphere and SO(3)](http://lavalle.pl/software/so3/so3.html)| X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [Theseus](https://github.com/facebookresearch/theseus/blob/e6dd7937edee59daf5f5c2ffe870f39fa1171e90/theseus/embodied/kinematics/kinematics_model.py#L44)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [transformations.py (<=2009.04.18)](http://docs.ros.org/jade/api/tf/html/python/transformations.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [transformations.py (>=2010.05.10](https://www.lfd.uci.edu/~gohlke/code/transformations.py.html), includes [trimesh.transformations)](https://github.com/mikedh/trimesh/blob/master/trimesh/transformations.py) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [Transforms3d](https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [tinyquaternion](https://github.com/rezaahmadzadeh/tinyquaternion/blob/master/tinyquaternion/tinyQuaternion.py)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [USD (Pixar) - API](https://graphics.pixar.com/usd/release/api/class_gf_quatf.html#a781cffeee14aa3ba3f89de7d6df5a035)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [USD (Pixar) - Internal memory ordering](https://graphics.pixar.com/usd/release/api/quatf_8h_source.html)| X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [ViSII - A VIrtual Scene Imaging Interface](https://owl-project.github.io/ViSII/all.html#visii.quat)| W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png)|
| [NVIDIA Warp](https://nvidia.github.io/warp/_build/html/modules/functions.html?highlight=transform#quaternion-math) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png)|
| [YARP - API](https://www.yarp.it/latest/classyarp_1_1math_1_1Quaternion.html) | X-Y-Z-W ![#ffa500](https://via.placeholder.com/15/ffa500/ffa500.png) |
| [YARP - Internal memory ordering](https://github.com/robotology/yarp/blob/v3.8.0/src/libYARP_math/src/yarp/math/Quaternion.h#L23) | W-X-Y-Z ![#89cff0](https://via.placeholder.com/15/89cff0/89cff0.png) | 
