# Quaternion Order Conventions
Quaternions are expressions of the form `w + x i + y j + z k`, with `x i + y j + z k` being the vector/imaginary part and `w` the scalar/real part. Math libraries differ in the way they order the vector and real part of a quaternion. This can lead to bothersome debugging sessions. 

The following table shows quaternion implementations and their chosen order:<br>
X-Y-Z-W 🟥 (scalar-last format) &nbsp; vs. &nbsp; W-X-Y-Z 🟦

| Library / Framework |  Quaternion Format |
| --- | --- |
| [autolab_core](https://berkeleyautomation.github.io/autolab_core/api/dual_quaternion.html#autolab_core.DualQuaternion.qr) |  W-X-Y-Z 🟦|
| [Blender.mathutils](https://docs.blender.org/api/blender_python_api_current/mathutils.html?highlight=vector#mathutils.Quaternion) | W-X-Y-Z 🟦|
| [Boost](https://www.boost.org/doc/libs/1_71_0/libs/math/example/HSO3.hpp) | W-X-Y-Z 🟦|
| [Bullet](https://pybullet.org/Bullet/BulletFull/classbtQuaternion.html) / [PyBullet](http://goo.gl/QwJnFX) | X-Y-Z-W 🟥|
| [DART](https://github.com/dartsim/dart/blob/5058255853d5b733476fc031b18fb95bdf7d7f5d/python/dartpy/eigen_geometry_pybind.cpp#L225) (uses Eigen::Quaternion)| W-X-Y-Z 🟦|
| [Drake](https://drake.mit.edu/pydrake/pydrake.common.eigen_geometry.html?highlight=quaternion#pydrake.common.eigen_geometry.Quaternion_[float]) (uses Eigen::Quaternion)| W-X-Y-Z 🟦|
| [differentiable-robot-model](https://github.com/facebookresearch/differentiable-robot-model/blob/7e58c1f286a57d48deaafc78bda0e3dedb8e5c4a/differentiable_robot_model/se3_so3_util.py) | X-Y-Z-W 🟥|
| [dm_robotics.transformations](https://github.com/deepmind/dm_robotics/tree/main/py/transformations)| W-X-Y-Z 🟦|
| [Eigen - API](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html)| W-X-Y-Z 🟦|
| [Eigen - Internal memory ordering](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html#a3eba7a582f77a8f30525614821d7056f)| X-Y-Z-W 🟥|
| [FleX](https://developer.nvidia.com/flex) | X-Y-Z-W 🟥|
| [Genesis](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/conventions.html) | W-X-Y-Z 🟦|
| [iDynTree](https://robotology.github.io/idyntree/classiDynTree_1_1Rotation.html#adcac444f00ca751417f3095b401de86c) | W-X-Y-Z 🟦 | 
| [Isaac Gym](https://developer.nvidia.com/isaac-gym) | X-Y-Z-W 🟥|
| [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/migration/migrating_from_isaacgymenvs.html#quaternion-convention) | W-X-Y-Z 🟦|
| [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/reference_conventions.html#quaternions) | W-X-Y-Z 🟦|
| [lietorch](https://github.com/princeton-vl/lietorch) | X-Y-Z-W 🟥|
| [jaxlie](https://brentyi.github.io/jaxlie/api/jaxlie/_se3/?highlight=quaternion#jaxlie._se3.SE3.wxyz_xyz) | W-X-Y-Z 🟦 | 
| [Klampt](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/_modules/klampt/math/so3.html#quaternion) | W-X-Y-Z 🟦|
| [MATLAB](https://www.mathworks.com/help/robotics/ref/quaternion.html) | W-X-Y-Z 🟦|
| [MuJoCo](http://mujoco.org/book/modeling.html#COrientation) | W-X-Y-Z 🟦|
| [Newton](https://newton-physics.github.io/newton/stable/concepts/conventions.html#quaternion-ordering-conventions) | X-Y-Z-W 🟥|
| [Nuklei](http://nuklei.sourceforge.net/doxygen/) | W-X-Y-Z 🟦|
| [numpy-quaternions](https://github.com/moble/quaternion) | W-X-Y-Z 🟦|
| [Open3D](https://github.com/isl-org/Open3D/blob/main/cpp/open3d/geometry/Geometry3D.cpp) (uses Eigen::Quaternion)| W-X-Y-Z 🟦|
| [OpenRAVE 0.9.0](http://openrave.org/docs/latest_stable/coreapihtml/geometry_8h_source.html) | X-Y-Z-W 🟥|
| [Orocos KDL](http://docs.ros.org/jade/api/orocos_kdl/html/classKDL_1_1Rotation.html) | X-Y-Z-W 🟥|
| [PhysX](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/apireference/files/classPxQuat.html) | X-Y-Z-W 🟥|
| [PyMesh](https://pymesh.readthedocs.io/en/latest/api_misc.html#quaternion) | W-X-Y-Z 🟦|
| [pyquaternion](https://github.com/KieranWynn/pyquaternion) | W-X-Y-Z 🟦|
| [pyrr](https://pyrr.readthedocs.io/en/latest/_modules/pyrr/quaternion.html) | X-Y-Z-W 🟥|
| [pytorch3d](https://pytorch3d.readthedocs.io/en/latest/_modules/pytorch3d/transforms/rotation_conversions.html#standardize_quaternion) | W-X-Y-Z 🟦|
| [pytransform3d](https://github.com/dfki-ric/pytransform3d/blob/c45e817c4a7960108afe9f5259542c8376c0e89a/pytransform3d/rotations/_quaternion_operations.py#L22) | W-X-Y-Z 🟦|
| [quaternions](https://github.com/mjsobrep/quaternions/blob/master/quaternions/quaternion.py) | W-X-Y-Z 🟦|
| [RoMa](https://naver.github.io/roma/#main-features) | X-Y-Z-W 🟥|
| [ROS](https://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html) | X-Y-Z-W 🟥|
| [ROS2 (tf2)](https://github.com/ros2/geometry2/blob/rolling/tf2/include/tf2/LinearMath/QuadWord.h) | X-Y-Z-W 🟥|
| [scipy.spatial.transforms](https://docs.scipy.org/doc/scipy/reference/spatial.transform.html#) | X-Y-Z-W 🟥|
| [spatialmath-python](https://bdaiinstitute.github.io/spatialmath-python/func_quat.html#module-spatialmath.base.quaternions) | W-X-Y-Z 🟦|
| [squaternion](https://github.com/MomsFriendlyRobotCompany/squaternion/blob/master/squaternion/squaternion.py) | W-X-Y-Z 🟦|
| [The Library for Uniform Deterministic Sequences and Sets of Samples over 2-sphere and SO(3)](http://lavalle.pl/software/so3/so3.html)| X-Y-Z-W 🟥|
| [Theseus](https://github.com/facebookresearch/theseus/blob/e6dd7937edee59daf5f5c2ffe870f39fa1171e90/theseus/embodied/kinematics/kinematics_model.py#L44)| W-X-Y-Z 🟦|
| [transformations.py (<=2009.04.18)](http://docs.ros.org/jade/api/tf/html/python/transformations.html) | X-Y-Z-W 🟥|
| [transformations.py (>=2010.05.10](https://www.lfd.uci.edu/~gohlke/code/transformations.py.html), includes [trimesh.transformations)](https://github.com/mikedh/trimesh/blob/master/trimesh/transformations.py) | W-X-Y-Z 🟦|
| [Transforms3d](https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html)| W-X-Y-Z 🟦|
| [tinyquaternion](https://github.com/rezaahmadzadeh/tinyquaternion/blob/master/tinyquaternion/tinyQuaternion.py)| W-X-Y-Z 🟦|
| [USD (Pixar) - API](https://graphics.pixar.com/usd/release/api/class_gf_quatf.html#a781cffeee14aa3ba3f89de7d6df5a035)| W-X-Y-Z 🟦|
| [USD (Pixar) - Internal memory ordering](https://graphics.pixar.com/usd/release/api/quatf_8h_source.html)| X-Y-Z-W 🟥|
| [ViSII - A VIrtual Scene Imaging Interface](https://owl-project.github.io/ViSII/all.html#visii.quat)| W-X-Y-Z 🟦|
| [NVIDIA Warp](https://nvidia.github.io/warp/_build/html/modules/functions.html?highlight=transform#quaternion-math) | X-Y-Z-W 🟥|
| [YARP - API](https://www.yarp.it/latest/classyarp_1_1math_1_1Quaternion.html) | X-Y-Z-W 🟥 |
| [YARP - Internal memory ordering](https://github.com/robotology/yarp/blob/v3.8.0/src/libYARP_math/src/yarp/math/Quaternion.h#L23) | W-X-Y-Z 🟦 |
