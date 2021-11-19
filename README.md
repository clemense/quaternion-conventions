# Quaternion Order Conventions
Quaternions are expressions of the form `w + x i + y j + z k`, with `x i + y j + z k` being the vector/imaginary part and `w` the scalar/real part. Math libraries differ in the way they order the vector and real part of a quaternion. This can lead to bothersome debugging sessions. 

The following table shows quaternion implementations and their chosen order:<br>
X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+) (scalar-last format) &nbsp; vs. &nbsp; W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)

| Library / Framework |  Quaternion Format |
| --- | --- |
| [Blender.mathutils](https://docs.blender.org/api/blender_python_api_current/mathutils.html?highlight=vector#mathutils.Quaternion) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [Boost](https://www.boost.org/doc/libs/1_71_0/libs/math/example/HSO3.hpp) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [Bullet](https://pybullet.org/Bullet/BulletFull/classbtQuaternion.html) / [PyBullet](http://goo.gl/QwJnFX) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [Eigen](https://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html)| W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [FleX](https://developer.nvidia.com/flex) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [Isaac Gym](https://developer.nvidia.com/isaac-gym) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [Klampt](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/_modules/klampt/math/so3.html#quaternion) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [MuJoCo](http://mujoco.org/book/modeling.html#COrientation) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [Nuklei](http://nuklei.sourceforge.net/doxygen/) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [numpy-quaternions](https://github.com/moble/quaternion) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [OpenRAVE 0.9.0](http://openrave.org/docs/latest_stable/coreapihtml/geometry_8h_source.html) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [Orocos KDL](http://docs.ros.org/jade/api/orocos_kdl/html/classKDL_1_1Rotation.html) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [PhysX](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/apireference/files/classPxQuat.html) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [PyMesh](https://pymesh.readthedocs.io/en/latest/api_misc.html#quaternion) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [pyquaternion](https://github.com/KieranWynn/pyquaternion) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [pyrr](https://pyrr.readthedocs.io/en/latest/_modules/pyrr/quaternion.html) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [ROS](https://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [scipy.spatial.transforms](https://docs.scipy.org/doc/scipy/reference/spatial.transform.html#) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [squaternion](https://github.com/MomsFriendlyRobotCompany/squaternion/blob/master/squaternion/squaternion.py) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [The Library for Uniform Deterministic Sequences and Sets of Samples over 2-sphere and SO(3)](http://lavalle.pl/software/so3/so3.html)| X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [transformations.py (<=2009.04.18)](http://docs.ros.org/jade/api/tf/html/python/transformations.html) | X-Y-Z-W ![#ffa500](http://placehold.it/15/ffa500/000000?text=+)|
| [transformations.py (>=2010.05.10](https://www.lfd.uci.edu/~gohlke/code/transformations.py.html), includes [trimesh.transformations)](https://github.com/mikedh/trimesh/blob/master/trimesh/transformations.py) | W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [Transforms3d](https://matthew-brett.github.io/transforms3d/reference/transforms3d.quaternions.html)| W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [tinyquaternion](https://github.com/rezaahmadzadeh/tinyquaternion/blob/master/tinyquaternion/tinyQuaternion.py)| W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
| [ViSII - A VIrtual Scene Imaging Interface](https://owl-project.github.io/ViSII/all.html#visii.quat)| W-X-Y-Z ![#89cff0](http://placehold.it/15/89cff0/000000?text=+)|
