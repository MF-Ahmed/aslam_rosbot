from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['aslam_turtlebot_mpc'],
    package_dir={'': 'libs'}
)
setup(**d)
