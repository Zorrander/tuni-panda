from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['robot_env', 'transformer_il', 'franka-pybullet'],
    package_dir={'': 'src'},
)

setup(**setup_args)