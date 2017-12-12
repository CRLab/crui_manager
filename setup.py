# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name="crui_manager",
    description="Code to manage input from controllers and output to a UI",
    packages=['crui_manager'],
    package_dir={'': 'src'})

setup(**setup_args)