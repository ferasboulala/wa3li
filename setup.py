import os
from glob import glob
from setuptools import setup

package_name = 'wa3li'

setup(
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.py'))
    ]
)
