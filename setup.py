from setuptools import setup
import os
from glob import glob

package_name = 'norlab_sound_indicators'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools', 'scikit-learn', 'numpy'],
    zip_safe=True,
    maintainer='Matej Boxan',
    maintainer_email='matej.boxan@gmail.com',
    description='Beeping controller quality indicator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'norlab_sound_indicators = norlab_sound_indicators.norlab_sound_indicators:main'
        ],
    },
)
