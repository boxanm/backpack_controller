from setuptools import setup

package_name = 'backpack_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'scikit-learn', 'numpy', 'preferredsoundplayer'],
    zip_safe=True,
    maintainer='Matej Boxan',
    maintainer_email='matej.boxan@gmail.com',
    description='Basic backpack controller playing sound',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'backpack_controller = backpack_controller.backpack_controller:main'
        ],
    },
)
