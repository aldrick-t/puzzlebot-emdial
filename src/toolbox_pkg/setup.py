from setuptools import find_packages, setup

package_name = 'toolbox_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atad',
    maintainer_email='taldrick@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_preprocess = toolbox_pkg.cam_preprocess:main',
            'img_capture = toolbox_pkg.img_capture:main',
            'path_recorder = toolbox_pkg.path_recorder:main',
        ],
    },
)
