from setuptools import find_packages, setup

package_name = 'mc1_opencontrol'

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
            'square_path = mc1_opencontrol.square_path:main',
            'user_path = mc1_opencontrol.user_path:main',

        ],
    },
)
