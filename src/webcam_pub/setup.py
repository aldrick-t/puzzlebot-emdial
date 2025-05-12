from setuptools import setup

package_name = 'webcam_pub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Webcam publisher using OpenCV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'webcam_publisher = webcam_pub.webcam_publisher:main'
        ],
    },
)