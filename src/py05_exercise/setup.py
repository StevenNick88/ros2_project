from setuptools import setup
import os
from glob import glob

package_name = 'py05_exercise'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.xml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exer01_spawn = py05_exercise.exer01_spawn:main',
            'exer02_tf_broadcaster = py05_exercise.exer02_tf_broadcaster:main',
            'exer03_tf_listener = py05_exercise.exer03_tf_listener:main',
        ],
    },
)
