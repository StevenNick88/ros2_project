from setuptools import setup

package_name = 'py08_tf_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tt',
    maintainer_email='tt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_tf_broadcaster = py08_tf_broadcaster.dynamic_tf_broadcaster:main'
        ],
    },
)
