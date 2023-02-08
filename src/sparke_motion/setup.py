from setuptools import setup

package_name = 'sparke_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, (package_name + '/libs/sparkeKinematics/kinematics_np')],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='echo',
    maintainer_email='colin.fuelberth@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sparke_motion_ctrl_node = sparke_motion.sparke_motion_ctrl_node:main'
        ],
    },
)
