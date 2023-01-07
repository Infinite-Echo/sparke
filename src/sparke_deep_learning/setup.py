from setuptools import setup

package_name = 'sparke_deep_learning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'sparke_deep_learning/sparke_deep_learning_submodule'],
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
            'sparke_AI_node = sparke_deep_learning.sparke_AI_node:main'
        ],
    },
)
