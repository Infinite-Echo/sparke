from setuptools import setup

package_name = "servo_move_keyboard"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="echo",
    maintainer_email="colin.fuelberth@icloud.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servoMoveKeyboard = servo_move_keyboard.servoMoveKeyboard:main",
            "servoConfigTest = servo_move_keyboard.servoConfigTest:main",
        ],
    },
)
