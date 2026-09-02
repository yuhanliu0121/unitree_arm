from glob import glob
from setuptools import find_packages, setup


package_name = "d1_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Yuhan Liu",
    maintainer_email="yuhanliu0813@gmail.com",
    description="Real-machine bringup, preflight, and recovery for the Unitree D1 stack.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "real_preflight = d1_bringup.real_preflight:main",
            "recover_stowed = d1_bringup.recover_stowed:main",
            "stack_readiness = d1_bringup.stack_readiness:main",
        ],
    },
)
