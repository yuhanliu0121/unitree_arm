from glob import glob

from setuptools import find_packages, setup


package_name = "d1_camera_visualization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yuhan Liu",
    maintainer_email="yuhanliu0813@gmail.com",
    description="Backend-neutral camera diagnostics for D1 simulation and hardware.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "depth_debug_visualizer = d1_camera_visualization.depth_debug_visualizer:main",
        ],
    },
)
