from glob import glob

from setuptools import find_packages, setup

package_name = "gr_blocks_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.json") + glob("config/*.yaml")),
        ("share/" + package_name + "/data", glob("../data/*.json")),
        ("share/" + package_name + "/data/cached_plans", glob("../data/cached_plans/*.json")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/worlds/textures", glob("worlds/textures/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
        ],
    },
)
