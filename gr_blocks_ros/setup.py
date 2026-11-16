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
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/data", glob("../data/*.json")),
        ("share/" + package_name + "/data/cached_plans", glob("../data/cached_plans/*.json")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/worlds/textures", glob("worlds/textures/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "gripper_check = gr_blocks_ros.gripper_check:main",
            "save_image = gr_blocks_ros.save_image:main",
            "move_named = gr_blocks_ros.move_named:main",
            "world_state = gr_blocks_ros.world_state:main",
            "gripper_gap = gr_blocks_ros.gripper_gap:main",
            "pick_test = gr_blocks_ros.pick_test:main",
            "place_test = gr_blocks_ros.place_test:main",
            "release_test = gr_blocks_ros.release_test:main",
            "creep_test = gr_blocks_ros.creep_test:main",
            "grip_sweep = gr_blocks_ros.grip_sweep:main",
            "replay_helper = gr_blocks_ros.replay_helper:main",
            "full_loop = gr_blocks_ros.full_loop:main",
            "make_world = gr_blocks_ros.make_world:main",
        ],
    },
)
