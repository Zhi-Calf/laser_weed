from setuptools import setup

package_name = "stanley_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laser-weed",
    maintainer_email="team@example.com",
    description="Stanley lateral control thin-shell node for the laser weeding robot",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "stanley_node = stanley_control.stanley_node:main",
            "sim_demo = stanley_control.sim_demo:main",
        ],
    },
)
