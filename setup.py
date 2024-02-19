from setuptools import find_packages, setup

package_name = "isaac_crowds_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryan Wüest",
    maintainer_email="ryan.wueest@protonmail.com",
    description="Isaac simulator environment to simulate scai wheelchair in airport environment",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["waypoint_controller = isaac_crowds_sim.waypoint_controller:main"],
    },
)
