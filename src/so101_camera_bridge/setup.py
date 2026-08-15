from setuptools import find_packages, setup


PACKAGE_NAME = "so101_camera_bridge"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Noah",
    maintainer_email="noah-wardlow@users.noreply.github.com",
    description="Read-only RTSP camera bridge for SO-101.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rtsp_camera_bridge = so101_camera_bridge.bridge_node:main",
        ],
    },
)
