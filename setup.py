# 子模块目录中创建 setup.py
from setuptools import setup, find_packages

setup(
    name="ompl_pkgs",
    version="0.1",
    packages=[
        package for package in find_packages() if package.startswith("ompl_planner")
    ],
    install_requires=[
        # 子模块的依赖项（如 requests>=2.0）
        "pygccxml==2.2.1",
    ],
    eager_resources=['*'],
)