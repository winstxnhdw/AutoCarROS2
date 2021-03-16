from setuptools import setup, find_packages

package_name = "utils"

setup(
    packages=find_packages(package_name),
    package_dir={'': 'src'},
)