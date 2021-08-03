from setuptools import setup, find_packages


with open("requirements.txt", "r") as fh:
    requirements = fh.readlines()

setup(name='rogata_library',
version='0.9',
description='Robotic Games Tabletop Engine',
url='https://rogata-engine.readthedocs.io/en/latest/what_is_rogata.html',
author='Jan Baumgärtner',
author_email='jan.baumgaertner@ziti.uni-heidelberg.de',
license='MIT',
packages=['rogata_library'],
install_requires=[req for req in requirements if req[:2] != "# "],
zip_safe=False)
