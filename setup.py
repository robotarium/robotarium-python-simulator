from setuptools import setup, find_packages

with open('README.md') as f:
    long_description = f.read()

setup(
    name='robotarium',
    version='0.3.0',
    author='Zahi Kakish',
    url='https://github.com/zmk5/robotarium-python-simulator',
    description='simulator for the Georgia Tech Robotarium',
    long_description=long_description,
    license='MIT',
    keywords=['robotics simulation'],
    install_requires=['numpy', 'matplotlib', 'sphinx', 'numpydoc'],
    include_package_data=True,
    packages=find_packages(),
    test_suite='tests',
    setup_requires=['pytest-runner'],
    tests_require=['pytest'],
    classifiers=['Programming Language :: Python :: 2',
                 'Programming Language :: Python :: 3',
                 'Intended Audience :: Science/Research',
                 'License :: OSI Approved :: MIT License',
                 'Topic :: Scientific/Engineering']
)
