from setuptools import setup

setup(
    name='python-MCP4728',
    version='0.0.1',
    packages=['mcp4728'],
    url='https://github.com/PaulKGrimes/python-MCP4728',
    license='GPL v2.0',
    author='Paul Grimes',
    author_email='pgrimes@cfa.harvard.edu',
    description='Package for communication with MCP4728 DAC convertor over i2c',
    install_requires=['smbus']
)
