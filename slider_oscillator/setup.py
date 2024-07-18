from setuptools import find_packages, setup

package_name = 'slider_oscillator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpm',
    maintainer_email='cs1221722@iitd.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slider_oscillator = slider_oscillator.slider_oscillator:main',
        ],
    },
)
