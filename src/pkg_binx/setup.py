from setuptools import setup

package_name = 'pkg_binx'

setup(
    name=package_name,
    version='0.0.0',
    packages=['mecanum_drive'],  # folder containing your node
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cloudy',
    maintainer_email='cloudy.white04@gmail.com',
    description='Mecanum drive node for binx robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mecanum_drive_node = mecanum_drive.mecanum_drive_node:main',
        ],
    },
)
