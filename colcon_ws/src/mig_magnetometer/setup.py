from setuptools import find_packages, setup

package_name = 'mig_magnetometer'

setup(
    name=package_name,
    version='0.0.0',
    packages=['mig_magnetometer'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mig',
    maintainer_email='mig@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'subpubmagneto = mig_magnetometer.subpub_magneto:main',
        ],
    },
)