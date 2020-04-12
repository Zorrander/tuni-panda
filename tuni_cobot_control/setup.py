from setuptools import setup

package_name = 'tuni_cobot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'owlready2'
    ],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexandre.angleraud@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = tuni_cobot_control.planner:main',
        ],
    },
)