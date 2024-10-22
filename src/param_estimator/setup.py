from setuptools import find_packages, setup

package_name = 'param_estimator'

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
    maintainer='aimotion-laptop-i9',
    maintainer_email='aimotionlab.sztaki@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_estimator = param_estimator.param_estimator:main'
        ],
    },
)
