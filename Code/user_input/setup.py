from setuptools import find_packages, setup

package_name = 'user_input'

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
    maintainer='sciarcan',
    maintainer_email='riccardo.sciacca21@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_pub = user_input.input_pub:main',
            'nao_camera_viewer = user_input.nao_camera_viewer:main',
            'nao_routine = user_input.nao_routine:main',
        ],
    },
)
