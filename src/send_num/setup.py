from setuptools import find_packages, setup

package_name = 'send_num'

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
    maintainer='danil',
    maintainer_email='podkolzindanil@yandex.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send = send_num.send:main',
            'send_test_copy = send_num.send_test_copy:main',
            'test_server_copy = send_num.test_server_copy:main',
        ],
    },
)
