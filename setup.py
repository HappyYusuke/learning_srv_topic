from setuptools import find_packages, setup

package_name = 'learning_srv_topic'

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
    maintainer='demulab-kohei',
    maintainer_email='c1005073@planet.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_topic = learning_srv_topic.pub_topic:main',
            'bad_srv_server = learning_srv_topic.bad_srv_server:main',
            'good_srv_server = learning_srv_topic.good_srv_server:main',
        ],
    },
)
