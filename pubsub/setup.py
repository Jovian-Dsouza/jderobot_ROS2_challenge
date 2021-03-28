from setuptools import setup

package_name = 'pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jovian',
    maintainer_email='dsouzajovian123@gmail.com',
    description='pub sub demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = pubsub.publisher:main',
            'subscriber = pubsub.subscriber:main',
        ],
    },
)
