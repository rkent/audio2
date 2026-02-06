from setuptools import find_packages, setup

package_name = 'piper_tts_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R. Kent James',
    maintainer_email='kent@caspia.com',
    description='piper-tts text to speech web server',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'piper_tts_server = piper_tts_server.piper_tts_server:ros_main',
        ],
    },
)
