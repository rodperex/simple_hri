from setuptools import setup

package_name = 'simple_hri'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_hri.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROI',
    maintainer_email='rodrigo.perez@urjc.es',
    description='Simple STT and TTS services for HRI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tts_service = simple_hri.tts_service:main',
            'stt_service = simple_hri.stt_service:main',
            'stt_service_local = simple_hri.stt_service_local:main',
            'test_services = simple_hri.test_services:main',
        ],
    },
)
