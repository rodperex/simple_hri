from setuptools import setup
from glob import glob

package_name = 'simple_hri'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*')),
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
            'tts_service_local = simple_hri.tts_service_local:main',
            'tts_service_hf = simple_hri.tts_service_hf:main',
            'stt_service = simple_hri.stt_service:main',
            'stt_service_local = simple_hri.stt_service_local:main',
            'extract_service = simple_hri.extract_service:main',
            'yesno_service = simple_hri.yesno_service:main',
            'yesno_service_local = simple_hri.yesno_service_local:main',
            'extract_service_local = simple_hri.extract_service_local:main',
            'test_services = simple_hri.test_services:main',
            'test_extract = simple_hri.test_extract:main', 
            'audio_service = simple_hri.audio_service:main',
            'audio_file_player = simple_hri.audio_file_player:main',
        ],
    },
)
