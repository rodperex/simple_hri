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
            # One node per service and backend: <service>_<backend>
            'stt_whisper = simple_hri.stt_whisper:main',
            'stt_openai = simple_hri.stt_openai:main',
            'tts_mms = simple_hri.tts_mms:main',
            'tts_google = simple_hri.tts_google:main',
            'llm_local = simple_hri.llm_local:main',  # Extract and Yes/No with one local model
            'extract_local = simple_hri.llm_local:main_extract',
            'extract_ollama = simple_hri.extract_ollama:main',
            'extract_hf = simple_hri.extract_hf:main',
            'extract_openai = simple_hri.extract_openai:main',
            'yesno_local = simple_hri.llm_local:main_yesno',
            'yesno_keywords = simple_hri.yesno_keywords:main',
            'yesno_ollama = simple_hri.yesno_ollama:main',
            'yesno_hf = simple_hri.yesno_hf:main',
            'yesno_openai = simple_hri.yesno_openai:main',
            'audio_service = simple_hri.audio_service:main',
            'audio_file_player = simple_hri.audio_file_player:main',
            'test_services = simple_hri.test_services:main',
            'test_extract = simple_hri.test_extract:main',
            'test_llm = simple_hri.test_llm:main',
            # Previous names, kept so that existing launch files and scripts keep working
            'stt_service = simple_hri.stt_openai:main',
            'stt_service_local = simple_hri.stt_whisper:main',
            'tts_service = simple_hri.tts_google:main',
            'tts_service_local = simple_hri.tts_mms:main',
            'extract_service = simple_hri.extract_openai:main',
            'extract_service_local = simple_hri.llm_local:main_extract',
            'extract_service_hugg = simple_hri.extract_hf:main',
            'yesno_service = simple_hri.yesno_openai:main',
            'yesno_service_local = simple_hri.yesno_keywords:main',
        ],
    },
)
