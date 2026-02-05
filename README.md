# simple_hri

A lightweight Python package providing speech-to-text (STT) and text-to-speech (TTS) services optimized for Human-Robot Interaction (HRI) applications with ROS 2.

## Features

- **Speech-to-Text (STT):** Convert audio to text using OpenAI Whisper API or local models
- **Text-to-Speech (TTS):** Generate natural speech from text using Google Cloud TTS or local models
- **Extract Service:** Extract specific information from audio using OpenAI LLMs
- **Yes/No Service:** Detect confirmation responses in simple interactions
- **Voice Activity Detection (VAD):** Automatically record audio when detecting human voice
- **Audio Playback:** Integrated services to play audio on the robot
- **Cloud and Local Versions:** Online (APIs) and offline (local models) options

## Requirements

- **ROS 2 Jazzy** (or compatible versions)
- **Python 3.12+**
- Required ROS 2 packages:
  - `audio_common` (https://github.com/rodperex/audio_common - branch: ros2)
  
## Installation

### 1. Clone the repository into your ROS 2 workspace

```bash
cd ~/ros2_ws/src
git clone https://github.com/rodperex/simple_hri.git
```

### 2. Install third-party dependencies using vcs

```bash
cd ~/ros2_ws
vcs import src < src/simple_hri/thirdparty.repos
```

This will automatically clone the required `audio_common` repository.

### 3. Install Python dependencies

```bash
cd ~/ros2_ws/src/simple_hri
pip install -r simple_hri/requirements.txt
```

### 4. Configure API credentials

#### OpenAI API (for STT with Whisper and Extract Service)
```bash
export OPENAI_API_KEY="your_openai_api_key"
```

#### Google Cloud TTS (for TTS)
```bash
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/your/google_credentials.json"
```

> **Note:** To obtain Google Cloud credentials, follow [these instructions](https://cloud.google.com/text-to-speech/docs/quickstart-client-libraries).

### 5. Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch all services (cloud version)

```bash
ros2 launch simple_hri simple_hri.launch.py
```

### Launch local services (no Internet connection required)

```bash
ros2 launch simple_hri local_simple_hri.launch.py
```

### Test the services

```bash
ros2 run simple_hri test_services
```

## Available Services

### 1. Speech-to-Text (STT)

**Service:** `/stt_service` (type: `std_srvs/srv/SetBool`)

Records audio when voice is detected and transcribes it to text.

**Published topic:** `/listened_text` (type: `std_msgs/msg/String`)

### 2. Text-to-Speech (TTS)

**Service:** `/tts_service` (type: `simple_hri_interfaces/srv/Speech`)

Converts text to speech and plays it.

### 3. Extract Service

**Service:** `/extract_service` (type: `simple_hri_interfaces/srv/Extract`)

Records audio, transcribes it, and extracts specific information using an LLM.

### 4. Yes/No Service

**Service:** `/yesno_service` (type: `std_srvs/srv/SetBool`)

Detects if the user says "yes" or "no".

### 5. Audio File Player

**Service:** `/play_audio_file` (type: `std_srvs/srv/SetBool`)

Plays stored audio files.

## Available Nodes

| Node | Description | Command |
|------|-------------|---------|
| `tts_service` | TTS with Google Cloud (online) | `ros2 run simple_hri tts_service` |
| `tts_service_local` | Local TTS with transformers | `ros2 run simple_hri tts_service_local` |
| `stt_service` | STT with Whisper API (online) | `ros2 run simple_hri stt_service` |
| `stt_service_local` | Local STT with Whisper | `ros2 run simple_hri stt_service_local` |
| `extract_service` | Extraction with LLM (online) | `ros2 run simple_hri extract_service` |
| `extract_service_local` | Local extraction | `ros2 run simple_hri extract_service_local` |
| `yesno_service` | Yes/No detection | `ros2 run simple_hri yesno_service` |
| `audio_service` | Audio service | `ros2 run simple_hri audio_service` |
| `audio_file_player` | File player | `ros2 run simple_hri audio_file_player` |

## Troubleshooting

### Error: "API key not found"
Make sure to export the environment variables before launching the services:
```bash
echo $OPENAI_API_KEY
echo $GOOGLE_APPLICATION_CREDENTIALS
```

### Error: "Service not available"
Verify that the services are running:
```bash
ros2 service list | grep -E "(stt|tts)"
```

### Audio not recording properly
Check microphone permissions and that `sounddevice` detects your device:
```bash
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

## Repository Structure

```
simple_hri/
├── audio_send_interfaces/     # Audio interfaces (ROS 1 compatibility)
├── simple_hri/                # Main Python package
│   ├── simple_hri/            # Python source code
│   ├── launch/                # Launch files
│   ├── params/                # Configuration files
│   ├── requirements.txt       # Python dependencies
│   ├── package.xml           # ROS 2 manifest
│   └── setup.py              # Python setup
├── simple_hri_interfaces/     # Custom ROS 2 interfaces
├── thirdparty.repos          # Third-party dependencies
└── README.md                 # This file
```

## Contributing

Contributions are welcome. Please:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Authors

- **Rodrigo Pérez** - [rodperex](https://github.com/rodperex)
- **Antonio Bono**

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.

## References

- [OpenAI Whisper](https://openai.com/research/whisper)
- [Google Cloud Text-to-Speech](https://cloud.google.com/text-to-speech)
- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/index.html)
- [audio_common](https://github.com/rodperex/audio_common)
