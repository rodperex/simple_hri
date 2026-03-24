
# simple_hri

A lightweight Python package providing speech-to-text (STT) and text-to-speech (TTS) services optimized for Human-Robot Interaction (HRI) applications with ROS 2.

## Features

- **Speech-to-Text (STT):** Convert audio to text using OpenAI Whisper API (cloud) or local Whisper models
- **Text-to-Speech (TTS):** Generate natural speech from text using Google Cloud TTS (cloud) or local TTS models
- **Extract Service:** Extract specific information from audio using OpenAI LLMs (cloud), local transformers models, or HuggingFace Inference API (free)
- **Yes/No Service:** Detect confirmation responses in simple interactions (cloud or local)
- **Voice Activity Detection (VAD):** Automatically record audio when detecting human voice
- **Audio Playback:** Integrated services to play audio on the robot
- **Three Deployment Options:** 
  - **Cloud:** High-quality APIs with OpenAI and Google Cloud (requires Internet and paid credentials)
  - **Local:** Fully offline using transformers models (no Internet required after setup, optional HF_TOKEN)
  - **Free:** Mix of local models + HuggingFace free API (requires free HF_TOKEN)

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

### 4. Configure API credentials (only for cloud services)

**⚠️ Note:** API credentials are **only required** if you plan to use the **cloud-based launcher** (`simple_hri.launch.py`). The local and free launchers work completely offline without any API keys.

#### OpenAI API (for cloud STT, Extract, and Yes/No services)
```bash
export OPENAI_API_KEY="your_openai_api_key"
```

#### Google Cloud TTS (for cloud TTS service)
```bash
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/your/google_credentials.json"
```

> **Note:** To obtain Google Cloud credentials, follow [these instructions](https://cloud.google.com/text-to-speech/docs/quickstart-client-libraries).

#### HuggingFace Token (only for free launcher)
```bash
export HF_TOKEN="your_huggingface_token"
```

> **Note:** 
> - **Mandatory** for `free_simple_hri.launch.py` (uses HuggingFace Inference API)
> - **Not needed** for `local_simple_hri.launch.py` (works without token, may help avoid rate limits during first download)
> - Get your free token at: https://huggingface.co/settings/tokens

**Summary of credential requirements by launcher:**
- **simple_hri.launch.py** → Requires `OPENAI_API_KEY` + `GOOGLE_APPLICATION_CREDENTIALS`
- **local_simple_hri.launch.py** → No credentials needed (works without any tokens)
- **free_simple_hri.launch.py** → **Requires `HF_TOKEN`** (uses HuggingFace Inference API)

### 5. Build the workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Available Launch Files

The package provides **three launch files** to accommodate different deployment scenarios:

#### 1. Cloud-based services (simple_hri.launch.py)

Uses cloud APIs for all services. **Requires Internet connection and API credentials.**

```bash
ros2 launch simple_hri simple_hri.launch.py
```

**Models and APIs used:**
- **STT:** OpenAI Whisper API (cloud)
- **TTS:** Google Cloud Text-to-Speech API (cloud)
- **Extract:** OpenAI GPT models (cloud)
- **Yes/No:** Uses cloud STT internally

**Required credentials:**
- `OPENAI_API_KEY` environment variable
- `GOOGLE_APPLICATION_CREDENTIALS` environment variable (path to JSON file)

#### 2. Local services (local_simple_hri.launch.py)

Runs completely offline using local models. **No Internet connection required after initial setup.**

```bash
ros2 launch simple_hri local_simple_hri.launch.py
```

**Models used:**
- **STT:** Local Whisper model (`openai-whisper` library)
- **TTS:** HuggingFace model `facebook/mms-tts-{lang}` via `transformers`
- **Extract:** HuggingFace model (e.g., `google/flan-t5-small`) via `transformers` - local inference
- **Yes/No:** Local STT processing

**Required credentials:** 
- **None** - Works completely without any tokens
- HF_TOKEN not required (transformers downloads public models without authentication)
- Optional: Setting `HF_TOKEN` may help avoid rate limits during first download

**First run:** Models will be automatically downloaded to `./models/` directory (requires Internet)

#### 3. Free/HuggingFace services (free_simple_hri.launch.py)

Uses HuggingFace Inference API for extract service. **Requires HF_TOKEN but uses free tier.**

```bash
ros2 launch simple_hri free_simple_hri.launch.py
```

**Models used:**
- **STT:** Local Whisper model (HuggingFace)
- **TTS:** HuggingFace model `facebook/mms-tts-{lang}` via `transformers` (local)
- **Extract:** HuggingFace Inference API with `meta-llama/Meta-Llama-3-8B-Instruct` (remote, free tier)
- **Yes/No:** Local STT processing

**Required credentials:** 
- **Mandatory:** `HF_TOKEN` (HuggingFace token) - Required by `extract_service_hugg`
  - Get your free token at: https://huggingface.co/settings/tokens
  - Uses HuggingFace free inference API (no charges)

**First run:** Local models will be downloaded to `./models/` directory (requires Internet)

### Test the services

```bash
ros2 run simple_hri test_services
```

## Available Services

### 1. Speech-to-Text (STT)

**Service:** `/stt_service` (type: `std_srvs/srv/SetBool`)

Records audio when voice is detected and transcribes it to text.

**Python usage example:**
```python
from std_srvs.srv import SetBool

# Create service client
stt_client = self.create_client(SetBool, '/stt_service')
stt_client.wait_for_service()

# Call the service
request = SetBool.Request()
request.data = True
response = stt_client.call(request)

if response.success:
    self.get_logger().info(f"Transcribed text: {response.message}")
```

**Published topic:** `/listened_text` (type: `std_msgs/msg/String`)

### 2. Text-to-Speech (TTS)

**Service:** `/tts_service` (type: `simple_hri_interfaces/srv/Speech`)

Converts text to speech and plays it.

**Python usage example:**
```python
from simple_hri_interfaces.srv import Speech

# Create service client
tts_client = self.create_client(Speech, '/tts_service')
tts_client.wait_for_service()

# Call the service
request = Speech.Request()
request.text = "Hello, I am a robot"
response = tts_client.call(request)

if response.success:
    self.get_logger().info("Audio played successfully")
```

### 3. Extract Service

**Service:** `/extract_service` (type: `simple_hri_interfaces/srv/Extract`)

Records audio, transcribes it, and extracts specific information using an LLM.

**Python usage example:**
```python
from simple_hri_interfaces.srv import Extract

# Create service client
extract_client = self.create_client(Extract, '/extract_service')
extract_client.wait_for_service()

# Call the service
request = Extract.Request()
request.interest = "person's name"  # What information to extract
response = extract_client.call(request)

if response.success:
    self.get_logger().info(f"Extracted information: {response.extracted}")
```

### 4. Yes/No Service

**Service:** `/yesno_service` (type: `std_srvs/srv/SetBool`)

Detects if the user says "yes" or "no".

**Python usage example:**
```python
from std_srvs.srv import SetBool

yesno_client = self.create_client(SetBool, '/yesno_service')
yesno_client.wait_for_service()

request = SetBool.Request()
request.data = True
response = yesno_client.call(request)

if response.success:
    answer = "Yes" if "yes" in response.message.lower() else "No"
    self.get_logger().info(f"User answered: {answer}")
```

### 5. Audio File Player

**Service:** `/play_audio_file` (type: `std_srvs/srv/SetBool`)

Plays stored audio files.

## Available Nodes

| Node | Description | Type | API Keys Required | Command |
|------|-------------|------|-------------------|---------|
| `tts_service` | TTS with Google Cloud | Cloud | `GOOGLE_APPLICATION_CREDENTIALS` | `ros2 run simple_hri tts_service` |
| `tts_service_local` | Local TTS with transformers | Local | No | `ros2 run simple_hri tts_service_local` |
| `stt_service` | STT with Whisper API | Cloud | `OPENAI_API_KEY` | `ros2 run simple_hri stt_service` |
| `stt_service_local` | Local STT with Whisper | Local | No | `ros2 run simple_hri stt_service_local` |
| `extract_service` | Extraction with OpenAI LLM | Cloud | `OPENAI_API_KEY` | `ros2 run simple_hri extract_service` |
| `extract_service_local` | Local extraction (transformers) | Local | No | `ros2 run simple_hri extract_service_local` |
| `extract_service_hugg` | Extraction with HF Inference API | Remote (Free) | **`HF_TOKEN`** (mandatory) | `ros2 run simple_hri extract_service_hugg` |
| `yesno_service` | Yes/No detection (cloud STT) | Cloud | `OPENAI_API_KEY` | `ros2 run simple_hri yesno_service` |
| `yesno_service_local` | Yes/No detection (pattern matching) | Local | No | `ros2 run simple_hri yesno_service_local` |
| `audio_service` | Audio recording service | Local | No | `ros2 run simple_hri audio_service` |
| `audio_file_player` | Audio file player | Local | No | `ros2 run simple_hri audio_file_player` |

## Integrating into Your Project

### Option 1: Use as dependency in package.xml

```xml
<package format="3">
  <name>your_package</name>
  <!-- ... -->
  
  <exec_depend>simple_hri</exec_depend>
  <exec_depend>simple_hri_interfaces</exec_depend>
  <exec_depend>sound_play</exec_depend>
  <exec_depend>audio_common</exec_depend>
  
</package>
```

### Option 2: Include in your launch file

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    simple_hri_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('simple_hri'),
                'launch',
                'simple_hri.launch.py'
            )
        ])
    )
    
    return LaunchDescription([
        simple_hri_launch,
        # Your other nodes...
    ])
```

### Option 3: Create a client node

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from simple_hri_interfaces.srv import Speech
from std_msgs.msg import String

class MyHRINode(Node):
    def __init__(self):
        super().__init__('my_hri_node')
        
        # Service clients
        self.stt_client = self.create_client(SetBool, '/stt_service')
        self.tts_client = self.create_client(Speech, '/tts_service')
        
        # Subscriber to listened text
        self.text_sub = self.create_subscription(
            String,
            '/listened_text',
            self.text_callback,
            10
        )
        
    def text_callback(self, msg):
        self.get_logger().info(f'Received text: {msg.data}')
        
    def speak(self, text):
        request = Speech.Request()
        request.text = text
        future = self.tts_client.call_async(request)
        return future
        
    def listen(self):
        request = SetBool.Request()
        request.data = True
        future = self.stt_client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = MyHRINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Advanced Configuration

### TTS Service Parameters

```bash
ros2 run simple_hri tts_service --ros-args -p play_sound:=false
```

- `play_sound`: If `false`, publishes audio data instead of playing it directly

### Customize Extract Service prompts

Edit the files in `params/` to modify the LLM prompts:
- `basic_prompt_es.txt`: Spanish prompt
- `basic_prompt_en.txt`: English prompt

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

### Dependency issues
Reinstall requirements in a virtual environment:
```bash
python3 -m venv ~/simple_hri_env
source ~/simple_hri_env/bin/activate
pip install -r requirements.txt
```

## Project Structure

```
simple_hri/
├── simple_hri/              # Python source code
│   ├── stt_service.py       # STT with Whisper API
│   ├── stt_service_local.py # Local STT
│   ├── tts_service.py       # TTS with Google Cloud
│   ├── tts_service_local.py # Local TTS
│   ├── extract_service.py   # Extraction with LLM
│   ├── yesno_service.py     # Yes/No detector
│   ├── audio_service.py     # Audio service
│   └── test_services.py     # Tests
├── launch/                  # Launch files
│   ├── simple_hri.launch.py
│   └── local_simple_hri.launch.py
├── params/                  # Configuration files
├── requirements.txt         # Python dependencies
├── package.xml             # ROS 2 manifest
└── setup.py                # Python setup

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