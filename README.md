
# simple_hri

A lightweight Python package providing speech-to-text and text-to-speech services tailored for Human-Robot Interaction (HRI) applications.

## Features

- **Speech-to-Text (STT):** Convert spoken language into written text using robust speech recognition.
- **Text-to-Speech (TTS):** Generate natural-sounding speech from written text.
- **Optimized for HRI:** Designed with human-robot interaction scenarios in mind.

## Installation

Clone the repository and install the necessary dependencies:

```bash
git clone https://github.com/rodperex/simple_hri.git
cd simple_hri
pip install -r requirements.txt
```

## Environment Variables

Before running the services, you need to export your API credentials:

```bash
# OpenAI Whisper API key
export OPENAI_API_KEY="your_openai_api_key"

# Google Cloud credentials JSON file for Text-to-Speech
export GOOGLE_APPLICATION_CREDENTIALS="/path/to/your/google_credentials.json"
```

## Usage

Launch the services:
```bash
ros2 launch simple_hri simple_hri.launch.py
```

Test it!:
```bash
ros2 run simple_hri test_services
```

## Project Structure

- `simple_hri/`: Core package containing the main functionalities.
- `tests/`: Some tests.
- `setup.py`: Installation script for the package.
- `requirements.txt`: List of dependencies required to run the package.

## License

This project is licensed under the Apache-2.0 License - see the [LICENSE](LICENSE) file for details.