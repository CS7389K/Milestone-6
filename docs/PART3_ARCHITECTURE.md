# Part 3 Distributed Architecture

## Overview

Part 3 has been refactored to support distributed deployment, separating the Whisper speech recognition workload from the TurtleBot to a remote PC. This prevents resource contention between Whisper and YOLO on the TurtleBot's Jetson.

## Architecture

### TurtleBot Side (part3.base.launch.py)
Runs the core mission with hardware integration:
- **Hardware Bringup**: TurtleBot3 manipulation arm and base controllers
- **YOLO Publisher**: Object detection using camera
- **Part 3 Node**: Voice-guided search and autonomous manipulation
  - Records audio locally using sounddevice
  - Publishes audio file path to `/audio_command` topic
  - Subscribes to `/voice_transcription` for results
  - Performs visual servoing and manipulation

### Remote PC Side (part3.nlp.launch.py)
Runs the Whisper transcription service:
- **Whisper Publisher**: Speech-to-text processing
  - Subscribes to `/audio_command` (receives audio file paths)
  - Loads and transcribes audio files
  - Publishes results to `/voice_transcription`

## Communication Flow

```
TurtleBot (part3.py)                    Remote PC (whisper_publisher)
     |                                           |
     | Records audio with sounddevice            |
     | Saves to /tmp/voice_input.wav             |
     |                                           |
     |----(audio file path)---> /audio_command >-|
     |                                           |
     |                          Loads audio file |
     |                          Transcribes with |
     |                          Whisper model    |
     |                                           |
     |<-- /voice_transcription <--(transcribed)-|
     |                                text       |
     | Parses command                            |
     | Executes action                           |
```

## Usage

### 1. Start TurtleBot Side
```bash
# On TurtleBot
ros2 launch milestone6 part3.base.launch.py
```

### 2. Start Remote PC Side
```bash
# On Remote PC (with good CPU/GPU)
ros2 launch milestone6 part3.nlp.launch.py

# Optional: Use CUDA for faster transcription
ros2 launch milestone6 part3.nlp.launch.py device:=cuda

# Optional: Use different model size
ros2 launch milestone6 part3.nlp.launch.py model_type:=tiny   # Faster
ros2 launch milestone6 part3.nlp.launch.py model_type:=medium # More accurate
```

## File Structure

### New Files
- `milestone6/nlp/whisper_publisher.py` - Whisper transcription service node
- `milestone6/nlp/whisper_subscriber.py` - Helper class for requesting transcriptions
- `launch/part3.base.launch.py` - TurtleBot-side launch file
- `launch/part3.nlp.launch.py` - Remote PC Whisper-only launch file

### Modified Files
- `milestone6/part3.py` - Refactored to use WhisperSubscriber instead of local WhisperBackend
- `milestone6/nlp/__init__.py` - Added new exports
- `setup.py` - Added whisper_publisher executable

## Topics

- `/audio_command` (std_msgs/String) - Audio file paths (TurtleBot → Remote PC)
- `/voice_transcription` (std_msgs/String) - Transcribed text (Remote PC → TurtleBot)

## Parameters

### part3.base.launch.py
All existing Part 3 parameters plus:
- `use_whisper` (bool, default: true) - Enable Whisper subscriber mode

### part3.nlp.launch.py
- `device` (string, default: 'cpu') - Whisper device (cpu/cuda)
- `model_type` (string, default: 'small') - Whisper model size
- `temperature` (float, default: 0.0) - Sampling temperature

## Network Configuration

Both machines must be on the same ROS 2 network. Configure ROS_DOMAIN_ID:

```bash
# On both TurtleBot and Remote PC
export ROS_DOMAIN_ID=42  # Use same ID on both machines
```

For cross-machine communication, ensure:
1. Both machines can ping each other
2. Firewall allows UDP multicast traffic
3. ROS 2 discovery is working (`ros2 topic list` shows topics from both machines)

## Benefits

1. **Performance**: Separates CPU-intensive Whisper from TurtleBot Jetson
2. **Scalability**: Can use powerful GPU on remote PC for faster transcription
3. **Flexibility**: Easy to upgrade Whisper model without affecting TurtleBot
4. **Reliability**: TurtleBot focuses on real-time control and perception
