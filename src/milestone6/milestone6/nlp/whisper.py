#!/usr/bin/env python3
"""
Whisper Publisher Node

Runs Whisper model and publishes transcriptions.
This node should run on a powerful machine (remote PC with good CPU/GPU).

Publishes to: /voice_transcription (std_msgs/String) - sends transcribed text
"""

import numpy as np
import rclpy
import sounddevice as sd
from rclpy.node import Node
from scipy.io.wavfile import write
from std_msgs.msg import String

from milestone6.nlp.backends.whisper import WhisperBackend


class WhisperPublisher(Node):
    """Publisher node for Whisper transcription service."""

    def __init__(self):
        super().__init__('whisper_publisher')

        # Declare parameters
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('model', 'tiny.en')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('timer_period', 1.0)
        self.declare_parameter('audio_sample_rate', 16000)
        self.declare_parameter('audio_duration', 5.0)
        self.declare_parameter('audio_threshold', 800)
        self.declare_parameter('audio_file', '/tmp/whisper_audio.wav')

        # Get parameters
        device = self.get_parameter('device').value
        model = self.get_parameter('model').value
        temperature = self.get_parameter('temperature').value
        timer_period = self.get_parameter('timer_period').value
        self.audio_sample_rate = self.get_parameter('audio_sample_rate').value
        self.audio_duration = self.get_parameter('audio_duration').value
        self.audio_threshold = self.get_parameter('audio_threshold').value
        self.audio_file = self.get_parameter('audio_file').value

        self.get_logger().info("Initializing Whisper Publisher...")
        self.get_logger().info(f"Model: {model}, Device: {device}")
        self.get_logger().info(f"Timer period: {timer_period}s")

        # Initialize Whisper backend
        self.whisper = WhisperBackend(
            model=model,
            device=device,
            temperature=temperature
        )

        # Create publisher for audio commands
        self.voice_transcription = self.create_publisher(
            String,
            '/voice_transcription',
            10
        )        

        # Create timer to periodically listen for audio
        self.timer = self.create_timer(timer_period, self._listen)

        self.get_logger().info("Whisper Publisher ready! Listening for audio...")

    def _listen(self):
        """
        Record audio using sounddevice when voice is detected and transcribe it.
        Waits for audio above threshold before starting recording.
        """
        try:
            self.get_logger().info("Listening for voice activity...")

            # Wait for voice activity (threshold detection)
            chunk_duration = 0.1  # Check every 100ms
            chunk_samples = int(self.audio_sample_rate * chunk_duration)

            # Listen for voice onset
            voice_detected = False
            max_wait_chunks = 100
            for _ in range(max_wait_chunks):
                chunk = sd.rec(
                    chunk_samples,
                    samplerate=self.audio_sample_rate,
                    channels=1,
                    dtype='int16'
                )
                sd.wait()

                # Check if amplitude exceeds threshold
                if np.max(np.abs(chunk)) > self.audio_threshold:
                    voice_detected = True
                    self.get_logger().info(
                        "Voice detected! Recording...")
                    break

            if not voice_detected:
                self.get_logger().debug(
                    "No voice detected within timeout")
                return

            # Record audio for specified duration
            audio = sd.rec(
                int(self.audio_duration * self.audio_sample_rate),
                samplerate=self.audio_sample_rate,
                channels=1,
                dtype='int16'
            )
            sd.wait()

            # Save to file
            write(self.audio_file, self.audio_sample_rate, audio)
            self.get_logger().info(
                f"Recording saved to {self.audio_file}")

            # Transcribe the audio
            self.get_logger().info("Transcribing...")
            text = self.whisper(self.audio_file).strip().lower()

            if text:
                # Publish transcribed text to voice_transcription topic
                msg = String()
                msg.data = text
                self.voice_transcription.publish(msg)
                self.get_logger().info(
                    f"Published to /voice_transcription: '{text}'")
            else:
                self.get_logger().warn("No speech detected in audio")

        except Exception as e:
            self.get_logger().error(f"Listen failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    publisher = WhisperPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info("Shutting down Whisper Publisher...")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
