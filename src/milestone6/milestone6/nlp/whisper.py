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

from milestone6.milestone6.nlp.backends.whisper import WhisperBackend


class WhisperPublisher(Node):
    """Publisher node for Whisper transcription service."""

    def __init__(self):
        super().__init__('whisper_publisher')

        # Declare parameters
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('model_type', 'base.en')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('timer_period', 1.0)

        # Get parameters
        device = self.get_parameter('device').value
        model_type = self.get_parameter('model_type').value
        temperature = self.get_parameter('temperature').value
        timer_period = self.get_parameter('timer_period').value

        self.get_logger().info("Initializing Whisper Publisher...")
        self.get_logger().info(f"Model: {model_type}, Device: {device}")
        self.get_logger().info(f"Timer period: {timer_period}s")

        # Initialize Whisper backend
        self.whisper = WhisperBackend(
            model_type=model_type,
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

        self.get_logger().info("Whisper Publisher ready! Waiting for audio files...")

    def _listen(self) -> str:
        """
        Record audio using sounddevice when voice is detected and request transcription.
        Waits for audio above threshold before starting recording.
        Returns transcribed text (lowercase) or empty string if Whisper disabled.
        """
        if not self.use_whisper or self.whisper_sub is None:
            self.get_logger().warn("Whisper disabled - cannot listen for commands")
            return ""

        try:
            self.get_logger().info("[WHISPER] Listening for voice activity...")

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
                        "[WHISPER] Voice detected! Recording...")
                    break

            if not voice_detected:
                self.get_logger().warn(
                    "[WHISPER] No voice detected within timeout")
                return ""

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
                f"[WHISPER] Recording saved to {self.audio_file}")

            # Request transcription from Whisper publisher
            self.get_logger().info("[WHISPER] Requesting transcription...")
            self.whisper_sub.request_transcription(self.audio_file)

            # Wait for transcription response (with timeout)
            timeout_counter = 0
            max_wait = 100  # 10 seconds at 10Hz tick rate
            while self.whisper_sub.is_waiting() and timeout_counter < max_wait:
                rclpy.spin_once(self, timeout_sec=0.1)
                timeout_counter += 1

            if timeout_counter >= max_wait:
                self.get_logger().error("[WHISPER] Transcription timeout!")
                return ""

            # Get transcription
            text = self.whisper_sub.get_last_transcription().lower()

            # Publish transcribed text to voice_transcription topic
            msg = String()
            msg.data = text
            self.voice_transcription_pub.publish(msg)
            self.get_logger().info(
                f"[WHISPER] Published to /voice_transcription: '{text}'")

        except Exception as e:
            self.get_logger().error(f"Listen failed: {e}")
            return ""


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
