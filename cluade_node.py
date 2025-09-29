#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import whisper
import pyaudio
import wave
import tempfile
import os
import threading
from typing import Optional

class WhisperSpeechToTextNode(Node):
    def __init__(self, model_size: str = "base"):
        # Initialize the ROS 2 Node
        super().__init__('whisper_stt_node')
        self.get_logger().info(f"Loading Whisper {model_size} model...")
        self.model = whisper.load_model(model_size)
        self.get_logger().info("Model loaded successfully!")
        
        # Create a publisher for the transcribed text
        self.transcription_publisher = self.create_publisher(String, '/gemini/context', 10)
        
        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.is_recording = False
        
        # Start continuous recording in a separate thread
        self.recording_thread = threading.Thread(target=self.record_loop)
        self.recording_thread.daemon = True
        self.is_recording = True
        self.recording_thread.start()
        self.get_logger().info("Continuous recording started.")
        
    def record_loop(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=self.format,
                       channels=self.channels,
                       rate=self.rate,
                       input=True,
                       frames_per_buffer=self.chunk)
        
        while self.is_recording:
            frames = []
            chunk_duration = 3  # Transcribe in 3-second chunks
            for _ in range(0, int(self.rate / self.chunk * chunk_duration)):
                if not self.is_recording:
                    break
                data = stream.read(self.chunk)
                frames.append(data)
            
            if frames:
                # Save to temporary file
                temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
                wf = wave.open(temp_file.name, 'wb')
                wf.setnchannels(self.channels)
                wf.setsampwidth(p.get_sample_size(self.format))
                wf.setframerate(self.rate)
                wf.writeframes(b''.join(frames))
                wf.close()
                
                # Transcribe and publish
                try:
                    result = self.transcribe_file(temp_file.name)
                    if result['text'].strip():
                        msg = String()
                        msg.data = result['text'].strip()
                        self.transcription_publisher.publish(msg)
                        self.get_logger().info(f'Published: "{msg.data}"')
                except Exception as e:
                    self.get_logger().error(f"Transcription error: {e}")
                finally:
                    os.unlink(temp_file.name)
        
        stream.stop_stream()
        stream.close()
        p.terminate()

    def transcribe_file(self, audio_file_path: str, language: Optional[str] = None) -> dict:
        """Transcribe an audio file using the Whisper model."""
        # This part of the logic remains the same
        if language:
            result = self.model.transcribe(audio_file_path, language=language)
        else:
            result = self.model.transcribe(audio_file_path)
        return result

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperSpeechToTextNode()
    try:
        rclpy.spin(whisper_node)
    except KeyboardInterrupt:
        pass
    finally:
        whisper_node.is_recording = False
        whisper_node.recording_thread.join()
        whisper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()