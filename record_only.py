#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import threading
import tempfile
import os
import time

class SmartAudioRecorderNode(Node):
    def __init__(self):
        super().__init__('smart_audio_recorder_node')
        self.get_logger().info("Smart Audio Recorder Node started.")

        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.is_recording = False
        self.is_monitoring = False
        
        # Sound detection parameters
        self.silence_threshold = 500  # Adjust as needed 
        self.recording_duration = 5   # Record for 5 seconds after sound is detected
        self.transcription_publisher = self.create_publisher(String, '/gemini/context', 10)
        # Start monitoring in a separate thread
        self.monitoring_thread = threading.Thread(target=self.monitor_audio, daemon=True)
        self.monitoring_thread.start()

    def monitor_audio(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=self.format,
                       channels=self.channels,
                       rate=self.rate,
                       input=True,
                       frames_per_buffer=self.chunk)
        
        self.is_monitoring = True
        self.get_logger().info("Monitoring for sound...")
        
        try:
            while self.is_monitoring:
                data = stream.read(self.chunk, exception_on_overflow=False)
                # Convert chunk to int16 to analyze volume
                audio_data = wave.struct.unpack_from("<" + "h" * self.chunk, data)
                
                # Simple volume check (RMS)
                rms = (sum(x**2 for x in audio_data) / len(audio_data))**0.5
                
                if rms > self.silence_threshold and not self.is_recording:
                    self.get_logger().info(f"Sound detected! RMS: {rms:.2f}. Starting recording...")
                    self.record_audio_and_save(stream)
                    self.transcription_publisher.publish(String(data="lift up the cup"))

                    self.get_logger().info("Monitoring for sound again...")
        except KeyboardInterrupt:
            self.get_logger().info("Stopping monitoring...")
        finally:
            stream.stop_stream()
            stream.close()
            p.terminate()

    def record_audio_and_save(self, stream):
        self.is_recording = True
        frames = []
        
        # Record for the specified duration
        for _ in range(0, int(self.rate / self.chunk * self.recording_duration)):
            data = stream.read(self.chunk, exception_on_overflow=False)
            frames.append(data)
        
        # Reset recording flag
        self.is_recording = False
        
        # Save the recorded data to a wave file
        temp_file_path = tempfile.NamedTemporaryFile(delete=False, suffix='.wav').name
        wf = wave.open(temp_file_path, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(pyaudio.PyAudio().get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        self.get_logger().info(f"Audio recorded and saved to: {temp_file_path}")
        
    def destroy_node(self):
        self.is_monitoring = False
        self.monitoring_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    smart_audio_recorder_node = SmartAudioRecorderNode()
    try:
        rclpy.spin(smart_audio_recorder_node)
    except KeyboardInterrupt:
        pass
    finally:
        smart_audio_recorder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()