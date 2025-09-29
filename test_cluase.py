import whisper
import pyaudio
import wave
import tempfile
import os
import threading
import time
from typing import Optional, Callable

class WhisperSpeechToText:
    def __init__(self, model_size: str = "large"):
        """
        Initialize Whisper Speech-to-Text
        
        Model sizes (larger = more accurate but slower):
        - tiny: fastest, least accurate
        - base: good balance (recommended for most use cases)
        - small: better accuracy
        - medium: even better accuracy  
        - large: best accuracy, slowest
        """
        print(f"Loading Whisper {model_size} model...")
        self.model = whisper.load_model(model_size)
        print("Model loaded successfully!")
        
        # Audio recording parameters
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        
        self.is_recording = False
        self.audio_data = []
        
    def transcribe_file(self, audio_file_path: str, language: Optional[str] = None) -> dict:
        """
        Transcribe an audio file
        
        Args:
            audio_file_path: Path to audio file
            language: Language code ('th' for Thai, 'en' for English, None for auto-detect)
        
        Returns:
            Dictionary with transcription results
        """
        print(f"Transcribing: {audio_file_path}")
        
        # Transcribe with language specification
        if language:
            result = self.model.transcribe(audio_file_path, language=language)
        else:
            # Auto-detect language (works well for mixed Thai/English)
            result = self.model.transcribe(audio_file_path)
        
        return result
    
    def record_audio(self, duration: int = 5) -> str:
        """
        Record audio for specified duration
        
        Args:
            duration: Recording duration in seconds
            
        Returns:
            Path to recorded audio file
        """
        print(f"Recording for {duration} seconds...")
        
        # Initialize PyAudio
        p = pyaudio.PyAudio()
        
        # Open stream
        stream = p.open(format=self.format,
                       channels=self.channels,
                       rate=self.rate,
                       input=True,
                       frames_per_buffer=self.chunk)
        
        frames = []
        
        # Record audio
        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)
        
        print("Recording finished!")
        
        # Stop and close stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # Save to temporary file
        temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
        wf = wave.open(temp_file.name, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        return temp_file.name
    
    def start_continuous_recording(self, callback: Callable[[str], None], 
                                 chunk_duration: int = 5):
        """
        Start continuous recording and transcription
        
        Args:
            callback: Function to call with transcription results
            chunk_duration: Duration of each recording chunk in seconds
        """
        self.is_recording = True
        
        def record_loop():
            p = pyaudio.PyAudio()
            stream = p.open(format=self.format,
                           channels=self.channels,
                           rate=self.rate,
                           input=True,
                           frames_per_buffer=self.chunk)
            
            while self.is_recording:
                frames = []
                # Record chunk
                for _ in range(0, int(self.rate / self.chunk * chunk_duration)):
                    if not self.is_recording:
                        break
                    data = stream.read(self.chunk)
                    frames.append(data)
                
                if frames:  # If we recorded something
                    # Save to temporary file
                    temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
                    wf = wave.open(temp_file.name, 'wb')
                    wf.setnchannels(self.channels)
                    wf.setsampwidth(p.get_sample_size(self.format))
                    wf.setframerate(self.rate)
                    wf.writeframes(b''.join(frames))
                    wf.close()
                    
                    # Transcribe
                    try:
                        result = self.transcribe_file(temp_file.name)
                        if result['text'].strip():  # Only callback if there's actual text
                            callback(result['text'].strip())
                    except Exception as e:
                        print(f"Transcription error: {e}")
                    finally:
                        # Clean up temp file
                        os.unlink(temp_file.name)
            
            stream.stop_stream()
            stream.close()
            p.terminate()
        
        # Start recording in separate thread
        self.recording_thread = threading.Thread(target=record_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        print("Continuous recording started. Press Ctrl+C to stop.")
    
    def stop_continuous_recording(self):
        """Stop continuous recording"""
        self.is_recording = False
        print("Stopping continuous recording...")


def main():
    """Example usage"""
    # Initialize with base model (good balance of speed/accuracy)
    # Use "large" for best accuracy, "tiny" for fastest processing
    stt = WhisperSpeechToText(model_size="base")
    
    print("\n=== Whisper Speech-to-Text Demo ===")
    print("1. Record and transcribe")
    print("2. Transcribe existing file")
    print("3. Continuous recording")
    print("4. Exit")
    
    while True:
        choice = input("\nEnter your choice (1-4): ")
        
        if choice == '1':
            duration = int(input("Recording duration in seconds (default 5): ") or "5")
            
            # Record audio
            audio_file = stt.record_audio(duration)
            
            # Transcribe
            result = stt.transcribe_file(audio_file)
            
            print(f"\nDetected language: {result['language']}")
            print(f"Transcription: {result['text']}")
            
            # Show segments with timestamps (if available)
            if 'segments' in result:
                print("\nDetailed segments:")
                for segment in result['segments']:
                    start = segment['start']
                    end = segment['end']
                    text = segment['text']
                    print(f"[{start:.1f}s - {end:.1f}s]: {text}")
            
            # Clean up temp file
            os.unlink(audio_file)
        
        elif choice == '2':
            file_path = input("Enter path to audio file: ")
            if os.path.exists(file_path):
                result = stt.transcribe_file(file_path)
                print(f"\nDetected language: {result['language']}")
                print(f"Transcription: {result['text']}")
            else:
                print("File not found!")
        
        elif choice == '3':
            def transcription_callback(text):
                print(f"Transcribed: {text}")
            
            try:
                stt.start_continuous_recording(transcription_callback, chunk_duration=3)
                # Keep the main thread alive
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                stt.stop_continuous_recording()
                print("\nStopped continuous recording.")
        
        elif choice == '4':
            print("Goodbye!")
            break
        
        else:
            print("Invalid choice!")


if __name__ == "__main__":
    main()