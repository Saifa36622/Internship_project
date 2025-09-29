import pyaudio
import wave
from datetime import datetime
import importlib.util
import sys

# Initialize speech recognition
try:
    import speech_recognition as sr
    SPEECH_RECOGNITION_ENABLED = True
    print("Speech recognition initialized successfully")
except ImportError:
    print("Speech recognition not available. Installing required packages...")
    print("Please run:")
    print("pip install SpeechRecognition")
    SPEECH_RECOGNITION_ENABLED = False

class SpeechToText:
    def __init__(self):
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.chunk = 1024  # Record in chunks of 1024 samples
        self.sample_format = pyaudio.paInt16  # 16 bits per sample
        self.channels = 1  # Single channel for microphone
        self.fs = 48000  # Record at 48000 samples per second (more widely supported)
        
        # Initialize speech recognition
        global SPEECH_RECOGNITION_ENABLED
        self.recognizer = None
        if SPEECH_RECOGNITION_ENABLED:
            try:
                self.recognizer = sr.Recognizer()
                print("Speech recognizer initialized successfully")
            except Exception as e:
                print(f"Error initializing speech recognition: {e}")
                SPEECH_RECOGNITION_ENABLED = False
            
    def list_microphones(self):
        """List all available microphones"""
        print("\nAvailable Microphones:")
        info = self.p.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        
        for i in range(0, numdevices):
            device_info = self.p.get_device_info_by_host_api_device_index(0, i)
            if device_info.get('maxInputChannels') > 0:
                print(f"Index {i}: {device_info.get('name')}")
        print()
        return numdevices
        
    def record_audio(self, seconds=5, device_index=None):
        """Record audio from specified device for given duration"""
        print(f"Recording for {seconds} seconds...")
        
        # List of commonly supported sample rates to try
        sample_rates = [48000, 44100, 16000, 8000]
        
        for rate in sample_rates:
            try:
                self.fs = rate
                # Try to open stream with current sample rate
                stream = self.p.open(format=self.sample_format,
                                   channels=self.channels,
                                   rate=self.fs,
                                   frames_per_buffer=self.chunk,
                                   input=True,
                                   input_device_index=device_index)
                
                print(f"Successfully opened stream with sample rate: {rate} Hz")
                
                frames = []  # Initialize array to store frames
                
                # Store data in chunks for specified number of seconds
                for i in range(0, int(self.fs / self.chunk * seconds)):
                    data = stream.read(self.chunk, exception_on_overflow=False)
                    frames.append(data)
                    
                # Stop and close the stream 
                stream.stop_stream()
                stream.close()
                
                return frames
                
            except Exception as e:
                print(f"Failed to record with sample rate {rate} Hz: {e}")
                # Try next sample rate
                continue
                
        print("Error: Failed to record with any sample rate")
        return None
    
    def save_audio(self, frames, filename):
        """Save the recorded audio to a WAV file"""
        if not frames:
            print("No audio data to save")
            return False
            
        try:
            wf = wave.open(filename, 'wb')
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.p.get_sample_size(self.sample_format))
            wf.setframerate(self.fs)
            wf.writeframes(b''.join(frames))
            wf.close()
            return True
        except Exception as e:
            print(f"Error saving audio: {e}")
            return False
    
    def transcribe_audio(self, audio_file):
        """Transcribe audio file to text using Google Speech Recognition"""
        if not SPEECH_RECOGNITION_ENABLED or self.recognizer is None:
            print("Speech recognition is not available. Please install required packages:")
            print("pip install SpeechRecognition")
            return None
            
        try:
            print(f"Loading audio file: {audio_file}")
            with sr.AudioFile(audio_file) as source:
                print("Reading audio data...")
                audio_data = self.recognizer.record(source)
                print("Sending to Google Speech Recognition...")
                text = self.recognizer.recognize_google(audio_data)
                print("Successfully received transcription")
                return text
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service: {e}")
            return None
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand the audio")
            return None
        except Exception as e:
            print(f"Error transcribing audio: {e}")
            return None
    
    def cleanup(self):
        """Cleanup PyAudio"""
        self.p.terminate()

def main():
    stt = SpeechToText()
    
    # List available microphones
    num_devices = stt.list_microphones()
    
    if num_devices == 0:
        print("No microphones found!")
        return
        
    # Ask user which microphone to use
    while True:
        try:
            device_index = int(input("Enter the index number of the microphone you want to use: "))
            if 0 <= device_index < num_devices:
                break
            else:
                print(f"Please enter a number between 0 and {num_devices-1}")
        except ValueError:
            print("Please enter a valid number")
    
    try:
        while True:
            input("Press Enter to start recording (or Ctrl+C to exit)...")
            
            # Record audio
            frames = stt.record_audio(5, device_index)  # Record for 5 seconds
            if not frames:
                continue
                
            # Generate unique filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.wav"
            
            # Save audio
            if stt.save_audio(frames, filename):
                print(f"Audio saved as {filename}")
                
                # Transcribe audio
                if SPEECH_RECOGNITION_ENABLED:
                    print("Transcribing...")
                    text = stt.transcribe_audio(filename)
                    if text:
                        print(f"Transcription: {text}")
                    else:
                        print("Transcription failed")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        stt.cleanup()

if __name__ == "__main__":
    main()