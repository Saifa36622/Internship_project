import os
import pyaudio
import wave
from datetime import datetime
from google.cloud import speech_v1p1beta1 as speech

class AudioProcessor:
    def __init__(self):
        # Initialize PyAudio
        self.p = pyaudio.PyAudio()
        self.chunk = 2048  # Increased chunk size for better USB handling
        self.sample_format = pyaudio.paInt16  # 16 bits per sample
        self.channels = 1  # Single channel for microphone
        self.fs = 48000  # Default sample rate for USB audio
        # Sample rates optimized for USB audio devices
        self.sample_rates = [48000, 16000, 32000, 8000]
        
        # Try to find SR-MV2000W device
        self.default_device = self.find_device("SR-MV2000W")
        
    def find_device(self, name):
        """Find a specific audio device by name"""
        info = self.p.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        
        for i in range(0, numdevices):
            try:
                device_info = self.p.get_device_info_by_host_api_device_index(0, i)
                if name.lower() in device_info.get('name', '').lower():
                    return i
            except:
                continue
        return None

    def list_microphones(self):
        """List all available microphones and their supported sample rates"""
        print("\nAvailable Microphones:")
        info = self.p.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')
        
        for i in range(0, numdevices):
            try:
                device_info = self.p.get_device_info_by_host_api_device_index(0, i)
                if device_info.get('maxInputChannels') > 0:
                    print(f"\nIndex {i}: {device_info.get('name')}")
                    print(f"Default Sample Rate: {int(device_info.get('defaultSampleRate'))} Hz")
                    print(f"Max Input Channels: {device_info.get('maxInputChannels')}")
                    if i == self.default_device:
                        print("*** This is your SR-MV2000W device ***")
            except Exception as e:
                continue
        print()
        return numdevices
        
    def record_audio(self, seconds=5, device_index=None):
        """Record audio from specified device for given duration"""
        print(f"Recording for {seconds} seconds...")
        
        for rate in self.sample_rates:
            try:
                self.fs = rate
                print(f"Trying sample rate: {rate} Hz...")
                
                stream = self.p.open(format=self.sample_format,
                                   channels=self.channels,
                                   rate=self.fs,
                                   frames_per_buffer=self.chunk,
                                   input=True,
                                   input_device_index=device_index)
                
                print(f"Successfully opened stream with sample rate: {rate} Hz")
                frames = []
                
                for i in range(0, int(self.fs / self.chunk * seconds)):
                    data = stream.read(self.chunk, exception_on_overflow=False)
                    frames.append(data)
                    
                stream.stop_stream()
                stream.close()
                
                return frames
                
            except Exception as e:
                print(f"Failed with sample rate {rate} Hz: {e}")
                continue
                
        print("Error: Could not find a working sample rate")
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
    
    def cleanup(self):
        """Cleanup PyAudio"""
        self.p.terminate()

def transcribe_with_google(file_path, sample_rate_hertz):
    """
    Transcribes an audio file using Google Cloud Speech-to-Text API.
    
    Args:
        file_path (str): The path to the audio file.
        sample_rate_hertz (int): The sample rate of the audio.
    
    Returns:
        str: The transcribed text.
    """
    try:
        client = speech.SpeechClient()
        
        with open(file_path, "rb") as audio_file:
            content = audio_file.read()

        audio = speech.RecognitionAudio(content=content)
        
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=sample_rate_hertz,
            language_code='en-US',
            alternative_language_codes=['th-TH'],
            model='default', 
            enable_automatic_punctuation=True,
            use_enhanced=True
        )
        
        print("Transcribing audio with Google Cloud Speech-to-Text...")
        response = client.recognize(config=config, audio=audio)

        transcript = ""
        for result in response.results:
            transcript += result.alternatives[0].transcript
        
        return transcript
    
    except Exception as e:
        return f"An error occurred: {e}"

def main():
    try:
        processor = AudioProcessor()
    except Exception as e:
        print(f"Failed to initialize: {e}")
        return
    
    num_devices = processor.list_microphones()
    
    if num_devices == 0:
        print("No microphones found!")
        return
    
    if processor.default_device is not None:
        device_index = processor.default_device
        print(f"Using SR-MV2000W device (index {device_index})")
    else:
        while True:
            try:
                device_index = int(input("Enter the index number of the microphone to use: "))
                if 0 <= device_index < num_devices:
                    break
                else:
                    print(f"Please enter a number between 0 and {num_devices-1}")
            except ValueError:
                print("Please enter a valid number")
    
    try:
        while True:
            input("Press Enter to start recording (or Ctrl+C to exit)...")
            
            frames = processor.record_audio(5, device_index)
            
            if frames:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"recording_{timestamp}.wav"
                
                if processor.save_audio(frames, filename):
                    print(f"Audio saved as {filename}")
                    
                    # Transcribe using Google's API
                    transcribed_text = transcribe_with_google(filename, processor.fs)
                    
                    if not transcribed_text.startswith("An error"):
                        print("-" * 30)
                        print("Transcription successful!")
                        print(f"Transcription: {transcribed_text}")
                    else:
                        print(transcribed_text)
                    
                    os.remove(filename)
                    print(f"Removed temporary file {filename}")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        processor.cleanup()

if __name__ == "__main__":
    main()
