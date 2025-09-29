import pyaudio
import wave
import whisper
from datetime import datetime
import os

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

        # Try different sample rates until one works
        for rate in self.sample_rates:
            try:
                self.fs = rate
                print(f"Trying sample rate: {rate} Hz...")

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

def transcribe_local_audio(file_path, model_size="base",language=None):
    """
    Transcribes an audio file locally using the Whisper model.

    Args:
        file_path (str): The path to the audio file.
        model_size (str): The size of the model to use (e.g., "tiny", "base", "small", "medium", "large").

    Returns:
        str: The transcribed text.
    """
    try:
        print(f"Loading Whisper model '{model_size}'...")
        model = whisper.load_model(model_size)
        print("Transcribing audio...")
        result = model.transcribe(file_path,language=language)
        return result["text"]
    except Exception as e:
        return f"An error occurred: {e}"

def main():
    try:
        processor = AudioProcessor()
    except Exception as e:
        print(f"Failed to initialize: {e}")
        return

    # List available microphones
    num_devices = processor.list_microphones()

    if num_devices == 0:
        print("No microphones found!")
        return

    # Use SR-MV2000W device if found, otherwise ask user
    if processor.default_device is not None:
        device_index = processor.default_device
        print(f"Using SR-MV2000W device (index {device_index})")
    else:
        # Ask user which microphone to use
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
            
            # Record audio
            frames = processor.record_audio(5, device_index)  # Record for 5 seconds

            if frames:
                # Generate unique filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"recording_{timestamp}.wav"

                # Save audio
                if processor.save_audio(frames, filename):
                    print(f"Audio saved as {filename}")

                    # Transcribe audio using local Whisper
                    transcribed_text = transcribe_local_audio(filename, model_size="base",language="en")
                    if not transcribed_text.startswith("An error"):
                        print("-" * 30)
                        print("Transcription successful!")
                        print(f"Transcription: {transcribed_text}")
                    else:
                        print(transcribed_text)
                    
                    # Optional: Clean up the temporary audio file
                    os.remove(filename)
                    print(f"Removed temporary file {filename}")
    
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        processor.cleanup()

if __name__ == "__main__":
    main()