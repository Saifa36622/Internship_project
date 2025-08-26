import pyaudio
import wave

# Settings
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024
RECORD_SECONDS = 5
OUTPUT_FILENAME = "virtualmic_record.wav"
TARGET_NAME = "Virtual-Mic"

# Initialize
p = pyaudio.PyAudio()

# List devices and find the target mic index
target_index = None
info = p.get_host_api_info_by_index(0)
num_devices = info.get('deviceCount')
for i in range(num_devices):
    dev = p.get_device_info_by_host_api_device_index(0, i)
    if dev.get('maxInputChannels', 0) > 0:
        name = dev.get('name', '')
        print(f"Input Device id {i} – {name}")
        if TARGET_NAME in name:
            target_index = i

if target_index is None:
    raise ValueError(f"Could not find mic named '{TARGET_NAME}'")

print(f" → Using device ID {target_index} for recording")

# Open stream with selected device
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK,
                input_device_index=target_index)

print("* recording")
frames = []
for _ in range(int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)
print("* done recording")

stream.stop_stream()
stream.close()
p.terminate()

# Write to WAV file
with wave.open(OUTPUT_FILENAME, 'wb') as wf:
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))

print(f"Saved audio to {OUTPUT_FILENAME}")
