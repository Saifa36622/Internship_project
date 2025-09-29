import whisper

def transcribe_local_audio(file_path, model_size="base"):
    """
    Transcribes an audio file locally using the Whisper model.

    Args:
        file_path (str): The path to the audio file.
        model_size (str): The size of the model to use (e.g., "tiny", "base", "small", "medium", "large").

    Returns:
        str: The transcribed text.
    """
    try:
        model = whisper.load_model(model_size)
        result = model.transcribe(file_path)
        return result["text"]
    except Exception as e:
        return f"An error occurred: {e}"

if __name__ == "__main__":
    audio_file_path = "/home/saifa/main_ws/recording_20250916_232421.wav"  # Replace with your audio file path

    transcribed_text = transcribe_local_audio(audio_file_path, model_size="base")
    if not transcribed_text.startswith("An error"):
        print("Transcription successful!")
        print("-" * 30)
        print(transcribed_text)
    else:
        print(transcribed_text)