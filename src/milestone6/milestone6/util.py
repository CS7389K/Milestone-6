import sounddevice as sd
from scipy.io.wavfile import write
from typing import Optional


def record_audio_and_saveas(
        file_name,
        duration_seconds : int = 5,
        sample_rate : int = 16000
    ):
    audio = sd.rec(
        int(duration_seconds * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype="int16"
    )
    sd.wait()
    write(file_name, sample_rate, audio)


def get_user_input() -> Optional[str]:
    text = input()
    if text.lower() in ("exit", "quit"):
        return None

    return text


def prompt_assistant(
        client,
        prompt : str
    ) -> Optional[str]:
    response = client("llama", prompt).strip()
    print("Prompt:\n", prompt, "\n")
    print("Response:\n", response, "\n")
    client("espeak", response)