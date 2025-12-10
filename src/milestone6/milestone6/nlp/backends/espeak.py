import subprocess


class EspeakBackend():

    def __init__(self, speech_rate: int = 140):
        """
        Initialize Espeak backend.

        Args:
            speech_rate: Words per minute (default: 140)
        """
        self.speech_rate = speech_rate

    def __call__(self, text: str) -> bool:
        try:
            subprocess.run(
                ["espeak", "-s", str(self.speech_rate), text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True
            )
            return True
        except subprocess.CalledProcessError:
            return False
