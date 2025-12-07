import whisper


class WhisperBackend():

    def __init__(
            self,
            model_type : str = "small",
            device : str = "cpu",
            temperature : float = 0.0
        ):
        # Initialize Whisper model
        self.whisper = whisper.load_model(model_type, device=device)
        self.temperature = temperature

    def __call__(
            self,
            file_name: str
        ) -> str:
        result = self.whisper.transcribe(
            file_name,
            temperature=self.temperature
        )
        return result["text"].strip()