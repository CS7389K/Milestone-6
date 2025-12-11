from pathlib import Path
from typing import Union

from llama_cpp import Llama


class LlamaBackend():

    _MODEL_CHAT_PATH = "/home/nvidia/llama.cpp/models/llama-2-7b-chat.Q4_K_M.gguf"
    _MODEL_INSTRUCT_PATH = "/home/nvidia/llama.cpp/models/llama-2-7b-instruct-q4_0.gguf"

    def __init__(
        self,
        model_path: Union[Path, str] = "",
        instruct: bool = True,
        n_ctx: int = 512,
        n_threads: int = 4,
        n_gpu_layers: int = 33,
        seed=42,
        use_mlock: bool = False,
        verbose: bool = True,
    ):

        if len(model_path) == 0:
            if instruct:
                model_path = Path(self._MODEL_INSTRUCT_PATH)
            else:
                model_path = Path(self._MODEL_CHAT_PATH)
        else:
            if isinstance(model_path, str):
                model_path = Path(model_path)

        print(f"Loading model: {str(model_path.resolve())}")

        # Verify file exists before loading
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        # Verify file is readable and non-empty
        if model_path.stat().st_size == 0:
            print(f"Model file size: {model_path.stat().st_size} bytes")

        print(f"Loading model from: {model_path}")
        print(f"File size: {model_path.stat().st_size / (1024**3):.2f} GB")
        print(
            f"Parameters: n_ctx={n_ctx}, n_threads={n_threads}, n_gpu_layers={n_gpu_layers}")

        self.llm = Llama(
            model_path=str(model_path),
            n_ctx=n_ctx,
            n_threads=n_threads,
            n_gpu_layers=n_gpu_layers,
            seed=seed,
            use_mlock=use_mlock,
            verbose=verbose,
        )

    def __call__(
        self,
        prompt: str,
        max_tokens: int = 128,
        temperature: float = 0.7,
        top_p: float = 0.95,
        stream: bool = True,
        stop: list = None
    ) -> str:
        if stop is None:
            # Default stop tokens optimized for action translation
            stop = ["\n", "</s>"]

        chunks = self.llm(
            prompt,
            max_tokens=max_tokens,
            temperature=temperature,
            top_p=top_p,
            stream=stream,
            stop=stop
        )
        response = "".join([c["choices"][0]["text"] for c in chunks])
        # Strip whitespace only (don't replace newlines since we stop on them)
        response = response.strip()
        return response
