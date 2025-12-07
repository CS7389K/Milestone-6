import subprocess


class EspeakBackend():

    def __call__(self, text: str)  -> bool:
        try:
            subprocess.run(
                ["espeak", "-s", "140", text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=True
            )
            return True
        except subprocess.CalledProcessError:
            return False