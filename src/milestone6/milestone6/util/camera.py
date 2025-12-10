
import cv2


class CameraCV2:
    """Camera capture using OpenCV VideoCapture (V4L2 or default)."""

    def __init__(self, device=1, width=640, height=480):
        self.device = device
        self.width = width
        self.height = height
        self._cap = None

    def open(self):
        self._cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise RuntimeError(f"Unable to open camera device {self.device}")
        self._cap.set(cv2.CAP_PROP_FOURCC,
                      cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def read(self):
        if self._cap is None:
            raise RuntimeError("Camera not opened")
        return self._cap.read()

    def isOpened(self):
        return self._cap is not None and self._cap.isOpened()

    def release(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None


class CameraGStreamer:
    """Camera capture using a GStreamer pipeline."""

    def __init__(self, pipeline: str, width=640, height=480):
        self.pipeline = pipeline
        self.width = width
        self.height = height
        self._cap = None

    def open(self):
        self._cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Unable to open GStreamer pipeline: {self.pipeline}")

    def read(self):
        if self._cap is None:
            raise RuntimeError("Camera not opened")
        return self._cap.read()

    def isOpened(self):
        return self._cap is not None and self._cap.isOpened()

    def release(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
