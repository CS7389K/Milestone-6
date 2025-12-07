from dataclasses import dataclass


@dataclass
class YOLOData:
    """Class representing data from a YOLO model."""
    bbox_x: float
    bbox_y: float
    bbox_w: float
    bbox_h: float
    clz: int