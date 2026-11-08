import json
import logging
from pathlib import Path
import supervision as sv
import numpy as np
import time


def mask_iou(mask1, mask2):
    intersection = np.logical_and(mask1, mask2).sum()
    union = np.logical_or(mask1, mask2).sum()
    if union == 0:
        return 0
    return intersection / union


def filter_detections(
    image,
    detections: sv.Detections,
    classes,
    top_x_detections=None,
    confidence_threshold: float = 0.0,
    given_labels=None,
    iou_threshold: float = 0.8,
    proximity_threshold: float = 20.0,
    keep_larger: bool = True,
    min_mask_size_ratio=0.00025,
) -> tuple[sv.Detections, list[str]]:
    if not (
        hasattr(detections, "confidence")
        and hasattr(detections, "class_id")
        and hasattr(detections, "xyxy")
    ):
        print("Detections object is missing required attributes.")
        return (detections, [])
    detections_combined = sorted(
        zip(
            detections.confidence,
            detections.class_id,
            detections.xyxy,
            detections.mask,
            range(len(given_labels)),
        ),
        key=lambda x: x[0],
        reverse=True,
    )
    if top_x_detections is not None:
        detections_combined = detections_combined[:top_x_detections]
    total_pixels = image.shape[0] * image.shape[1]
    small_mask_size = total_pixels * min_mask_size_ratio
    filtered_detections = []
    for idx, current_det in enumerate(detections_combined):
        conf, curr_class_id, curr_xyxy, curr_mask, _ = current_det
        curr_center = (
            (curr_xyxy[0] + curr_xyxy[2]) / 2,
            (curr_xyxy[1] + curr_xyxy[3]) / 2,
        )
        curr_area = (curr_xyxy[2] - curr_xyxy[0]) * (curr_xyxy[3] - curr_xyxy[1])
        keep = True
        mask_size = np.count_nonzero(current_det[3])
        if mask_size < small_mask_size:
            logging.debug(
                f"Removing {classes.get_classes_arr()[curr_class_id]} because the mask size is too small."
            )
            keep = False
            continue
        if conf < confidence_threshold:
            logging.debug(
                f"Removing {classes.get_classes_arr()[curr_class_id]} because it has a confidence of {conf} < {confidence_threshold}."
            )
            keep = False
            continue
        for other in filtered_detections:
            _, other_class_id, other_xyxy, other_mask, _ = other
            if mask_iou(curr_mask, other_mask) > iou_threshold:
                keep = False
                logging.debug(
                    f"Removing {classes.get_classes_arr()[curr_class_id]} because it has an IoU of {mask_iou(curr_mask, other_mask)} with object {classes.get_classes_arr()[other_class_id]}."
                )
                break
            other_center = (
                (other_xyxy[0] + other_xyxy[2]) / 2,
                (other_xyxy[1] + other_xyxy[3]) / 2,
            )
            other_area = (other_xyxy[2] - other_xyxy[0]) * (
                other_xyxy[3] - other_xyxy[1]
            )
            dist = np.sqrt(
                (curr_center[0] - other_center[0]) ** 2
                + (curr_center[1] - other_center[1]) ** 2
            )
            if dist < proximity_threshold:
                if (
                    keep_larger
                    and curr_area > other_area
                    or (not keep_larger and curr_area < other_area)
                ):
                    filtered_detections.remove(other)
                else:
                    keep = False
                    break
        if classes.get_classes_arr()[curr_class_id] in classes.bg_classes:
            logging.debug(
                f"Removing {classes.get_classes_arr()[curr_class_id]} because it is a background class, specifically {classes.bg_classes}."
            )
            keep = False
            continue
        if keep:
            filtered_detections.append(current_det)
    if len(filtered_detections) == 0:
        return (None, None)
    confidences, class_ids, xyxy, masks, indices = zip(*filtered_detections)
    filtered_labels = [given_labels[i] for i in indices]
    filtered_detections = sv.Detections(
        class_id=np.array(class_ids, dtype=np.int64),
        confidence=np.array(confidences, dtype=np.float32),
        xyxy=np.array(xyxy, dtype=np.float32),
        mask=np.array(masks, dtype=np.bool_),
    )
    return (filtered_detections, filtered_labels)


def measure_time(func):

    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        elapsed_time = end_time - start_time
        logging.info(
            f"Done! Execution time of {func.__name__} function: {elapsed_time:.2f} seconds"
        )
        return result

    return wrapper


class ObjectClasses:

    def __init__(self, classes_file_path, bg_classes, skip_bg, class_set):
        self.classes_file_path = Path(classes_file_path)
        self.bg_classes = bg_classes
        self.skip_bg = skip_bg
        assert class_set in [
            "hm3d",
            "scannet200",
            "yolo_finetune",
        ], f"Invalid class set: {class_set}"
        self.class_set = class_set
        self.classes = self._load_or_create_colors()

    def _load_or_create_colors(self):
        if self.class_set == "hm3d":
            with open(self.classes_file_path, "r") as f:
                all_lines = [cls.strip() for cls in f.readlines()][1:]
                all_classes = [
                    line.split(",")[2].replace('"', "") for line in all_lines
                ]
                all_classes = list(set(all_classes))
                all_classes = [cls for cls in all_classes if cls != "unknown"]
                logging.info(
                    f"Loaded {len(all_classes)} classes from hm3d: {self.classes_file_path}!!!"
                )
            if self.skip_bg:
                all_classes = [cls for cls in all_classes if cls not in self.bg_classes]
        elif self.class_set == "scannet200":
            self.classes_file_path = Path("data/scannet200_classes.txt")
            with open(self.classes_file_path, "r") as f:
                all_lines = [cls.strip() for cls in f.readlines()]
                all_classes = list(set(all_lines))
                all_classes = [cls for cls in all_classes if cls != "unknown"]
                logging.info(
                    f"Loaded {len(all_classes)} classes from scannet 200: {self.classes_file_path}!!!"
                )
        else:
            self.classes_file_path = "yolo_finetune/class_id_to_class_name.json"
            class_id_to_class_name = json.load(open(self.classes_file_path, "r"))
            all_classes = list(class_id_to_class_name.values())
            logging.info(
                f"Loaded {len(all_classes)} classes from yolo finetune class: {self.classes_file_path}!!!"
            )
        return all_classes

    def get_classes_arr(self):
        return self.classes

    def get_bg_classes_arr(self):
        return self.bg_classes
