from collections import defaultdict


class MappingTracker:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(MappingTracker, cls).__new__(cls)
            cls._instance.__initialized = False
        return cls._instance

    def __init__(self):
        if not self.__initialized:
            self.curr_frame_idx = 0
            self.curr_object_count = 0
            self.total_detections = 0
            self.total_objects = 0
            self.total_merges = 0
            self.merge_list = []
            self.object_dict = {}
            self.curr_class_count = defaultdict(int)
            self.total_object_count = 0
            self.prev_obj_names = []
            self.prev_bbox_names = []
            self.brand_new_counter = 0

    def increment_total_detections(self, count):
        self.total_detections += count

    def get_total_detections(self):
        return self.total_detections

    def set_total_detections(self, count):
        self.total_detections = count

    def increment_total_detections(self, count):
        self.total_detections += count

    def get_total_operations(self):
        return self.total_operations

    def set_total_operations(self, count):
        self.total_operations = count

    def increment_total_operations(self, count):
        self.total_operations += count

    def get_total_objects(self):
        return self.total_objects

    def set_total_objects(self, count):
        self.total_objects = count

    def increment_total_objects(self, count):
        self.total_objects += count

    def track_merge(self, obj1, obj2):
        self.total_merges += 1
        self.merge_list.append((obj1, obj2))

    def increment_total_merges(self, count):
        self.total_merges += count
