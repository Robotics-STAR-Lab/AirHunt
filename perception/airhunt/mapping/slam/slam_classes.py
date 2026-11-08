import copy
import torch
import torch.nn.functional as F
import numpy as np
import open3d as o3d


def to_numpy(tensor):
    if isinstance(tensor, np.ndarray):
        return tensor
    return tensor.detach().cpu().numpy()


def to_tensor(numpy_array, device=None):
    if isinstance(numpy_array, torch.Tensor):
        return numpy_array
    if device is None:
        return torch.from_numpy(numpy_array)
    else:
        return torch.from_numpy(numpy_array).to(device)


class DetectionDict(dict):

    def get_values(self, key, idx: int = None):
        if idx is None:
            return [detection[key] for detection in self.values()]
        else:
            return [detection[key][idx] for detection in self.values()]

    def get_stacked_values_torch(self, key, idx: int = None):
        values = []
        for detection in self.values():
            v = detection[key]
            if idx is not None:
                v = v[idx]
            if isinstance(v, o3d.geometry.OrientedBoundingBox) or isinstance(
                v, o3d.geometry.AxisAlignedBoundingBox
            ):
                v = np.asarray(v.get_box_points())
            if isinstance(v, np.ndarray):
                v = torch.from_numpy(v)
            values.append(v)
        return torch.stack(values, dim=0)

    def get_stacked_values_numpy(self, key, idx: int = None):
        values = self.get_stacked_values_torch(key, idx)
        return to_numpy(values)


class MapObjectDict(DetectionDict):

    def compute_similarities(self, new_clip_ft):
        new_clip_ft = to_tensor(new_clip_ft)
        clip_fts = self.get_stacked_values_torch("clip_ft")
        similarities = F.cosine_similarity(new_clip_ft.unsqueeze(0), clip_fts)
        return similarities

    def to_serializable(self):
        s_obj_list = []
        for obj in self.values():
            s_obj_dict = copy.deepcopy(obj)
            s_obj_dict["clip_ft"] = to_numpy(s_obj_dict["clip_ft"])
            s_obj_dict["pcd_np"] = np.asarray(s_obj_dict["pcd"].points)
            s_obj_dict["bbox_np"] = np.asarray(s_obj_dict["bbox"].get_box_points())
            s_obj_dict["pcd_color_np"] = np.asarray(s_obj_dict["pcd"].colors)
            del s_obj_dict["pcd"]
            del s_obj_dict["bbox"]
            s_obj_list.append(s_obj_dict)
        return s_obj_list

    def load_serializable(self, s_obj_list):
        assert len(self) == 0, "MapObjectDict should be empty when loading"
        for s_obj_dict in s_obj_list:
            new_obj = copy.deepcopy(s_obj_dict)
            new_obj["clip_ft"] = to_tensor(new_obj["clip_ft"])
            new_obj["pcd"] = o3d.geometry.PointCloud()
            new_obj["pcd"].points = o3d.utility.Vector3dVector(new_obj["pcd_np"])
            new_obj["bbox"] = o3d.geometry.OrientedBoundingBox.create_from_points(
                o3d.utility.Vector3dVector(new_obj["bbox_np"])
            )
            new_obj["bbox"].color = new_obj["pcd_color_np"][0]
            new_obj["pcd"].colors = o3d.utility.Vector3dVector(new_obj["pcd_color_np"])
            del new_obj["pcd_np"]
            del new_obj["bbox_np"]
            del new_obj["pcd_color_np"]
            self[new_obj["id"]] = new_obj
