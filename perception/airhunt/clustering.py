import HiPart.__utility_functions as util
import numpy as np
from HiPart.__partition_class import Partition
from sklearn.cluster import KMeans
from treelib import Tree


class BisectingKmeans(Partition):
    decreasing = True

    def __init__(self, max_clusters_number=100, min_sample_split=5, random_state=None):
        super().__init__(
            max_clusters_number=max_clusters_number, min_sample_split=min_sample_split
        )
        self.random_state = random_state

    def fit(self, X):
        self.X = X
        self.samples_number = X.shape[0]
        indices = np.array([int(i) for i in range(np.size(self.X, 0))])
        tree = Tree()
        self.node_ids = 0
        self.cluster_color = 0
        tree.create_node(
            tag="cl_" + str(self.node_ids),
            identifier=self.node_ids,
            data=self.calculate_node_data(indices, self.cluster_color),
        )
        selected_node = 0
        if not tree.get_node(0).data["split_permission"]:
            print("cannot split at all")
            return self
        found_clusters = 1
        while selected_node is not None and found_clusters < self.max_clusters_number:
            self.split_function(tree, selected_node)
            selected_node = self.select_kid(tree.leaves(), decreasing=self.decreasing)
            found_clusters = found_clusters + 1
        self.tree = tree
        return self

    def split_function(self, tree, selected_node):
        node = tree.get_node(selected_node)
        node.data["split_permission"] = False
        left_index = node.data["left_indices"]
        right_index = node.data["right_indices"]
        tree.create_node(
            tag="cl" + str(self.node_ids + 1),
            identifier=self.node_ids + 1,
            parent=node.identifier,
            data=self.calculate_node_data(left_index, node.data["color_key"]),
        )
        tree.create_node(
            tag="cl" + str(self.node_ids + 2),
            identifier=self.node_ids + 2,
            parent=node.identifier,
            data=self.calculate_node_data(right_index, self.cluster_color + 1),
        )
        self.cluster_color += 1
        self.node_ids += 2

    def calculate_node_data(self, indices, key):
        if indices.shape[0] > self.min_sample_split:
            model = KMeans(n_clusters=2, n_init=10, random_state=self.random_state)
            labels = model.fit_predict(self.X[indices, :])
            centers = model.cluster_centers_
            left_child = indices[np.where(labels == 0)]
            right_child = indices[np.where(labels == 1)]
            centers = centers
            centered = util.center_data(self.X[indices, :])
            scat = np.linalg.norm(centered, ord="fro")
            split_criterion = scat
            flag = True
        else:
            left_child = None
            right_child = None
            centers = None
            split_criterion = None
            flag = False
        return {
            "indices": indices,
            "left_indices": left_child,
            "right_indices": right_child,
            "centers": centers,
            "split_criterion": split_criterion,
            "split_permission": flag,
            "color_key": key,
            "dendrogram_check": False,
        }

    @property
    def random_state(self):
        return self._random_seed

    @random_state.setter
    def random_state(self, v):
        if v is not None and (not isinstance(v, int)):
            raise ValueError(
                "BisectingKmeans: min_sample_split: Invalid value it should be int and > 1"
            )
        np.random.seed(v)
        self._random_seed = v


class SceneHierarchicalClustering(BisectingKmeans):

    def __init__(self, max_clusters_number=100, min_sample_split=5, random_state=None):
        super().__init__(
            max_clusters_number=max_clusters_number, random_state=random_state
        )

    @staticmethod
    def select_frame(obj_set, frames, current_keys):
        candidates = []
        for key in frames.keys():
            if obj_set.issubset(frames[key].full_obj_list.keys()):
                candidates.append(key)
        if len(candidates) == 0:
            return None
        else:

            def key_func(x):
                return (
                    -len(frames[x].full_obj_list),
                    -sum([frames[x].full_obj_list[obj] for obj in obj_set]),
                )

            sorted_candidates = sorted(candidates, key=lambda x: key_func(x))
            for key in sorted_candidates:
                if key in current_keys:
                    return key
            return sorted_candidates[0]

    def fit(self, X, obj_ids, frames):
        snapshots = {}
        self.X = X
        self.samples_number = X.shape[0]
        indices = np.array([int(i) for i in range(np.size(self.X, 0))])
        tree = Tree()
        self.node_ids = 0
        self.cluster_color = 0
        tree.create_node(
            tag="cl_" + str(self.node_ids),
            identifier=self.node_ids,
            data=self.calculate_node_data(indices, self.cluster_color),
        )
        selected_node = 0
        if not tree.get_node(0).data["split_permission"]:
            print("cannot split at all")
            return self
        found_clusters = 1
        while True:
            while True:
                selected_node = self.select_kid(
                    tree.leaves(), decreasing=self.decreasing
                )
                if selected_node is None:
                    break
                obj_indices = tree.get_node(selected_node).data["indices"]
                obj_set = set([obj_ids[idx] for idx in obj_indices])
                key = self.select_frame(obj_set, frames, snapshots.keys())
                if key is not None:
                    if key not in snapshots:
                        snapshots[key] = frames[key]
                        snapshots[key].cluster = list(obj_set)
                    else:
                        snapshots[key].cluster += list(obj_set)
                    tree.remove_node(selected_node)
                else:
                    break
            if selected_node is None:
                break
            self.split_function(tree, selected_node)
            found_clusters = found_clusters + 1
        return snapshots

    def calculate_node_data(self, indices, key):
        if indices.shape[0] > 2:
            model = KMeans(n_clusters=2, n_init=10, random_state=self.random_state)
            labels = model.fit_predict(self.X[indices, :])
            centers = model.cluster_centers_
            left_child = indices[np.where(labels == 0)]
            right_child = indices[np.where(labels == 1)]
            centers = centers
            centered = util.center_data(self.X[indices, :])
            scat = np.linalg.norm(centered, ord="fro")
            split_criterion = indices.shape[0]
            flag = True
        elif indices.shape[0] == 2:
            left_child = indices[0:1]
            right_child = indices[1:2]
            centers = self.X[indices, :]
            centered = util.center_data(self.X[indices, :])
            scat = np.linalg.norm(centered, ord="fro")
            split_criterion = indices.shape[0]
            flag = True
        else:
            left_child = None
            right_child = None
            centers = None
            centered = util.center_data(self.X[indices, :])
            scat = np.linalg.norm(centered, ord="fro")
            split_criterion = indices.shape[0]
            flag = True
        return {
            "indices": indices,
            "left_indices": left_child,
            "right_indices": right_child,
            "centers": centers,
            "split_criterion": split_criterion,
            "split_permission": flag,
            "color_key": key,
            "dendrogram_check": False,
        }
