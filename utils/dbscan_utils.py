import numpy as np

"""
dbscan_utils.py
---------------
This script is part of a simulation project for performing DBSCAN clustering on point cloud data.

It performs the following tasks:

1. Defines a function `filter_cloud` to filter a point cloud based on distance and height criteria.
2. Defines a function `cluster_extent` to calculate the bounding-box extents of a cluster along the x and y axes.
3. Defines a function `collate_segmentation` to separate the DBSCAN results into segments, calculate their centroids, and filter out segments that exceed a maximum extent.

Dependencies:
- numpy: Python library for numerical computations.

Usage:
Import this module to use the `filter_cloud`, `cluster_extent`, and `collate_segmentation` functions for DBSCAN clustering operations on point cloud data. Note that the actual DBSCAN operation is performed outside of this module.
"""

# The actual DBSCAN is performed outside of this module.

def filter_cloud(pointcloud, min_distance=0.0, max_distance=100.0, min_height=-1.0, max_height=1.0):
    distances = np.linalg.norm(pointcloud, axis=1)
    filtered_distance = np.bitwise_and(distances > min_distance,
                                       distances < max_distance)
    filtered_heights = np.bitwise_and(pointcloud[:, 2] > min_height,
                                      pointcloud[:, 2] < max_height)
    filtered_indices = np.bitwise_and(filtered_distance, filtered_heights)

    return pointcloud[filtered_indices, :]


def cluster_extent(cluster):
    # Returns an array of the bounding-box extents of the cluster along the x and y axes.
    return np.amax(cluster, axis=0) - np.amin(cluster, axis=0)


def collate_segmentation(dbscan_obj, max_extent=1.0):
    # Sort-out labels:
    labels = dbscan_obj.labels_
    unique_labels = set(labels)
    if -1 in unique_labels:
        unique_labels.remove(-1)  # No use for noise (label = -1)

    # Separate data into segments:
    segments = []
    centroids = []
    relevant_labels = []
    for idx in unique_labels:

        class_member_mask = (labels[dbscan_obj.core_sample_indices_] == idx)
        group = dbscan_obj.components_[class_member_mask]
        extents = cluster_extent(group)

        # Filtering out every cluster whose XY spreads too much (larger than the target):
        if group.size > 0 and np.linalg.norm(extents) < max_extent:
            segments.append(group)
            centroids.append(np.mean(group, axis=0))
            relevant_labels.append(idx)

    return segments, centroids, relevant_labels


