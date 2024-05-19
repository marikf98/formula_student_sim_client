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
# Function to filter a point cloud based on distance and height criteria
def filter_cloud(pointcloud, min_distance=0.0, max_distance=100.0, min_height=-1.0, max_height=1.0):
    # Calculate the Euclidean distance for each point in the point cloud
    distances = np.linalg.norm(pointcloud, axis=1)
    # Filter the points based on the distance criteria
    filtered_distance = np.bitwise_and(distances > min_distance,
                                       distances < max_distance)
    # Filter the points based on the height criteria
    filtered_heights = np.bitwise_and(pointcloud[:, 2] > min_height,
                                      pointcloud[:, 2] < max_height)
    # Combine the distance and height filters
    filtered_indices = np.bitwise_and(filtered_distance, filtered_heights)

    # Return the filtered point cloud
    return pointcloud[filtered_indices, :]

# Function to calculate the bounding-box extents of a cluster along the x and y axes
def cluster_extent(cluster):
    # Returns an array of the bounding-box extents of the cluster along the x and y axes.
    return np.amax(cluster, axis=0) - np.amin(cluster, axis=0)

# Function to separate the DBSCAN results into segments, calculate their centroids, and filter out segments that exceed a maximum extent
def collate_segmentation(dbscan_obj, max_extent=1.0):
    # Sort-out labels:
    # Get the labels from the DBSCAN object
    labels = dbscan_obj.labels_
    # Get the unique labels
    unique_labels = set(labels)
    # Remove the noise label (-1)
    if -1 in unique_labels:
        unique_labels.remove(-1)  # No use for noise (label = -1)

    # Separate data into segments:
    # Initialize the segments, centroids, and relevant labels lists
    segments = []
    centroids = []
    relevant_labels = []
    # Loop over each unique label
    for idx in unique_labels:
        # Get the points that belong to the current label
        class_member_mask = (labels[dbscan_obj.core_sample_indices_] == idx)
        group = dbscan_obj.components_[class_member_mask]
        # Calculate the extents of the current cluster
        extents = cluster_extent(group)

        # Filtering out every cluster whose XY spreads too much (larger than the target):
        if group.size > 0 and np.linalg.norm(extents) < max_extent:
            # Add the current cluster to the segments list
            segments.append(group)
            # Calculate and add the centroid of the current cluster to the centroids list
            centroids.append(np.mean(group, axis=0))
            # Add the current label to the relevant labels list
            relevant_labels.append(idx)

    # Return the segments, centroids, and relevant labels
    return segments, centroids, relevant_labels


