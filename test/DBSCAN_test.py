#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import time
import pickle
import csv
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN

"""
DBSCAN_test.py
--------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Loads a point cloud from a pickle file.
2. Filters the point cloud based on specified criteria.
3. Applies the DBSCAN algorithm to the filtered point cloud.
4. Segments the point cloud into clusters and calculates the centroids of the clusters.
5. Visualizes the segmented point cloud and the centroids.

Dependencies:
- numpy: Python library for numerical computations.
- scipy.spatial.transform: Module for 3D rotations and transformations.
- time: Standard Python library for time-related tasks.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- csv: Standard Python library for reading and writing CSV files.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- sklearn.cluster: Module for clustering data.

Usage:
Run this script to start the DBSCAN test. Ensure that the point cloud data is available in a pickle file.
"""

def cluster_extent(cluster):
    # Returns an array of the bounding-box extents of the cluster along the x and y axes.
    return np.amax(cluster, axis=0) - np.amin(cluster, axis=0)


if __name__ == '__main__':
    # Load point cloud from a pickle file
    with open('pointcloud.pickle', 'rb') as pickle_file:
        pointcloud = pickle.load(pickle_file)
    print('loaded pointcloud data')
    tic = time.time()
    pointcloud[:, 2] *= -1  # Z is reversed in airsim because of flying convention
    # Filter the point cloud based on specified criteria
    distances = np.linalg.norm(pointcloud, axis=1)
    filtered_indices = np.bitwise_and(distances < 10.0,
                                      distances > 5.0)
    filtered_indices = np.bitwise_and(filtered_indices, pointcloud[:, 2] > -0.5)
    filtered_pc = pointcloud[filtered_indices, 0:2]

    print('filtering takes:', time.time()-tic)
    tic = time.time()

    # Apply the DBSCAN algorithm to the filtered point cloud
    # Compute DBSCAN
    db = DBSCAN(eps=0.1, min_samples=3).fit(filtered_pc)
    print('DBSCAN takes:', time.time() - tic)
    tic = time.time()

    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_
    # db.components correspond to the original pc array
    # db.labels correspond to each point in the array, assigining it a "group index"
    # db.core_sample_indices are indices in pc array which have SOME affiliation with a cluster

    # Number of clusters in labels, ignoring noise if present.
    unique_labels = set(labels)
    n_clusters_ = len(unique_labels) - (1 if -1 in labels else 0)
    # n_noise_ = list(labels).count(-1)

    groups = {}
    centroids = {}
    max_extent = 0.5  # Diameter of a cone. Any cluster larger than that will be disqualified.

    # Segment the point cloud into clusters and calculate the centroids of the clusters
    for idx in unique_labels:
        if idx != -1:
            class_member_mask = (labels == idx)
            group = filtered_pc[class_member_mask & core_samples_mask]  # From all pointcloud
            group2 = db.components_[labels[db.core_sample_indices_] == idx]  # From non-noise samples (components) only

            extents = cluster_extent(group)
            if np.linalg.norm(extents) < max_extent:
                groups[idx] = group
                centroids[idx] = np.mean(group, axis=0)

    print('segmentation takes:', time.time() - tic)
    tic = time.time()

    # Visualize the segmented point cloud and the centroids
    # Black removed and is used for noise instead.
    colors = [plt.cm.Spectral(each)
              for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = (labels == k)

        xy = filtered_pc[class_member_mask & core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor='k', markersize=14)

        xy = filtered_pc[class_member_mask & ~core_samples_mask]
        plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
                 markeredgecolor='k', markersize=6)

    plt.title('Estimated number of clusters: %d' % n_clusters_)
    plt.show()
    a=5

    # Plot the result
    fig, ax = plt.subplots(1, 1)
    ax.plot(filtered_pc[:, 0], filtered_pc[:, 1], '*b')
    ax.grid(True)
    plt.xlim([0, 10])
    plt.ylim([-5, 5])
    fig.show()
    a = 5