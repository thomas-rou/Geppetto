# based on: https://gamedev.stackexchange.com/questions/25397/obb-vs-obb-collision-detection

import numpy as np
from .. classes.core.entity import Entity

def get_normal_axes(yaw):
    unit_x = np.array([1, 0])  # Corresponds to the x-axis (width direction)
    unit_y = np.array([0, 1])  # Corresponds to the y-axis (height direction)
    
    # Create a 2D rotation matrix based on the yaw angle
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])
    
    # Rotate the normal axes by the yaw
    rotated_x = np.dot(rotation_matrix, unit_x)
    rotated_y = np.dot(rotation_matrix, unit_y)
    
    return rotated_x, rotated_y

def project_point_onto_axis(point, axis):
    # Project a point onto an axis using the dot product
    return np.dot(point, axis) / np.linalg.norm(axis)

def get_box_corners(center, width, height, yaw):
    half_width = width / 2
    half_height = height / 2
    
    # Corners in the local box frame (before applying yaw)
    local_corners = np.array([
        [-half_width, -half_height],
        [half_width, -half_height],
        [half_width, half_height],
        [-half_width, half_height]
    ])
    
    # Rotation matrix for the yaw
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])
    
    # Rotate corners and translate by the box's center
    world_corners = [np.dot(rotation_matrix, corner) + center for corner in local_corners]
    
    return world_corners

def get_projection_range(corners, axis):
    # Project each corner onto the axis and get the min and max projection
    projections = [project_point_onto_axis(corner, axis) for corner in corners]
    return min(projections), max(projections)

def check_overlap(range1, range2):
    # Check if two projection ranges overlap
    return not (range1[1] < range2[0] or range2[1] < range1[0])

def boxes_overlap(entity1: Entity, entity2: Entity):
    # Get rotated normal axes for both boxes
    axes1 = get_normal_axes(entity1.pose.yaw)
    axes2 = get_normal_axes(entity2.pose.yaw)
    
    # Get the corners of both boxes
    corners1 = get_box_corners(entity1.pose.to_array()[0:2], entity1.size.x, entity1.size.y, entity1.pose.yaw)
    corners2 = get_box_corners(entity2.pose.to_array()[0:2], entity2.size.x, entity2.size.y, entity2.pose.yaw)
    
    # Check projections on each axis (both normal axes from box 1 and box 2)
    for axis in [axes1[0], axes1[1], axes2[0], axes2[1]]:
        range1 = get_projection_range(corners1, axis)
        range2 = get_projection_range(corners2, axis)
        if not check_overlap(range1, range2):
            return False  # Separating axis found, no overlap
    
    return True  # Overlap on all axes, boxes intersect