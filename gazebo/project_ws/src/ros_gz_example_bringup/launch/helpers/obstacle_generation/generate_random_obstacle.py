import random
from .. config import *
from .. classes.core.entity import Entity
from . oob_collision_detection import boxes_overlap


def check_spawn_kill(new_entity: Entity):
    spawned_entities = robots + boundary_walls
    for spawned_entity in spawned_entities:
        if boxes_overlap(new_entity, spawned_entity):
            return True
    return False

def generate_random_border_wall():
    while True:
        start = random.randint(0, 3) # 4 sides
        size=Size(x=random.uniform(0.5, max_width/2))
        match(start):
            case 0 : # north
                wall = Wall(
                    pose=Pose(y=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), x=max_width / 2 - size.x / 2 - wall_thickness), size=size)
            case 1 : # south
                wall = Wall(
                    pose=Pose(y=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), x=-max_width / 2 + size.x / 2 - wall_thickness), size=size)
            case 2 : # east
                wall = Wall(
                    pose=Pose(x=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), y=max_width / 2 - size.x / 2 - wall_thickness, yaw=horizontal_yaw), size=size)
            case 3 : # west
                wall = Wall(
                    pose=Pose(x=random.uniform(-max_width / 2 + wall_thickness, max_width / 2 - wall_thickness), y=-max_width / 2 + size.x / 2 - wall_thickness, yaw=horizontal_yaw), size=size)

        if not check_spawn_kill(wall):
            break

    boundary_walls.append(wall)
    return wall

def generate_random_inner_wall():
    while True:
        wall = Wall(
                    pose=Pose(x=random.uniform(-max_height / 2 + wall_thickness, max_height / 2 - wall_thickness), y=random.uniform(-max_width  / 2 + wall_thickness, max_width / 2 - wall_thickness), yaw=random.uniform(0, math.pi)), size=Size(x=random.uniform(0.5, max_width/2)))
        
        if not check_spawn_kill(wall):
            break

    boundary_walls.append(wall)
    return wall

def generate_random_wall_obstacles(n_walls: int):
    n_border_walls = random.randint(0, n_wall_obstacles)
    n_inner_walls = n_wall_obstacles - n_border_walls

    border_walls = [ generate_random_border_wall() for _ in range(n_border_walls)]
    inner_walls = [generate_random_inner_wall() for _ in range(n_inner_walls)]
    
    return border_walls + inner_walls