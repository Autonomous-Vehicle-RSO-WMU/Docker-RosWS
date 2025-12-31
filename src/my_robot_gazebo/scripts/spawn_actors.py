#!/usr/bin/env python3
import carla
import random

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    pedestrian_blueprints = blueprint_library.filter('walker.pedestrian.*')
    num_vehicles = 50
    num_pedestrians = 50
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    for _ in range(num_vehicles):
        bp = random.choice(vehicle_blueprints)
        sp = random.choice(spawn_points)
        actor = world.try_spawn_actor(bp, sp)
        if actor:
            actor.set_autopilot(True)
            print(f'Spawned {actor.type_id} at {sp.location}')
    for _ in range(num_pedestrians):
        bp = random.choice(pedestrian_blueprints)
        sp = carla.Transform()
        sp.location = world.get_random_location_from_navigation()
        actor = world.try_spawn_actor(bp, sp)
        if actor:
            print(f'Spawned {actor.type_id} at {sp.location}')

if __name__ == '__main__':
    main()
