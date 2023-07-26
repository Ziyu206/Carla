import os

import carla.libcarla

import carla
import random

def main():



    try:
        client = carla.Client('localhost', 2000)
        world = client.get_world()

        client.load_world('Town04')

        # Set the simulation to sync mode
        init_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        # After that, set the TM to sync mode

        level = world.get_map()

        weather = world.get_weather()

        blueprint_library = world.get_blueprint_library()

        # # Retrieve the spectator object
        # spectator = world.get_spectator()
        #
        # # Get the location and rotation of the spectator through its transform
        # transform = spectator.get_transform()
        #
        # location = transform.location
        # rotation = transform.rotation
        # print(location)
        # print(rotation)
        #
        # self_location = carla.Location(200,200,20)
        # self_transform =carla.Transform(self_location,rotation)
        #
        # # Set the spectator with an empty transform
        # spectator.set_transform(self_transform)
        # # This will set the spectator at the origin of the map, with 0 degrees
        # # pitch, yaw and roll - a good way to orient yourself in the map

        # # Retrieve the spectator object
        # spectator = world.get_spectator()
        #
        # # Get the location and rotation of the spectator through its transform
        # transform = spectator.get_transform()
        #
        # location = transform.location
        # rotation = transform.rotation
        #
        # # Set the spectator with an empty transform
        # spectator.set_transform(carla.Transform())
        # # This will set the spectator at the origin of the map, with 0 degrees
        # pitch, yaw and roll - a good way to orient yourself in the map

        # # Get the blueprint library and filter for the vehicle blueprints
        # vehicle_blueprints = world.get_blueprint_library().find('vehicle.ford.mustang')

        # # Get the blueprint library and filter for the vehicle blueprints


        vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

        # # Get the map's spawn points
        spawn_points = world.get_map().get_spawn_points()

        # Spawn 50 vehicles randomly distributed throughout the map
        # for each spawn point, we choose a random vehicle from the blueprint library
        # for i in range(0, 50):
        #     world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

        # Spawn 50 vehicles randomly distributed throughout the map
        # for each spawn point, we choose a random vehicle from the blueprint library
        for i in range(0, 20):
            world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

        # original_settings = world.get_settings()
        # settings = world.get_settings()
        # settings.fixed_delta_seconds = 0.05
        # settings.synchronous_mode = True
        # world.apply_settings(settings)



        for vehicle in world.get_actors().filter('*vehicle*'):
            print(vehicle)
            vehicle.set_autopilot()

        ego_spawn_point = random.choice(spawn_points)

        ego_bp = world.get_blueprint_library().find('vehicle.ford.mustang')

        ego_bp.set_attribute('role_name', 'hero')

        ego_vehicle = world.spawn_actor(ego_bp, ego_spawn_point)

        trafficManager = client.get_trafficmanager()
        trafficManager.synchronous_mode = True
        trafficManager.set_desired_speed(ego_vehicle,90)

        # # Create a transform to place the camera on top of the vehicle
        # transform_ego = ego_vehicle.get_transform()
        # camera_init_trans = carla.Transform(ego_spawn_point.location + carla.Location(z=2))
        # spectator.set_transform(carla.Transform(ego_spawn_point.location + carla.Location(z=2)))

        ego_vehicle.set_autopilot()
        # We create the camera through a blueprint that defines its properties
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
                                           carla.Rotation(pitch=10, yaw=0, roll=0))
        camera = world.spawn_actor(camera_bp, camera_transform,
                                           attach_to=ego_vehicle, attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)

        # Create a transform to place the camera on top of the vehicle
        recorder_init_trans = carla.Transform(carla.Location(x=vehicle.bounding_box.extent.x+0.5,z=vehicle.bounding_box.extent.z+0.2))

        # We create the camera through a blueprint that defines its properties
        recorder_bp = world.get_blueprint_library().find('sensor.camera.rgb')

        # We spawn the camera and attach it to our ego vehicle
        recorder = world.spawn_actor(camera_bp, recorder_init_trans, attach_to=ego_vehicle)
        output_path = "/home/ziyu/data/carla_pic"
        recorder.listen(lambda image: image.save_to_disk(os.path.join(output_path, '%06d.png' % image.frame)))
        world.apply_settings(init_settings)
        world.tick()

        while True:
            world.get_spectator().set_transform(camera.get_transform())

    finally:
        # world.apply_settings(original_settings)
        print('destroying actors')
        # client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        # for sensor in sensor_list:
        #     sensor.destroy()
        # for actor in actor_list:
        #     actor.destroy()
        # Always disable sync mode before the script ends to prevent the server blocking whilst waiting for a tick
        settings.synchronous_mode = False
        trafficManager.set_synchronous_mode(False)
        print('done.')

    # We spawn the camera and attach it to our ego vehicle
    # camera = world.try_spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

    # Start camera with PyGame callback
    # camera.listen(lambda image: image.save_to_disk('out/%06d.png' % image.frame))

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')