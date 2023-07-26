import carla

try:

    client = carla.Client('localhost', 2000)
    client.set_timeout(3.0)
    world = client.get_world()
    world_map = world.get_map()
    vehicles_list = world.get_actors().filter('vehicle.*')
    for v in vehicles_list:
        if v.attributes['role_name'] == "hero":
            hero_vehicle = v  # Actor

    while True:
        # print('position:',hero_vehicle.get_transform().location)
        print('vehicle waypoint', world_map.get_waypoint(hero_vehicle.get_transform().location).lane_id)

except KeyboardInterrupt:
    print('\nCancelled by user. Bye!')
