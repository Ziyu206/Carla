import random
import carla



def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.load_world('Town01')
        num_walkers = 200
        num_vehicle = 20
        # 设置跑步的行人比例
        percentage_pedestrians_running = 0.25
        # 设置横穿马路的行人比例
        percentage_pedestrians_crossing = 0.15

        # 获取我们client所对应的world
        world = client.get_world()
        # 获得这个world中的观察者
        spectator = world.get_spectator()

        # 获得观察者的方位信息
        transform = spectator.get_transform()

        # 根据观察者默认方位设置新的方位
        location = transform.location + carla.Location(x=-30, z=20)
        rotation = carla.Rotation(pitch=-20, yaw=-20, roll=0)
        new_transform = carla.Transform(location, rotation)

        # 将观察者设置到新方位上
        spectator.set_transform(new_transform)

        # 获得整个的blueprint库并从中筛选出车辆
        vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')

        # 获得整个的blueprint库并从中筛选出行人
        ped_blueprints = world.get_blueprint_library().filter('*pedestrian*')

        # 通过world获得map并获得所有可以生成车辆的地点
        vehicle_spawn_points = world.get_map().get_spawn_points()

        # 通过world获得所有可以生成行人的地点并存储
        ped_spawn_points = []
        for i in range(num_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                ped_spawn_points.append(spawn_point)

        # 在地图上随机生成num_vehicle辆车，每辆车为车辆蓝图库中的随机车辆
        for i in range(0, num_vehicle):
            world.try_spawn_actor(random.choice(vehicle_blueprints),
                                  random.choice(vehicle_spawn_points))

        # 创建用来存储行人，行人速度设置和行人控制器的list
        walker_batch = []
        walker_speed = []
        walker_ai_batch = []

        # 在地图上随机生成num_walkers个行人，每个行人为行人蓝图库中的随机行人，并设定行人的移动速度
        for j in range(num_walkers):
            walker_bp = random.choice(ped_blueprints)

            # 取消行人无敌状态
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')

            # 设置行人的移动速度
            if walker_bp.has_attribute('speed'):
                if random.random() > percentage_pedestrians_running:
                    # 将对应行人速度设置为走路速度
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # 将对应行人速度设置为跑步速度
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            # 从可生成行人的生成点中随机选择并生成随机行人，之后将生成的行人添加到同一批中
            walker_batch.append(world.try_spawn_actor(walker_bp, random.choice(ped_spawn_points)))

        # 从蓝图库中寻找控制行人行动逻辑的控制器
        walker_ai_blueprint = world.get_blueprint_library().find('controller.ai.walker')

        # 为整批行人各自生成对应的控制器，并把控制器添加到代表批量控制器的列表中
        for walker in world.get_actors().filter('*pedestrian*'):
            walker_ai_batch.append(world.spawn_actor(walker_ai_blueprint, carla.Transform(), walker))

        # 批量启动行人控制器，并设置控制器参数
        for i in range(len(walker_ai_batch)):
            # 启动控制器
            walker_ai_batch[i].start()
            # 通过控制器设置行人的目标点
            walker_ai_batch[i].go_to_location(world.get_random_location_from_navigation())
            # 通过控制器设置行人的行走速度
            walker_ai_batch[i].set_max_speed(float(walker_speed[i]))

        # 设置行人横穿马路的参数
        world.set_pedestrians_cross_factor(percentage_pedestrians_crossing)

        # 从world中获取车辆，并将每辆车的autopilot模式设置为打开
        for vehicle in world.get_actors().filter('*vehicle*'):
            vehicle.set_autopilot()

        # 从可生成车辆地点中随机选择一个地点并记录
        ego_spawn_point = random.choice(vehicle_spawn_points)
        # 从蓝图库中挑选我们需要的主车蓝图
        ego_bp = world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')
        # 设置主车蓝图的属性中的角色名
        ego_bp.set_attribute('role_name', 'hero')
        # 生成主车
        ego_vehicle = world.spawn_actor(ego_bp, ego_spawn_point)
        # 将主车设置为自动驾驶模式
        ego_vehicle.set_autopilot()

        # 从蓝图库中寻找rgb相机
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # 设置rgb相机的方位信息
        camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
                                           carla.Rotation(pitch=10, yaw=0, roll=0))
        # 生成rgb相机并用SpringArmGhost的方式绑定到主车上
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle,
                                   attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)

        while True:
            # 从world中获取观察者视角，并将观察者视角的方位信息设置为相机的对应方位信息
            world.get_spectator().set_transform(camera.get_transform())
            # 等待server更新world状态
            world.wait_for_tick()

    finally:
        # 停止并销毁所有controller
        for controller in world.get_actors().filter('*controller*'):
            controller.stop()
        # 销毁所有车辆
        for vehicle in world.get_actors().filter('*vehicle*'):
            vehicle.destroy()
        # 销毁所有行人
        for walker in world.get_actors().filter('*walker*'):
            walker.destroy()


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
