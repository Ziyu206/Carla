import random
import carla



def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.load_world('/home/ziyu/data/carla/CarlaUE4/Content/Carla/Maps/OpenDrive/01(1).xodr')

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

        #
        # # todo
        # ego_spawn_point =
        # # 从蓝图库中挑选我们需要的主车蓝图
        # ego_bp = world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')
        # # 设置主车蓝图的属性中的角色名
        # ego_bp.set_attribute('role_name', 'hero')
        # # 生成主车
        # ego_vehicle = world.spawn_actor(ego_bp, ego_spawn_point)
        # # 将主车设置为自动驾驶模式
        # ego_vehicle.set_autopilot()
        #
        # # 从蓝图库中寻找rgb相机
        # camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # # 设置rgb相机的方位信息
        # camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
        #                                    carla.Rotation(pitch=10, yaw=0, roll=0))
        # # 生成rgb相机并用SpringArmGhost的方式绑定到主车上
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle,
        #                            attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)

        while True:
            # 从world中获取观察者视角，并将观察者视角的方位信息设置为相机的对应方位信息
            # world.get_spectator().set_transform(camera.get_transform())
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
