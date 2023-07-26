import glob
import os
import sys
import random
import os
from queue import Queue
from queue import Empty
import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    output_path = "/home/ziyu/桌面/carla_exp"
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk(os.path.join(output_path, '%06d.ply' % sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join(output_path, '%06d.png' % sensor_data.frame))
    sensor_queue.put((sensor_data.frame, sensor_name))


def main():
    actor_list = []
    sensor_list = []
    try:
        # 连接Carla服务器
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        blueprint_library = world.get_blueprint_library()

        # 设置同步模式
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        sensor_queue = Queue()
        # 创建车辆
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.etron')
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        transform = random.choice(world.get_map().get_spawn_points())
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        ego_vehicle.set_autopilot(True)
        actor_list.append(ego_vehicle)

        # 指定传感器数据的保存目录
        output_path = '../outputs/output_synchronized'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        # 创建相机传感器
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        # 注册回调函数
        camera.listen(lambda image: sensor_callback(image, sensor_queue, "camera"))
        sensor_list.append(camera)

        # # 创建LiDAR传感器
        # lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('channels', str(32))
        # lidar_bp.set_attribute('points_per_second', str(90000))
        # lidar_bp.set_attribute('rotation_frequency', str(40))
        # lidar_bp.set_attribute('range', str(20))
        # lidar_location = carla.Location(0, 0, 2)
        # lidar_rotation = carla.Rotation(0, 0, 0)
        # lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        # lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
        # lidar.listen(
        #     lambda point_cloud: sensor_callback(point_cloud, sensor_queue, "lidar"))
        # sensor_list.append(lidar)

        while True:
            world.tick()
            # 使用queue.get()函数，在数据处理完成之前，阻止程序的推进
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))
                    world.get_spectator().set_transform(camera.get_transform())

            except Empty:
                print("   Some of the sensor information is missed")

    finally:
        # 恢复异步模式，否则退出之后Carla将卡死
        world.apply_settings(original_settings)
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        for actor in actor_list:
            actor.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
