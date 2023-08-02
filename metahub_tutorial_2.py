import math
import os
import queue
import random
import sys

import carla



def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        client.load_world('Town10HD')
        num_walkers = 20
        num_vehicle = 20
        # 设置跑步的行人比例
        percentage_pedestrians_running = 0.25
        # 设置横穿马路的行人比例
        percentage_pedestrians_crossing = 0.15

        # 获取我们client所对应的world
        world = client.get_world()
        # 获得这个world中的观察者
        spectator = world.get_spectator()

        # 获得当前客户端的交通管理器
        traffic_manager = client.get_trafficmanager()

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
            # walker_batch.append(world.try_spawn_actor(walker_bp, random.choice(ped_spawn_points)))

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

        # 设定主车生成位置
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

        # 从蓝图库中寻找深度相机
        camera_bp_depth = world.get_blueprint_library().find('sensor.camera.depth')
        camera_bp_depth_1 = world.get_blueprint_library().find('sensor.camera.depth')

        # 从蓝图库中寻找rgb相机
        camera_bp_record = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp_record_1 = world.get_blueprint_library().find('sensor.camera.rgb')

        # 从蓝图库中寻找语义分割相机
        camera_bp_sem = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        camera_bp_sem_1 = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')

        # 从蓝图库中寻找实例分割相机
        camera_bp_ins = world.get_blueprint_library().find('sensor.camera.instance_segmentation')
        camera_bp_ins_1 = world.get_blueprint_library().find('sensor.camera.instance_segmentation')

        # 从蓝图库中寻找Lidar的蓝图
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp_1 = world.get_blueprint_library().find('sensor.lidar.ray_cast')

        # 从蓝图库中寻找semantic_Lidar的蓝图
        lidar_bp_sem = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        lidar_bp_sem_1 = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')

        # 设置RGB相机蓝图属性
        camera_bp_record.set_attribute('bloom_intensity','1')
        camera_bp_record.set_attribute('fov','120')
        camera_bp_record.set_attribute('slope','0.5')

        # 设置深度相机蓝图属性
        camera_bp_depth_1.set_attribute('lens_x_size','0.8')
        camera_bp_depth_1.set_attribute('lens_y_size','0.8')
        camera_bp_depth_1.set_attribute('fov','120')

        # 设置语义分割相机蓝图属性
        camera_bp_sem_1.set_attribute('lens_x_size', '0.8')
        camera_bp_sem_1.set_attribute('lens_y_size', '0.8')
        camera_bp_sem_1.set_attribute('fov', '120')


        # 设置实例分割相机蓝图属性
        camera_bp_ins_1.set_attribute('lens_x_size', '0.8')
        camera_bp_ins_1.set_attribute('lens_y_size', '0.8')
        camera_bp_ins_1.set_attribute('fov', '120')

        # 设置Lidar蓝图属性
        lidar_bp_1.set_attribute('channels', '64')
        lidar_bp_1.set_attribute('points_per_second', '560000')
        lidar_bp_1.set_attribute('upper_fov', '30')
        lidar_bp_1.set_attribute('dropoff_general_rate', '0.3')

        # 设置semantic_Lidar蓝图属性
        lidar_bp_sem_1.set_attribute('channels', '64')
        lidar_bp_sem_1.set_attribute('points_per_second', '560000')
        lidar_bp_sem_1.set_attribute('upper_fov', '30')

        # 设置rgb相机的方位信息
        camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
                                           carla.Rotation(pitch=10, yaw=0, roll=0))
        camera_transform_record = carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00))

        # 生成rgb相机并绑定到主车上
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle,
                                   attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)
        camera_record = world.spawn_actor(camera_bp_record, camera_transform_record, attach_to=ego_vehicle)
        camera_record_1 = world.spawn_actor(camera_bp_record_1, camera_transform_record, attach_to=ego_vehicle)

        # 生成深度相机并绑定到主车上
        camera_depth = world.spawn_actor(camera_bp_depth, camera_transform_record, attach_to=ego_vehicle)
        camera_depth_1 = world.spawn_actor(camera_bp_depth_1, camera_transform_record, attach_to=ego_vehicle)

        # 生成语义分割相机并绑定到主车上
        camera_sem = world.spawn_actor(camera_bp_sem, camera_transform_record, attach_to=ego_vehicle)
        camera_sem_1 = world.spawn_actor(camera_bp_sem_1, camera_transform_record, attach_to=ego_vehicle)

        # 生成实例分割相机并绑定到主车上
        camera_ins = world.spawn_actor(camera_bp_ins, camera_transform_record, attach_to=ego_vehicle)
        camera_ins_1 = world.spawn_actor(camera_bp_ins_1, camera_transform_record, attach_to=ego_vehicle)

        # 生成lidar并绑定到主车上
        lidar = world.spawn_actor(lidar_bp, camera_transform_record, attach_to=ego_vehicle)
        lidar_1 = world.spawn_actor(lidar_bp_1, camera_transform_record, attach_to=ego_vehicle)

        # 生成lidar并绑定到主车上
        lidar_sem = world.spawn_actor(lidar_bp_sem, camera_transform_record, attach_to=ego_vehicle)
        lidar_sem_1 = world.spawn_actor(lidar_bp_sem_1, camera_transform_record, attach_to=ego_vehicle)

        # 获得当前模拟世界的设定
        setting = world.get_settings()
        # 设定为异步模式
        setting.synchronous_mode = True
        # 将时间步长设定为固定的0.03秒
        setting.fixed_delta_seconds = 0.03
        # 应用设定
        world.apply_settings(setting)

        # 将交通管理器设置为同步模式
        traffic_manager.synchronous_mode = True
        # 通过交通管理器设置所有车辆相对于限速的差值，这里为负即为所有车辆都会超速行驶
        traffic_manager.global_percentage_speed_difference(-30)

        # 设定存储摄像头数据的队列
        image_queue = queue.Queue()
        image_queue_1 = queue.Queue()

        lidar_measurement_queue = queue.Queue()
        lidar_measurement_queue_1 = queue.Queue()

        # 设定color_converter用于将深度元数据转成深度图
        depth_color_converter = carla.ColorConverter.LogarithmicDepth

        # 设定color_converter用于将语义分割元数据转成语义分割图
        semantic_color_converter = carla.ColorConverter.CityScapesPalette

        # 设定传感器每读取一帧数据后存储到队列中(同步模式)
        # camera.listen(image_queue.put)
        # camera_record.listen(image_queue.put)
        # camera_record_1.listen(image_queue_1.put)
        # camera_depth.listen(image_queue.put)
        # camera_depth_1.listen(image_queue_1.put)
        # camera_sem.listen(image_queue.put)
        # camera_sem_1.listen(image_queue_1.put)
        camera_ins.listen(image_queue.put)
        camera_ins_1.listen(image_queue_1.put)
        # lidar.listen(lidar_measurement_queue.put)
        # lidar_1.listen(lidar_measurement_queue_1.put)
        lidar_sem.listen(lidar_measurement_queue.put)
        lidar_sem_1.listen(lidar_measurement_queue_1.put)

        # 设定数据的存储路径
        output_path = os.path.join("/home/ziyu/data/carla_pic", '%06d.png')
        output_path_1 = os.path.join("/home/ziyu/data/carla_pic/pic_1", '%06d.png')

        output_path_lidar = os.path.join("/home/ziyu/data/carla_pic", '%06d.ply')
        output_path_lidar_1 = os.path.join("/home/ziyu/data/carla_pic/pic_1", '%06d.ply')
        # 令摄像头读取数据并存储(异步模式)
        #camera.listen(lambda image: image.save_to_disk(output_path % image.frame))

        while True:
            # 从world中获取观察者视角，并将观察者视角的方位信息设置为相机的对应方位信息
            world.get_spectator().set_transform(camera.get_transform())

            # 如果为同步模式设定
            if traffic_manager.synchronous_mode:
                # 更新模拟世界
                world.tick()
                # 从队列中读取传感器图像
                image = image_queue.get()
                image_1 = image_queue_1.get()
                measurement = lidar_measurement_queue.get()
                measurement_1 = lidar_measurement_queue_1.get()

                # 将图像存储到本地路径(同步模式)
                # image.save_to_disk(output_path % image.frame)
                # image.save_to_disk(output_path_1 % image.frame, semantic_color_converter)
                measurement.save_to_disk(output_path_lidar % measurement.frame)
                measurement_1.save_to_disk(output_path_lidar_1 % measurement_1.frame)
            # 如果为异步模式设定
            else:
                # 更新模拟世界
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

        # 获得当前模拟世界设定
        settings = world.get_settings()
        # 设定为异步模式
        settings.synchronous_mode = False
        # 设定为可变时间步长
        settings.fixed_delta_seconds = None
        # 应用设定
        world.apply_settings(settings)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
