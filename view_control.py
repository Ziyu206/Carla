import os

import carla
import random
import time
import threading
import keyboard

threads = []


# def updateSpectator(vehicle, world):
#     spectator_obj_list = []
#
#     # 获得车辆的中心点，并将点定位到右上角，便于运算
#     bound_x = 0.5 + vehicle.bounding_box.extent.x
#     bound_y = 0.5 + vehicle.bounding_box.extent.y
#     bound_z = 0.5 + vehicle.bounding_box.extent.z
#
#     # 查找相机蓝图
#     camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
#
#     # 设置Camera的附加类型Camera跟随车辆(幽灵模式)；
#     Atment_SpringArmGhost = carla.libcarla.AttachmentType.SpringArmGhost
#
#     Atment_Rigid = carla.libcarla.AttachmentType.Rigid
#
#     # 设置相对车辆的安装位置，配置上帝视图(Camera无法实现上帝视图，画面会抖动)
#     Vehicle_transform_list = [
#         (carla.Location(z=50), carla.Rotation(pitch=-90))
#     ]
#
#     # 设置camera的安装位置，配置后往前视图以及前后左右视图
#     Camera_transform_list = [
#         (carla.Transform(carla.Location(x=-8, y=0, z=5),
#                          carla.Rotation(pitch=15, yaw=0, roll=0)), Atment_SpringArmGhost),
#
#         (carla.Transform(carla.Location(x=bound_x, y=0, z=bound_z),
#                          carla.Rotation(pitch=-15, yaw=180, roll=0)), Atment_SpringArmGhost),
#
#         (carla.Transform(carla.Location(x=-bound_x, y=0, z=bound_z),
#                          carla.Rotation(pitch=-15, yaw=-180, roll=0)), Atment_SpringArmGhost),
#
#         (carla.Transform(carla.Location(x=bound_x - 0.5, y=-bound_y, z=bound_z),
#                          carla.Rotation(pitch=-15, yaw=120, roll=20)), Atment_SpringArmGhost),
#
#         (carla.Transform(carla.Location(x=bound_x - 0.5, y=bound_y, z=bound_z),
#                          carla.Rotation(pitch=-15, yaw=-120, roll=-20)), Atment_SpringArmGhost)
#     ]
#
#     # 拼接两个transform_list
#     spectator_transform_list = Vehicle_transform_list + Camera_transform_list
#
#     # 上帝视图坐标系以及所有camera对象填入spectator_obj_list；
#     for spectator_transform_index in spectator_transform_list:
#
#         # spectator_transform_list第0个元素为上帝视图坐标系
#         if spectator_transform_list.index(spectator_transform_index) == 0:
#             spectator_obj_list.append(spectator_transform_index)
#
#         # spectator_transform_list其余元素为Camera安装参数，下面生成Camera对象
#         else:
#             camera = world.spawn_actor(camera_bp, spectator_transform_index[0],
#                                        attach_to=vehicle, attachment_type=spectator_transform_index[1])
#             spectator_obj_list.append(camera)
#
#     # 设置Vehicle_transform_list[0]为初始视图(上帝视图)；
#     spectator_obj = Vehicle_transform_list[0]
#
#     # 每一帧都需要更新视图，因为坐标时刻在变化；
#     while True:
#         # if spectator_obj_list.index(spectator_obj) != 0:
#         #     camera.listen(lambda image: image.save_to_disk(os.path.join("/home/ziyu/桌面/carla_exp", '%06d.png' % image.frame)))
#         # 按Tab键切换
#         if keyboard.is_pressed("tab"):
#             # 上一个spectator的索引号；
#             last_spectator_obj_index = spectator_obj_list.index(spectator_obj)
#             # 计算下一个spectator的索引，如果列表索引超限则重新拿第0个spectator；
#             spectator_obj_index = last_spectator_obj_index + 1 if len(
#                 spectator_obj_list) - last_spectator_obj_index - 1 > 0 else 0
#             spectator_obj = spectator_obj_list[spectator_obj_index]
#             time.sleep(0.2)
#
#         # 更新视图
#         if spectator_obj_list.index(spectator_obj) == 0:
#             # 设置上帝视图
#             Vehicle_transform = carla.Transform(vehicle.get_transform().location + spectator_obj_list[0][0],
#                                                 spectator_obj_list[0][1])
#             world.get_spectator().set_transform(Vehicle_transform)
#         else:
#             # 设置其他Camera视图
#             world.get_spectator().set_transform(spectator_obj.get_transform())


def autopilot(vehicle):
    # 自动领航
    vehicle.set_autopilot()

# def record_mode(vehicle,world):
#     record_mode = False
#     sensor_bps = world.get_blueprint_library().find('sensor.camera.rgb')
#
#     sensor_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
#                     carla.Rotation(pitch=15, yaw=0, roll=0)),
#     sensor = world.spawn_actor(sensor_bps, sensor_transform,
#                                attach_to=vehicle)
#     while True:
#         if keyboard.is_pressed("r"):
#             record_mode = not record_mode
#         if record_mode:
#             print('record')
#             sensor.listen(lambda image: image.save_to_disk(os.path.join(output_path, '%06d.png' % image.frame)))

def POV_change(vehicle, world):
    spectator_obj_list = []
    camera_lock = False
    record_mode = False
    # 获得车辆的中心点，并将点定位到右上角，便于运算
    bound_x = 0.5 + vehicle.bounding_box.extent.x
    bound_y = 0.5 + vehicle.bounding_box.extent.y
    bound_z = 0.5 + vehicle.bounding_box.extent.z

    # 查找相机蓝图
    camera_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')

    # 设置Camera的附加类型Camera跟随车辆(幽灵模式)；
    Atment_SpringArmGhost = carla.libcarla.AttachmentType.SpringArmGhost

    Atment_Rigid = carla.libcarla.AttachmentType.Rigid

    # 设置camera的安装位置，配置后往前视图以及前后左右视图
    Camera_transform_list = [
        (carla.Transform(carla.Location(x=-8, y=0, z=5),
                         carla.Rotation(pitch=15, yaw=0, roll=0)), Atment_SpringArmGhost),

        (carla.Transform(carla.Location(x=bound_x, y=0, z=bound_z),
                         carla.Rotation(pitch=-15, yaw=180, roll=0)), Atment_SpringArmGhost),

        (carla.Transform(carla.Location(x=-bound_x, y=0, z=bound_z),
                         carla.Rotation(pitch=-15, yaw=-180, roll=0)), Atment_SpringArmGhost),

        (carla.Transform(carla.Location(x=bound_x - 0.5, y=-bound_y, z=bound_z),
                         carla.Rotation(pitch=-15, yaw=120, roll=23)), Atment_SpringArmGhost),

        (carla.Transform(carla.Location(x=bound_x - 0.5, y=bound_y, z=bound_z),
                         carla.Rotation(pitch=-15, yaw=-120, roll=-23)), Atment_SpringArmGhost)
    ]

    # # 拼接两个transform_list
    # spectator_transform_list = Vehicle_transform_list + Camera_transform_list

    # 上帝视图坐标系以及所有camera对象填入spectator_obj_list；
    for camera_transform_index in Camera_transform_list:

        # spectator_transform_list第0个元素为上帝视图坐标系
        # if spectator_transform_list.index(spectator_transform_index) == 0:
        #     spectator_obj_list.append(spectator_transform_index)

        # spectator_transform_list其余元素为Camera安装参数，下面生成Camera对象
            camera = world.spawn_actor(camera_bp, camera_transform_index[0],
                                       attach_to=vehicle, attachment_type=camera_transform_index[1])
            spectator_obj_list.append(camera)

    # 设置Vehicle_transform_list[0]为初始视图(上帝视图)；
    spectator_obj = spectator_obj_list[1]
    output_path = "/home/ziyu/data/carla_pic"
    output_path = os.path.join(output_path, '%06d.png')
    cc = carla.ColorConverter.CityScapesPalette
    spectator_obj.listen(lambda image: image.save_to_disk(output_path % image.frame, cc))


    # 每一帧都需要更新视图，因为坐标时刻在变化；
    while True:
        # if spectator_obj_list.index(spectator_obj) != 0:
        #     camera.listen(lambda image: image.save_to_disk(os.path.join("/home/ziyu/桌面/carla_exp", '%06d.png' % image.frame)))
        # 按Tab键切换

        if keyboard.is_pressed("space"):
            camera_lock = not camera_lock



        if keyboard.is_pressed("tab") and camera_lock != True:
            # 上一个spectator的索引号；
            last_spectator_obj_index = spectator_obj_list.index(spectator_obj)
            # 计算下一个spectator的索引，如果列表索引超限则重新拿第0个spectator；
            spectator_obj_index = last_spectator_obj_index + 1 if len(
                spectator_obj_list) - last_spectator_obj_index - 1 > 0 else 0
            time.sleep(0.2)
            spectator_obj = spectator_obj_list[spectator_obj_index]
        world.get_spectator().set_transform(spectator_obj.get_transform())




        # 更新视图
        # if spectator_obj_list.index(spectator_obj) == 0:
        #     # 设置上帝视图
        #     Vehicle_transform = carla.Transform(vehicle.get_transform().location + spectator_obj_list[0][0],
        #                                         spectator_obj_list[0][1])
        #     world.get_spectator().set_transform(Vehicle_transform)
        # else:
        #     # 设置其他Camera视图


def main():
    try:
        # 连接至服务器
        client = carla.Client('localhost', 2000)
        client.set_timeout(10)

        # 获取世界对象
        world = client.get_world()

        # 获取车辆原型
        car_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
        car_blueprint.set_attribute('role_name', 'ego_vehicle')

        # 在随机位置生成车辆
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(car_blueprint, spawn_point)

        # # 创建自动运行车辆的线程
        # Thread_autopilot = threading.Thread(target=autopilot, args=(vehicle,))
        #
        # # 创建切换视图的线程
        # Thread_updateSpectator = threading.Thread(target=POV_change, args=(vehicle, world))
        #
        # # 添加线程
        # threads.append(Thread_autopilot)
        # threads.append(Thread_updateSpectator)
        # for thread in threads:
        #     thread.start()
        autopilot(vehicle)
        POV_change(vehicle,world)


    finally:
        # 销毁车辆
        vehicle.destroy()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')