import math
import os
import queue
import random
import sys
import time
import carla
import pygame
import numpy as np
from carla import VehicleLightState


# 根据天气和车辆状态实时更新车辆灯光信息
def update_light_state(world):
    # 获取天气信息
    weather = world.get_weather()
    # 获取每辆车
    for vehicle in world.get_actors().filter('*vehicle*'):
        # 获取每辆车的控制信息
        control = vehicle.get_control()
        # 获取每辆车的灯光信息
        current_lights = vehicle.get_light_state()
        # 当进入夜晚打开位置灯，打开远光灯以及内部灯
        if weather.sun_altitude_angle < 0:
            current_lights |= carla.VehicleLightState.Position
            current_lights |= carla.VehicleLightState.HighBeam
            current_lights |= carla.VehicleLightState.Interior
        # 进入白天，关掉远灯及内部灯
        if weather.sun_altitude_angle > 0:
            current_lights &= 0b11011111011
        # 当雾气浓度大于30或者降雨量大于30时，打开位置灯，近光灯以及雾灯
        if weather.fog_density > 30 or weather.precipitation > 30:
            current_lights |= carla.VehicleLightState.Position
            current_lights |= carla.VehicleLightState.LowBeam
            current_lights |= carla.VehicleLightState.Fog
        # 当降雨小于30并且雾气浓度小于30时关闭近光灯以及雾灯
        if weather.fog_density < 30 and weather.precipitation < 30:
            current_lights &= 0b11101111101
        # 当踩下刹车时，亮起刹车灯
        if control.brake > 0.1:
            current_lights |= carla.VehicleLightState.Brake
        # 刹车松开，关闭刹车灯
        if control.brake <= 0.1:
            current_lights &= 0b11111110111
        # 方向盘左转，亮起左转向灯
        if control.steer < -0.1:
            current_lights |= carla.VehicleLightState.LeftBlinker
        # 方向盘右转，亮起右转向灯
        if control.steer > 0.1:
            current_lights |= carla.VehicleLightState.RightBlinker
        # 方向回正，关闭所有转向灯
        if abs(control.steer) < 0.1:
            current_lights &= 0b11111001111
        # 处于倒档时，亮起倒车灯
        if control.reverse:
            current_lights |= carla.VehicleLightState.Reverse
        # 不处于倒档时，关闭倒车灯
        if not control.reverse:
            current_lights &= 0b11110111111

        # 应用车灯信息
        vehicle.set_light_state(VehicleLightState(current_lights))

def info(ego_vehicle,world):
    vehicles = world.get_actors().filter("*vehicle*")

    active_vehicle_num = 0
    ego_loc = ego_vehicle.get_transform()
    for vehicle in vehicles:
        t = vehicle.get_transform()
        get_distance = lambda l: math.sqrt(
            (l.x - t.location.x) ** 2 + (l.y - t.location.y) ** 2 + (l.z - t.location.z) ** 2)
        distance = get_distance(ego_loc.location)
        if distance < 100.0:
            active_vehicle_num += 1

    _info_text = "Vehicle status: %d vehicles nearby" % (active_vehicle_num - 1)
    return _info_text

# 限值函数，将输入的参数限制在0到100中间后输出
def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))

# 太阳类，用以控制太阳方位角和太阳高度较
class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0
    # 步进更新，更新太阳高度，方位角
    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = (70 * math.sin(self._t)) - 20

    # 输出当前太阳高度，方位角
    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)

# 风暴类用于控制降水，云雾，湿度，积水，风等
class Storm(object):
    def __init__(self, precipitation):
        self._t = precipitation if precipitation > 0.0 else -50.0
        self._increasing = True
        self.clouds = 0.0
        self.rain = 0.0
        self.wetness = 0.0
        self.puddles = 0.0
        self.wind = 0.0
        self.fog = 0.0

    # 更新风暴类中的所有参数，降水云雾等
    def tick(self, delta_seconds):
        delta = (1.3 if self._increasing else -1.3) * delta_seconds
        self._t = clamp(delta + self._t, -250.0, 100.0)
        self.clouds = clamp(self._t + 40.0, 0.0, 90.0)
        self.rain = clamp(self._t, 0.0, 80.0)
        delay = -10.0 if self._increasing else 90.0
        self.puddles = clamp(self._t + delay, 0.0, 85.0)
        self.wetness = clamp(self._t * 5, 0.0, 100.0)
        self.wind = 5.0 if self.clouds <= 20 else 90 if self.clouds >= 70 else 40
        self.fog = clamp(self._t - 10, 0.0, 30.0)
        if self._t == -250.0:
            self._increasing = True
        if self._t == 100.0:
            self._increasing = False
    # 输出当前云量，降水和风强度
    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)

# 天气类，包含太阳和风暴
class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

    # 将自定义类太阳和风暴的参数传给weather
    def tick(self, delta_seconds):
        self._sun.tick(delta_seconds)
        self._storm.tick(delta_seconds)
        self.weather.cloudiness = self._storm.clouds
        self.weather.precipitation = self._storm.rain
        self.weather.precipitation_deposits = self._storm.puddles
        self.weather.wind_intensity = self._storm.wind
        self.weather.fog_density = self._storm.fog
        self.weather.wetness = self._storm.wetness
        self.weather.sun_azimuth_angle = self._sun.azimuth
        self.weather.sun_altitude_angle = self._sun.altitude

    # 输出当前天气参数
    def __str__(self):
        return '%s %s' % (self._sun, self._storm)

# 计时器
class CustomTimer:
    # 使用本机时间作为计时器，如果报错则改用浮点秒数计时
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time
    # 返回计时器
    def time(self):
        return self.timer()

# 显示控制器
class DisplayManager:
    # 根据输入窗口大小初始化现实控制器并启用硬件加速，双缓冲
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []

    # 返回整个窗口大小
    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]

    # 返回单个监视窗的大小
    def get_display_size(self):
        return [int(self.window_size[0] / self.grid_size[1]), int(self.window_size[1] / self.grid_size[0])]

    #返回单个监视窗的位置
    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]

    # 添加传感器
    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    # 获取传感器列表
    def get_sensor_list(self):
        return self.sensor_list

    # 渲染画面并将画面输出
    def render(self):
        if not self.render_enabled():
            return

        for s in self.sensor_list:
            s.render()

        pygame.display.flip()

    # 销毁传感器
    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

    # 渲染开关
    def render_enabled(self):
        return self.display != None


# 传感器控制器
class SensorManager:
    # 初始化传感器，计时器等
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.time_processing = 0.0
        self.tics_processing = 0
        self.display_man.add_sensor(self)

    # 初始化传感器方法
    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        # 当输入传感器类型为RGBCamera时
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        # 当输入传感器类型为LiDAR时
        elif sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate',
                                   lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit',
                                   lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity',
                                   lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_lidar_image)

            return lidar

        # 当输入传感器类型为SemanticLiDAR时
        elif sensor_type == 'SemanticLiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
            lidar_bp.set_attribute('range', '100')

            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)

            lidar.listen(self.save_semanticlidar_image)

            return lidar

        # 当输入传感器类型为Radar时
        elif sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)

            return radar

        # 当输入传感器类型为DepthCamera时
        elif sensor_type == "DepthCamera":
            depth_camera_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
            for key in sensor_options:
                depth_camera_bp.set_attribute(key, sensor_options[key])

            depth_camera = self.world.spawn_actor(depth_camera_bp, transform, attach_to=attached)
            depth_camera.listen(self.save_depth_image)

            return depth_camera

        # 当输入传感器类型为SemanticSegmentationCamera时
        elif sensor_type == "SemanticSegmentationCamera":
            semantic_segmentation_camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            for key in sensor_options:
                semantic_segmentation_camera_bp.set_attribute(key, sensor_options[key])

            semantic_segmentation_camera = self.world.spawn_actor(semantic_segmentation_camera_bp, transform, attach_to=attached)
            semantic_segmentation_camera.listen(self.save_semantic_segmentation_image)

            return semantic_segmentation_camera

        # 当输入传感器类型为InstanceSegmentationCamera时
        elif sensor_type == "InstanceSegmentationCamera":
            instance_segmentation_camera_bp = self.world.get_blueprint_library().find(
                'sensor.camera.instance_segmentation')
            for key in sensor_options:
                instance_segmentation_camera_bp.set_attribute(key, sensor_options[key])

            instance_segmentation_camera = self.world.spawn_actor(instance_segmentation_camera_bp, transform,
                                                                  attach_to=attached)
            instance_segmentation_camera.listen(self.save_instance_segmentation_image)

            return instance_segmentation_camera
        else:
            return None

    # 获取传感器
    def get_sensor(self):
        return self.sensor

    # 转化rgb图像，渲染并更新时间
    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 转化depth图像，渲染并更新时间
    def save_depth_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.LogarithmicDepth)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 转化semantic_segmentation图像，渲染并更新时间
    def save_semantic_segmentation_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 转化instance_segmentation图像，渲染并更新时间
    def save_instance_segmentation_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 转化lidar数据，渲染并更新时间
    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0 * float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 转化semanticlidar数据，渲染并更新时间
    def save_semanticlidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0 * float(self.sensor_options['range'])

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 6), 6))
        lidar_data = np.array(points[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 转换radar数据并更新时间
    def save_radar_image(self, radar_data):
        t_start = self.timer.time()
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar_data), 4))

        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    # 渲染屏幕输出
    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    # 摧毁传感器
    def destroy(self):
        self.sensor.destroy()


def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        client.load_world('Town10HD')
        num_walkers = 10
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
        # 设置rgb相机的方位信息
        camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
                                           carla.Rotation(pitch=10, yaw=0, roll=0))
        # 生成rgb相机并用SpringArmGhost的方式绑定到主车上
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle,
                                   attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)

        # 获得当前模拟世界的设定
        setting = world.get_settings()
        # 设定为异步模式
        setting.synchronous_mode = True
        # 将时间步长设定为固定的0.05秒
        setting.fixed_delta_seconds = 0.05
        setting.spectator_as_ego = True
        setting.tile_stream_distance = 300
        # 设定在距离主车距离小于1km时唤醒车辆
        setting.actor_active_distance = 200
        # 应用设定
        world.apply_settings(setting)

        # 将交通管理器设置为同步模式
        traffic_manager.synchronous_mode = True
        # 通过交通管理器设置所有车辆相对于限速的差值，这里为负即为所有车辆都会超速行驶
        traffic_manager.global_percentage_speed_difference(-30)
        # 开启休眠车辆重新生成模式
        traffic_manager.set_respawn_dormant_vehicles(True)
        # 设定重新生成车辆范围，为距离主车25 - 100 米
        traffic_manager.set_boundaries_respawn_dormant_vehicles(75, 150)
        # 设定存储摄像头数据的队列
        image_queue = queue.Queue()
        # 设定传感器每读取一帧数据后存储到队列中(同步模式)
        camera.listen(image_queue.put)
        # 设定数据的存储路径
        output_path = os.path.join("/home/ziyu/data/carla_pic", '%06d.png')

        # 令摄像头读取数据并存储(异步模式)
        #camera.listen(lambda image: image.save_to_disk(output_path % image.frame))


        # 显示管理器根据输入的网格数量以及窗口大小自动划分并且管理各个传感器以及在窗口中的显示
        # 在这里总共2行3列网格，整个窗口的大小是1920x1080像素
        display_manager = DisplayManager(grid_size=[2, 3], window_size=[1920,1080])

        # 传感器管理器负责根据我们输入的属性去初始化传感器并且将最终的显示输出的位置进行管理，这里初始化了
        # RGBCamera，DepthCamera，SemanticSegmentationCamera，InstanceSegmentationCamera
        # LiDAR以及SemanticLiDAR共6个传感器，具体传感器属性参数含义请参考上一篇文章
        SensorManager(world, display_manager, 'DepthCamera',
                      carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
                      ego_vehicle, {'fov':'90'}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
                      ego_vehicle, {'fov':'90'}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'SemanticSegmentationCamera',
                      carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
                      ego_vehicle, {'fov':'90'}, display_pos=[0, 2])
        SensorManager(world, display_manager, 'InstanceSegmentationCamera',
                      carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)),
                      ego_vehicle, {'fov':'90'}, display_pos=[1, 1])
        SensorManager(world, display_manager, 'LiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                      ego_vehicle,
                      {'channels': '64', 'range': '100', 'points_per_second': '250000', 'rotation_frequency': '20'},
                      display_pos=[1, 0])
        SensorManager(world, display_manager, 'SemanticLiDAR', carla.Transform(carla.Location(x=0, z=2.4)),
                      ego_vehicle,
                      {'channels': '64', 'range': '100', 'points_per_second': '100000', 'rotation_frequency': '20'},
                      display_pos=[1, 2])

        weather = Weather(world.get_weather())
        elapsed_time = 0.0
        world.get_spectator().set_transform(camera.get_transform())
        while True:
            info_text = info(ego_vehicle, world)
            update_light_state(world)
            # 从world中获取观察者视角，并将观察者视角的方位信息设置为相机的对应方位信息
            world.get_spectator().set_transform(camera.get_transform())
            display_manager.render()
            elapsed_time += 0.05
            weather.tick(elapsed_time)
            world.set_weather(weather.weather)
            sys.stdout.write('\r' + str(weather) + 12 * ' ')
            sys.stdout.write(info_text + 5 * ' ')
            sys.stdout.write(str(bin(ego_vehicle.get_light_state())) + ' ')
            speed = '%.2f' % (ego_vehicle.get_velocity().length() * 3.6)
            sys.stdout.write(str(speed) + ' km/h ')
            sys.stdout.flush()
            elapsed_time = 0.0

            # 如果为同步模式设定
            if traffic_manager.synchronous_mode:
                # 更新模拟世界
                world.tick()
                # 从队列中读取传感器图像
                image = image_queue.get()
                # 将图像存储到本地路径(同步模式)
                # image.save_to_disk(output_path % image.frame)

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

        if display_manager:
            display_manager.destroy()

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
