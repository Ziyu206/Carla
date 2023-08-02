import math
import sys

import carla
import random
import pygame
import numpy as np
from carla import VehicleLightState

# 连接到客户端并检索世界对象
client = carla.Client('localhost', 2000)
world = client.get_world()
weather = carla.WeatherParameters(
    cloudiness=80.0,
    precipitation=30.0,
    sun_altitude_angle=70.0)

world.set_weather(weather)

print(world.get_weather())

# 将模拟环境设定为同步模式
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 将交通管理器设为同步模式
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

# 设置种子，以便必要时行为可以重复
traffic_manager.set_random_device_seed(0)
random.seed(0)

# 设置观察者视角
spectator = world.get_spectator()

# 获取地图的生成点
spawn_points = world.get_map().get_spawn_points()

# 选择车辆模型范围
models = ['dodge', 'audi', 'model3', 'mini', 'mustang', 'lincoln', 'prius', 'nissan', 'crown', 'impala']
blueprints = []
for vehicle in world.get_blueprint_library().filter('*vehicle*'):
    if any(model in vehicle.id for model in models):
        blueprints.append(vehicle)

# 设置车辆生成数量上限
max_vehicles = 30
max_vehicles = min([max_vehicles, len(spawn_points)])
vehicles = []

# 在随机生成点位生成随机车辆
for i in range(0,max_vehicles):
    spawn_point = random.choice(spawn_points)
    temp = world.try_spawn_actor(random.choice(blueprints), spawn_point)
    if temp is not None:
        vehicles.append(temp)

# 将生成的车辆设定为自动驾驶模式
for vehicle in vehicles:
    vehicle.set_autopilot(True)
    # 将车辆闯红灯的可能性设定为随机0-50%
    traffic_manager.ignore_lights_percentage(vehicle, 50)

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
        return '%s %s' % (self._sun, self._storm)# 限值函数，将输入的参数限制在0到100中间后输出
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
# 相机传感器回调，将相机的原始数据重塑为2D RGB，并应用于PyGame表面
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))


# Render object to keep and pass the PyGame surface
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))


# 管理主车的控制
class ControlObject(object):
    def __init__(self, veh):

        # 设定主车控制属性的初始值
        self._vehicle = veh
        self._steer = 0
        self._throttle = False
        self._brake = False
        self._steer = None
        self._steer_cache = 0
        self._autopilot = True
        # 需要一个carla.VehicleControl实例来改变主车控制状态
        self._control = carla.VehicleControl()


    # 检测键盘输入并且根据输入来设定主车各个控制属性的状态
    def parse_control(self, event):
        if event.type == pygame.KEYDOWN:
            # 当按下回车键时切换自动驾驶状态
            if event.key == pygame.K_RETURN:
                self._autopilot = not self._autopilot
                self._vehicle.set_autopilot(self._autopilot)
            # 按下方向键上键时将油门属性设定为True
            if event.key == pygame.K_UP:
                self._throttle = True
            # 按下方向键下键时将刹车属性设定为True
            if event.key == pygame.K_DOWN:
                self._brake = True
            # 按下方向键右键时将方向盘属性设定为1
            if event.key == pygame.K_RIGHT:
                self._steer = 1
            # 按下方向键左键时将方向盘属性设定为-1
            if event.key == pygame.K_LEFT:
                self._steer = -1

        # 抬起按键时将属性重设为默认值
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                self._throttle = False
            if event.key == pygame.K_DOWN:
                self._brake = False
                self._control.reverse = False
            if event.key == pygame.K_RIGHT:
                self._steer = None
            if event.key == pygame.K_LEFT:
                self._steer = None

    # 将当前控制属性转化为carla.VehicleControl()的控制信息以应用
    def process_control(self):
        if self._throttle:
            self._control.throttle = min(self._control.throttle + 0.05, 1)
            self._control.gear = 1
            self._control.brake = False
        elif not self._brake:
            self._control.throttle = 0.0

        if self._brake:
            # 当按住方向下键并且车辆处于静止状态，切换到倒车档位并加速
            if self._vehicle.get_velocity().length() < 0.01 and not self._control.reverse:
                self._control.brake = 0.0
                self._control.gear = 1
                self._control.reverse = True
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            elif self._control.reverse:
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            else:
                self._control.throttle = 0.0
                self._control.brake = min(self._control.brake + 0.3, 1)
        else:
            self._control.brake = 0.0

        if self._steer is not None:
            if self._steer == 1:
                self._steer_cache += 0.03
            if self._steer == -1:
                self._steer_cache -= 0.03
            min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache,1)
        else:
            if self._steer_cache > 0.0:
                self._steer_cache *= 0.2
            if self._steer_cache < 0.0:
                self._steer_cache *= 0.2
            if 0.01 > self._steer_cache > -0.01:
                self._steer_cache = 0.0
            self._control.steer = round(self._steer_cache,1)

        # 将存储在self._control的控制信息应用
        self._vehicle.apply_control(self._control)

# 为主车随机选择一个蓝图
ego_vehicle = random.choice(vehicles)

# 生成绑定到主车的摄像头
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x','1280')
camera_bp.set_attribute('image_size_y','720')

camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

# 设定camera读取数据后调用回调函数
camera.listen(lambda image: pygame_callback(image, renderObject))

# 获得相机画面尺寸
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# 初始化渲染的物体和控制的物体
renderObject = RenderObject(image_w, image_h)
controlObject = ControlObject(ego_vehicle)

# 初始化pygame视窗
pygame.init()
gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
gameDisplay.fill((0,0,0))
gameDisplay.blit(renderObject.surface, (0,0))
pygame.display.flip()

# 初始化weather
weather = Weather(world.get_weather())
# 将时间步长设置为0
elapsed_time = 0.0

# Game loop
crashed = False

while not crashed:
    # 更新模拟环境时间
    world.tick()
    update_light_state(world)

    elapsed_time += 0.05
    weather.tick(elapsed_time)
    world.set_weather(weather.weather)
    sys.stdout.write('\r' + str(weather) + 12 * ' ')
    elapsed_time = 0.0

    # 更新pygame现实
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
    # 应用当前控制状态
    controlObject.process_control()
    # 监听按键信息
    for event in pygame.event.get():
        # 窗口关闭则退出循环
        if event.type == pygame.QUIT:
            crashed = True

        # 将监听到的按键信息转化成控制信息
        controlObject.parse_control(event)
        if event.type == pygame.KEYUP:
            # 按下tab键切换到其他车辆
            if event.key == pygame.K_TAB:
                ego_vehicle.set_autopilot(True)
                ego_vehicle = random.choice(vehicles)
                # 确保车辆仍存活
                if ego_vehicle.is_alive:
                    # 清除原本摄像头
                    camera.stop()
                    camera.destroy()

                    # 生成新摄像头并绑定到新主车上
                    controlObject = ControlObject(ego_vehicle)
                    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)
                    camera.listen(lambda image: pygame_callback(image, renderObject))

                    # 更新pygame窗口
                    gameDisplay.fill((0,0,0))
                    gameDisplay.blit(renderObject.surface, (0,0))
                    pygame.display.flip()

# 停止摄像头并退出pygame
# 销毁所有车辆
for vehicle in world.get_actors().filter('*vehicle*'):
    vehicle.destroy()
# 获得当前模拟世界设定
settings = world.get_settings()
# 设定为异步模式
settings.synchronous_mode = False
# 设定为可变时间步长
settings.fixed_delta_seconds = None
# 应用设定
world.apply_settings(settings)
camera.stop()
pygame.quit()
