import math
import os.path
import queue
import sys
import threading

import carla
import random

from carla.libcarla import VehicleLightState

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def info(ego_vehicle,world):
    vehicles = world.get_actors().filter("*vehicle*")

    active_vehicle_num = 0
    ego_loc = ego_vehicle.get_transform()
    for vehicle in vehicles:
        t = vehicle.get_transform()
        get_distance = lambda l: math.sqrt(
            (l.x - t.location.x) ** 2 + (l.y - t.location.y) ** 2 + (l.z - t.location.z) ** 2)
        distance = get_distance(ego_loc.location)
        # vehicle_type = get_actor_display_name(vehicle, truncate=15)
        # _info_text.append('% 4dm %s' % (distance, vehicle_type))
        if distance < 200.0:
            active_vehicle_num += 1

    _info_text = "Vehicle status: % 8d vehicles nearby" % (active_vehicle_num - 1) + 5 * " "
    # print(active_vehicle_num)
    return _info_text

def clamp(value, minimum=0.0, maximum=100.0):
    return max(minimum, min(value, maximum))


class Sun(object):
    def __init__(self, azimuth, altitude):
        self.azimuth = azimuth
        self.altitude = altitude
        self._t = 0.0

    def tick(self, delta_seconds):
        self._t += 0.008 * delta_seconds
        self._t %= 2.0 * math.pi
        self.azimuth += 0.25 * delta_seconds
        self.azimuth %= 360.0
        self.altitude = (70 * math.sin(self._t)) - 20

    def __str__(self):
        return 'Sun(alt: %.2f, azm: %.2f)' % (self.altitude, self.azimuth)


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

    def __str__(self):
        return 'Storm(clouds=%d%%, rain=%d%%, wind=%d%%)' % (self.clouds, self.rain, self.wind)


class Weather(object):
    def __init__(self, weather):
        self.weather = weather
        self._sun = Sun(weather.sun_azimuth_angle, weather.sun_altitude_angle)
        self._storm = Storm(weather.precipitation)

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

    def __str__(self):
        return '%s %s' % (self._sun, self._storm)

def update_light_state(world):
    weather = world.get_weather()
    for vehicle in world.get_actors().filter('*vehicle*'):
        control = vehicle.get_control()
        current_lights = vehicle.get_light_state()
        if weather.sun_altitude_angle < 0:
            current_lights |= carla.VehicleLightState.Position
            current_lights |= carla.VehicleLightState.HighBeam
            current_lights |= carla.VehicleLightState.Interior
        if weather.sun_altitude_angle > 0:
            current_lights &= 0b11011111011
        if weather.fog_density > 30 or weather.precipitation > 30:
            current_lights |= carla.VehicleLightState.Position
            current_lights |= carla.VehicleLightState.LowBeam
            current_lights |= carla.VehicleLightState.Fog
        if weather.fog_density < 30 and weather.precipitation < 30:
            current_lights &= 0b11101111101
        if control.brake > 0.1:
            current_lights |= carla.VehicleLightState.Brake
        if control.brake <= 0.1:
            current_lights &= 0b11111110111
        if control.steer < 0:
            current_lights |= carla.VehicleLightState.LeftBlinker
        if control.steer > 0:
            current_lights |= carla.VehicleLightState.RightBlinker
        if abs(control.steer) < 0.1:
            current_lights &= 0b11111001111
        if control.reverse:
            current_lights |= carla.VehicleLightState.Reverse
        if not control.reverse:
            current_lights &= 0b11110111111

        vehicle.set_light_state(VehicleLightState(current_lights))



def main():
    try:
        # setup client并且加载我们所需要的地图
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        client.load_world('Town12')
        num_walkers = 200
        num_vehicle = 50
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
        # for spwan_point in vehicle_spawn_points:
        #     print(spwan_point.location)

        # 通过world获得所有可以生成行人的地点并存储
        ped_spawn_points = []
        for i in range(num_walkers):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                ped_spawn_points.append(spawn_point)

        # 在地图上随机生成num_vehicle辆车，每辆车为车辆蓝图库中的随机车辆
        for i in range(0, num_vehicle):
            world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(vehicle_spawn_points))

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
            walker_ai_batch.append(world.try_spawn_actor(walker_ai_blueprint, carla.Transform(), walker))

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

        ego_spawn_point = random.choice(vehicle_spawn_points)

        ego_bp = world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')

        ego_bp.set_attribute('role_name', 'hero')

        ego_vehicle = world.spawn_actor(ego_bp, ego_spawn_point)

        # 获得车辆的中心点，并将点定位到右上角，便于运算
        bound_x = 0.25 + ego_vehicle.bounding_box.extent.x
        bound_y = 0.25 + ego_vehicle.bounding_box.extent.y
        bound_z = 0.25 + ego_vehicle.bounding_box.extent.z

        trasform_camera_record = carla.Transform(carla.Location(x=bound_x, y=0, z=bound_z),
                         carla.Rotation(pitch=-15, yaw=180, roll=0))

        # 从蓝图库中寻找rgb相机
        camera_bp_record = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp_record.set_attribute('image_size_x',str(800))
        camera_bp_record.set_attribute('image_size_y',str(600))

        # 生成rgb相机并用SpringArmGhost的方式绑定到主车上
        camera_record = world.spawn_actor(camera_bp_record, trasform_camera_record,
                                   attach_to=ego_vehicle, attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)


        trafficManager = client.get_trafficmanager()
        trafficManager.synchronous_mode = True
        trafficManager.set_desired_speed(ego_vehicle,90)
        # 开启休眠车辆重新生成模式
        trafficManager.set_respawn_dormant_vehicles(True)
        # 设定重新生成车辆范围，为距离主车25 - 700 米
        trafficManager.set_boundaries_respawn_dormant_vehicles(25, 200)
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        # 设定在距离主车距离小于2km时唤醒车辆
        settings.actor_active_distance = 1000
        # 应用设定
        world.apply_settings(settings)

        # Create a transform to place the camera on top of the vehicle
        transform_ego = ego_vehicle.get_transform()
        camera_init_trans = carla.Transform(ego_spawn_point.location + carla.Location(z=2))
        spectator.set_transform(carla.Transform(ego_spawn_point.location + carla.Location(z=2)))

        ego_vehicle.set_autopilot()

        # 从蓝图库中寻找rgb相机
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        # 设置rgb相机的方位信息
        camera_transform = carla.Transform(carla.Location(x=-8, y=0, z=5),
                                           carla.Rotation(pitch=10, yaw=0, roll=0))
        # 生成rgb相机并用SpringArmGhost的方式绑定到主车上
        camera = world.spawn_actor(camera_bp, camera_transform,
                                   attach_to=ego_vehicle, attachment_type=carla.libcarla.AttachmentType.SpringArmGhost)

        image_queue = queue.Queue()
        camera_record.listen(image_queue.put)

        output_path = os.path.join("/home/ziyu/data/carla_pic",'%06d.png')

        weather = Weather(world.get_weather())
        elapsed_time = 0.0


        while True:
            info_text = info(ego_vehicle, world)
            sys.stdout.write(info_text)
            sys.stdout.flush()
            # 从world中获取观察者视角，并将观察者视角的方位信息设置为相机的对应方位信息
            update_light_state(world)
            sys.stdout.write(str(bin(ego_vehicle.get_light_state())) + ' ')
            sys.stdout.flush()
            sys.stdout.write(str(ego_vehicle.get_velocity().length()*3.6) + ' km/h ')
            sys.stdout.flush()
            world.get_spectator().set_transform(camera.get_transform())
            if trafficManager.synchronous_mode:
                world.tick()
                image = image_queue.get()
                #image.save_to_disk(output_path % image.frame)
                elapsed_time += 0.05
                weather.tick(elapsed_time)
                world.set_weather(weather.weather)
                sys.stdout.write('\r' + str(weather) + 12 * ' ')
                sys.stdout.flush()
                elapsed_time = 0.0
            else:
                world.wait_for_tick()
    finally:
        for controller in world.get_actors().filter('*controller*'):
            controller.stop()
        for vehicle in world.get_actors().filter('*vehicle*'):
            vehicle.destroy()
        for walker in world.get_actors().filter('*walker*'):
            walker.destroy()


        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
