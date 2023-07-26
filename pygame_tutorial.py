import carla
import random
import pygame
import numpy as np

# 连接到客户端并检索世界对象
client = carla.Client('localhost', 2000)
world = client.get_world()

# 在同步模式下设置模拟器
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# 以同步模式设置TM
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)

# 设置种子，以便必要时行为可以重复
traffic_manager.set_random_device_seed(0)
random.seed(0)

# 获取地图的刷出点
spawn_point = random.choice(world.get_map().get_spawn_points())

# 生成车辆并设置自动驾驶
vehicle_bp = random.choice(world.get_blueprint_library().filter('*vehicle*'))
ego_vehicle = world.spawn_actor(vehicle_bp, spawn_point)
ego_vehicle.set_autopilot(True)

traffic_manager.ignore_lights_percentage(ego_vehicle, 100)

# 渲染对象来保持和传递PyGame表面
class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

# 相机传感器回调，将相机的原始数据重塑为2D RGB，并应用于PyGame表面
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))

# 控件对象来管理车辆控件
class ControlObject(object):
    def __init__(self, veh):

        # 控制参数来存储控制状态
        self._vehicle = veh
        self._steer = 0
        self._throttle = False
        self._brake = False
        self._steer = None
        self._steer_cache = 0
        self._control = carla.VehicleControl()
        self._autopilot = True

    # 检查PyGame窗口中的按键事件
    # 定义控件状态
    def parse_control(self, event):
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RETURN:
                self._autopilot = not self._autopilot
                self._vehicle.set_autopilot(self._autopilot)
            if event.key == pygame.K_UP:
                self._throttle = True
            if event.key == pygame.K_DOWN:
                self._brake = True
            if event.key == pygame.K_RIGHT:
                self._steer = 1
            if event.key == pygame.K_LEFT:
                self._steer = -1
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

    # 处理当前控制状态，改变控制参数
    def process_control(self):

        if self._throttle:
            self._control.throttle = min(self._control.throttle + 0.01, 0.5)
            self._control.gear = 1
            self._control.brake = False
        elif not self._brake:
            self._control.throttle = 0.0

        if self._brake:
            # 如果在汽车静止时按住向下箭头，则切换到倒车
            if self._vehicle.get_velocity().length() < 0.01 and not self._control.reverse:
                self._control.brake = 0.0
                self._control.gear = 1
                self._control.reverse = True
                self._control.throttle = min(self._control.throttle + 0.1, 0.5)
            elif self._control.reverse:
                self._control.throttle = min(self._control.throttle + 0.1, 0.5)
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

        # Ápply小车的控制参数
        self._vehicle.apply_control(self._control)

# 初始化安装在车辆后面的摄像头
camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', str(960))
camera_bp.set_attribute('image_size_y', str(540))

camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego_vehicle)

# 用PyGame回调启动camera
camera.listen(lambda image: pygame_callback(image, renderObject))

# 设置渲染画面size
image_w = 960
image_h = 540

# 为渲染和车辆控制实例化对象
renderObject = RenderObject(image_w, image_h)
renderObject1 = RenderObject(image_w, image_h)

controlObject = ControlObject(ego_vehicle)

# 初始化显示
pygame.init()
gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
# 填充黑色背景
gameDisplay.fill((0,0,0))
gameDisplay.blit(renderObject.surface, (0,0))
pygame.display.flip()


# 循环执行
crashed = False

while not crashed:
    # 等待同步
    world.tick()
    # 按帧更新渲染的Camera画面
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
    # 处理车辆控制请求
    controlObject.process_control()
    # 获取pygame事件
    for event in pygame.event.get():
        # If the window is closed, break the while loop
        if event.type == pygame.QUIT:
            crashed = True
        # 获得pygame控制车辆键盘事件
        controlObject.parse_control(event)

# 结束
ego_vehicle.destory()
camera.stop()
pygame.quit()
