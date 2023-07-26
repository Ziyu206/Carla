# -- coding:UTF-8 --
# !/usr/bin/env python
# Author:Kin Zhang
# email: kin_eng@163.com

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls
import argparse
import logging
from numpy import random


def main():
    synchronous_master = False
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()

        # 拿到这个世界所有物体的蓝图
        blueprint_library = world.get_blueprint_library()
        # 从浩瀚如海的蓝图中找到奔驰的蓝图
        ego_vehicle_bp = blueprint_library.find('vehicle.audi.a2')
        # 给我们的车加上特定的颜色
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')

        # 找到所有可以作为初始点的位置并随机选择一个
        # transform = random.choice(world.get_map().get_spawn_points())

        # 设置固定点
        transform = carla.Transform(carla.Location(x=-9, y=80, z=2), carla.Rotation(yaw=90))
        # 在这个位置生成汽车
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)

        while True:
            if synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()
    finally:
        # 如果设置了同步记得 设置回去异步否则下次无法开启同步
        if synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        print('\ndestroying vehicles')
        ego_vehicle.destroy()
        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
