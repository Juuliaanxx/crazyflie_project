#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Crazyflie Follow Demo – mantiene distancia relativa a la persona más cercana
# Requiere Flowdeck + Multiranger
#
# Autor: Adaptado para comportamiento de seguimiento

import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

import os
import platform


# Cambia el canal si es necesario
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7AA')

if len(sys.argv) > 1:
    URI = sys.argv[1]

logging.basicConfig(level=logging.ERROR)


# ---- Parámetros de distancia ----
DESIRED_DISTANCE = 0.5     # Distancia objetivo con respecto al objeto (m)
MIN_DISTANCE = 0.3         # Si está más cerca que esto, se aleja (m)
MAX_DISTANCE = 1.0         # Si está más lejos que esto, se acerca (m)
VELOCITY = 0.3             # Velocidad base (m/s)
UPDATE_RATE = 0.1          # Tiempo entre actualizaciones (s)


def get_nearest_direction(multiranger):
    """Devuelve la dirección más cercana detectada y su distancia"""
    distances = {
        "front": multiranger.front,
        "back": multiranger.back,
        "left": multiranger.left,
        "right": multiranger.right
    }

    # Filtra None
    distances = {k: v for k, v in distances.items() if v is not None}

    if not distances:
        return None, None

    # Encuentra la dirección con menor distancia
    direction = min(distances, key=distances.get)
    return direction, distances[direction]


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')

    with SyncCrazyflie(URI, cf=cf) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        with MotionCommander(scf, default_height=0.3) as motion_commander:
            with Multiranger(scf) as multiranger:
                print("Seguimiento iniciado. Coloca tu mano u objeto frente al dron.")
                print("Ctrl+C para detener.")

                try:
                    while True:
                        direction, distance = get_nearest_direction(multiranger)
                        vx, vy = 0.0, 0.0

                        # Detección superior → detener demo
                        if  multiranger.up < 0.25:
                            print("Objeto detectado arriba. Deteniendo vuelo.")
                            break

                        if direction is not None and distance is not None:
                            if distance > MAX_DISTANCE:
                                # Muy lejos → acercarse
                                if direction == "front":
                                    vx = VELOCITY
                                elif direction == "back":
                                    vx = -VELOCITY
                                elif direction == "left":
                                    vy = VELOCITY
                                elif direction == "right":
                                    vy = -VELOCITY
                            elif distance < MIN_DISTANCE:
                                # Muy cerca → alejarse
                                if direction == "front":
                                    vx = -VELOCITY
                                elif direction == "back":
                                    vx = VELOCITY
                                elif direction == "left":
                                    vy = -VELOCITY
                                elif direction == "right":
                                    vy = VELOCITY
                            else:
                                # Dentro de la zona deseada → mantener
                                vx, vy = 0.0, 0.0

                            print(f"Dir: {direction:>5} | Dist: {distance:.2f} m | v=({vx:.2f}, {vy:.2f})")

                        motion_commander.start_linear_motion(vx, vy, 0.0)
                        time.sleep(UPDATE_RATE)

                except KeyboardInterrupt:
                    print("\nInterrupción manual: deteniendo vuelo.")
            
            # Reproducir sonido al finalizar
            audio_file = "fin.mp3"  # Cambia por la ruta de tu archivo
            print("Reproduciendo audio:", audio_file)
            os.system(f'start {audio_file}')
            print("Demo finalizada.")

