#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7AA')

logging.basicConfig(level=logging.INFO)

def is_close(d):
    return d is not None and d < 0.25

if __name__ == "__main__":
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:

        # -----------------------------
        # Configuración log Flowdeck (posición estimada)
        # -----------------------------
        log_pos = LogConfig(name='pos', period_in_ms=50)
        log_pos.add_variable('stateEstimate.x', 'float')
        log_pos.add_variable('stateEstimate.y', 'float')
        log_pos.add_variable('stateEstimate.z', 'float')

        pos = {"x": 0, "y": 0, "z": 0}

        def cb_pos(ts, data, logconf):
            pos["x"] = data["stateEstimate.x"]
            pos["y"] = data["stateEstimate.y"]
            pos["z"] = data["stateEstimate.z"]

        scf.cf.log.add_config(log_pos)
        log_pos.data_received_cb.add_callback(cb_pos)
        log_pos.start()

        # -----------------------------
        # Multiranger
        # -----------------------------
        with Multiranger(scf) as mr:

            print("Logging de sensores activo (cada 2 segundos). Presiona Ctrl+C para detener.")

            try:
                while True:
                    print(
                        f"Posición → x: {pos['x']:.2f}, y: {pos['y']:.2f}, z: {pos['z']:.2f} | "
                        f"Front: {mr.front}, Back: {mr.back}, Left: {mr.left}, Right: {mr.right}, Up: {mr.up}, Down: {mr.down}"
                    )
                    time.sleep(2.0)  # pausa de 2 segundos entre medidas
            except KeyboardInterrupt:
                print("Logging detenido por el usuario.")

        log_pos.stop()
