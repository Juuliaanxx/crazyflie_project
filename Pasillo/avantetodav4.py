"""
Crazyflie - Wall following + evitación frontal + parada de emergencia

Modos:
- WALL_FOLLOW
- FRONT_AVOID_DECIDE
- FRONT_AVOID_MOVE
- FRONT_AVOID_EXTRA
"""

import time
import logging

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper

# ================= CONFIGURACIÓN =================
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7AA')

# Velocidades
VX = 0.05                  # avance adelante
LOOP_DT = 0.1              # periodo de control
FLIGHT_TIME = 30           # tiempo total de vuelo (s)

# Wall follow
KP = 0.05
MAX_VY = 0.25
DEADBAND = 0.10            # 10 cm

# Evitación frontal
FRONT_THRESHOLD = 0.5
AVOID_VY = 0.1
EXTRA_FORWARD_TIME = 0.5   # ~10 cm extra
# =================================================

logging.basicConfig(level=logging.ERROR)

# ================= WALL FOLLOW MODE =================
def wall_follow_mode(left, right, last_left, last_right,
                     kp=KP, max_vy=MAX_VY, deadband=DEADBAND):

    if left is not None:
        last_left = left
    if right is not None:
        last_right = right

    if last_left is None or last_right is None:
        return 0.0, last_left, last_right, 0.0

    error = last_left - last_right

#    if abs(error) < deadband:
#        return 0.0, last_left, last_right, error

    vy = kp * error
    #vy = max(-max_vy, min(max_vy, vy))

    return vy, last_left, last_right, error


# ================= FUNCION EVITACIÓN FRONTAL =================
def front_obstacle_avoidance(front, left, right,
                             last_left, last_right,
                             mode, avoid_direction, extra_forward_start,
                             front_threshold=FRONT_THRESHOLD,
                             avoid_vy=AVOID_VY,
                             extra_forward_time=EXTRA_FORWARD_TIME,
                             vx_cruise=VX):
    """
    Gestiona la lógica de evitación de obstáculos frontales.

    Returns:
        vx, vy, mode, avoid_direction, extra_forward_start, last_left, last_right
    """

    vx = vx_cruise
    vy = 0.0

    # ================= WALL FOLLOW → DECISIÓN =================
    if mode == "WALL_FOLLOW":
        if front is not None and front < front_threshold:
            mode = "FRONT_AVOID_DECIDE"
            vx = -0.1
            print("⚠️ Obstáculo frontal → decidiendo dirección")
    #REVISAR Y PONER TIEMPO DE ESPERA ANTES DE DECIDIR PARA LUCHAR CONTRA LA INERCIA
    # ================= DECISIÓN =================
    elif mode == "FRONT_AVOID_DECIDE":
        vx = 0.0

        # Actualizar últimas medidas válidas
        if left is not None:
          last_left = left
        if right is not None:
            last_right = right

        # Actualizar últimas medidas válidas
        if left is None:
            last_left = 999.0  # Asumir muy lejos
        if right is None:
            last_right = 998.0  # Asumir muy lejos

        if last_left is not None and last_right is not None:
            avoid_direction = 1 if last_left > last_right else -1
            mode = "FRONT_AVOID_MOVE"
            print(
                "➡️ Evitando hacia",
                "IZQUIERDA" if avoid_direction == 1 else "DERECHA"
            )

    # ================= MOVIMIENTO LATERAL =================
    elif mode == "FRONT_AVOID_MOVE":
        vy = avoid_direction * avoid_vy
        vx = 0.0
        # Cuando deja de ver obstáculo frontal
        if front is None or front > front_threshold:
        #    vx = vx_cruise
        #    vy = 0.0
            extra_forward_start = time.time()
            mode = "FRONT_AVOID_EXTRA"
            print("✅ Obstáculo superado → avanzando extra")

    # ================= AVANCE EXTRA =================
    elif mode == "FRONT_AVOID_EXTRA":
        if time.time() - extra_forward_start < extra_forward_time:
            vx = vx_cruise
            vy = 0.0
        else:
            mode = "WALL_FOLLOW"
            print("🔄 Volviendo a wall follow")

    return vx, vy, mode, avoid_direction, extra_forward_start, last_left, last_right

def landing_conditions(up, time):
    if up is not None and up < 0.25:  # menos de 25 cm arriba → stop
        mode = "LANDING"   
    if time > FLIGHT_TIME - 5:  # menos de 5 segundos restantes → stop
        mode = "LANDING"

# ================= MAIN =================
if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)

        with MotionCommander(scf) as mc:
            with Multiranger(scf) as mr:

                last_left = None
                last_right = None

                mode = "WALL_FOLLOW"
                avoid_direction = 0
                extra_forward_start = None

                start_time = time.time()
                print("🚀 Despegado")

                while time.time() - start_time < FLIGHT_TIME:

                    # ================= SENSORES =================
                    front = mr.front
                    left = mr.left
                    right = mr.right
                    up = mr.up

                    # ================= PARADA DE EMERGENCIA =================
                    if up is not None and up < 0.25:  # menos de 25 cm arriba → stop
                        mc.start_linear_motion(0.0, 0.0, 0.0)
                        print("🛑 EMERGENCIA: objeto detectado arriba, parada inmediata")
                        break

                    # ------------------- EVITACIÓN FRONTAL -------------------
                    vx, vy, mode, avoid_direction, extra_forward_start, last_left, last_right = \
                        front_obstacle_avoidance(front, left, right,
                                                 last_left, last_right,
                                                 mode, avoid_direction,
                                                 extra_forward_start)

                    # ------------------- WALL FOLLOW -------------------
                    if mode == "WALL_FOLLOW":
                        vy, last_left, last_right, _ = wall_follow_mode(
                            left, right, last_left, last_right
                        )

                    # -------------------ATERRIZAJE-------------------------------
                    landing_conditions(up, time.time())

                    if mode == "LANDING":
                        mc.start_linear_motion(0.0, 0.0, 0.0)
                        print("🛬 Aterrizando")
                        break
                    # ------------------- COMANDO DE MOVIMIENTO -------------------
                    mc.start_linear_motion(vx, vy, 0.0)

                    # Debug
                    print(
                        f"time={time.time() - start_time:.4f} mode={mode} front={front if front else None} up={up if up else None} left={left if left else None} right={right if right else None} vx={vx:.2f} vy={vy:.4f}"
                    )

                    time.sleep(LOOP_DT)

                print("🛑 Tiempo cumplido o parada de emergencia, aterrizando")

        print("✅ Demo terminada")
