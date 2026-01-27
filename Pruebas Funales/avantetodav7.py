import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

CONF = {
    "URI": 'radio://0/80/2M/E7E7E7E7AA',
    "CRUISE_VEL": 0.07,
    "DODGE_VEL": 0.35,
    "BRAKE_VEL": -0.10,
    "DODGE_DURATION": 1.0,   # duración de la esquiva en segundos
    "TARGET_DIST": 10.0,
    "SAFE_DIST": 0.8,         # detección anticipada de obstáculos
    "KP": 0.3,
    "KD": 0.2,
    "ALPHA": 0.7
}

MIN_WALL_DIST = 0.15  # margen mínimo para no acercarse demasiado a la pared

class SmartDroneController:
    def __init__(self):
        self.last_error = 0.0
        self.last_time = time.time()
        self.dist_x = 0.0
        self.vals = {"left": 0.5, "right": 0.5}

        # Estado de esquiva persistente
        self.dodge_until = 0.0
        self.dodge_dir = 0  # 1 izquierda, -1 derecha

    def filter_val(self, key, new_val):
        prev = self.vals.get(key)
        if new_val is None:
            return prev
        smoothed = (CONF["ALPHA"] * new_val) + (1 - CONF["ALPHA"]) * prev
        self.vals[key] = smoothed
        return smoothed

    def get_velocities(self, mr):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        l_wall = self.filter_val("left", mr.left)
        r_wall = self.filter_val("right", mr.right)
        front_dist = mr.front if mr.front is not None else 2.0

        # ---------- LÓGICA DE DECISIÓN ----------

        if front_dist < CONF["SAFE_DIST"]:
            # FASE 1: Obstáculo detectado → frenar y esquivar
            self.dodge_until = max(self.dodge_until, now + CONF["DODGE_DURATION"])
            self.dodge_dir = 1 if l_wall > r_wall else -1

            vel_x = CONF["BRAKE_VEL"]
            vel_y = CONF["DODGE_VEL"] * self.dodge_dir

            # Evitar acercarse demasiado a las paredes
            if self.dodge_dir == 1 and r_wall < MIN_WALL_DIST:
                vel_y = 0.0
            elif self.dodge_dir == -1 and l_wall < MIN_WALL_DIST:
                vel_y = 0.0

            msg = "¡FRENANDO Y ESQUIVANDO!"

        elif now < self.dodge_until:
            # FASE 2: Rebase → mantener esquiva aunque el frente esté libre
            vel_x = CONF["CRUISE_VEL"] * 0.7
            vel_y = CONF["DODGE_VEL"] * self.dodge_dir

            # Evitar acercarse demasiado a las paredes
            if self.dodge_dir == 1 and r_wall < MIN_WALL_DIST:
                vel_y = 0.0
            elif self.dodge_dir == -1 and l_wall < MIN_WALL_DIST:
                vel_y = 0.0

            msg = "REBASANDO OBSTÁCULO"

        else:
            # FASE 3: Crucero normal → centrado PD
            vel_x = CONF["CRUISE_VEL"]
            error = l_wall - r_wall
            deriv = (error - self.last_error) / dt if dt > 0 else 0.0
            vel_y = (CONF["KP"] * error) + (CONF["KD"] * deriv)
            self.last_error = error
            msg = "CRUCERO: CENTRANDO"

        if vel_x > 0:
            self.dist_x += vel_x * dt

        return vel_x, vel_y, msg

def run_navigation(scf):
    ctrl = SmartDroneController()

    with MotionCommander(scf) as mc:
        with Multiranger(scf) as mr:
            print(f"Navegación con esquiva persistente ({CONF['DODGE_DURATION']}s) y detección anticipada...")

            while True:
                if mr.up and mr.up < 0.15:
                    print("Parada manual.")
                    break

                vx, vy, modo = ctrl.get_velocities(mr)

                if ctrl.dist_x >= CONF["TARGET_DIST"]:
                    print("Meta alcanzada.")
                    break

                mc.start_linear_motion(vx, vy, 0)

                f_val = mr.front if mr.front else 2.0
                print(f"[{modo:25}] Dist: {ctrl.dist_x:4.2f} m | Front: {f_val:4.2f} m")

                time.sleep(0.1)

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    logging.basicConfig(level=logging.ERROR)

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(CONF["URI"], cf=cf) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        run_navigation(scf)
