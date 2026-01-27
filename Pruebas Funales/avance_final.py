#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Crazyflie: seguir recto centrado y esquivar obstáculos (MultiRanger)

Este script combina:
- Control de centrado en pasillo (equidistancia L/R) con PD y filtros.
- Evitación reactiva tipo "push" usando el Multi-ranger (front/back/left/right).

Idea de fusión:
- vx = VELOCITY_X_CRUISE (trayectoria recta) + vx_avoid (repulsión)
- vy = vy_center (PD centrado) + vy_avoid (repulsión)
- Saturación y limitador de aceleración lateral para evitar tirones.
- Parada de demo con la mano encima (sensor up).

Hardware: Crazyflie 2.x + Crazyradio PA + Flow deck + Multi-ranger deck
"""

import logging
import sys
import time
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.log import LogConfig

# =====================
# Parámetros principales
# =====================

# Control lateral (centrado L/R)
KP = 0.3
KD = 0.2
ALPHA = 0.6                 # suavizado exponencial para medidas L/R
MIN_LAT_SPEED = 0.03        # mínimo para vencer inercias
MAX_LAT_VELOCITY = 0.5      # límite absoluto de vy
DERIVATIVE_FILTER_ALPHA = 0.6
MAX_LAT_ACCEL = 1.0         # limitador de aceleración lateral (m/s^2)

# --- Avance recto ---
VELOCITY_X_CRUISE = 0.075 #0.05    # m/s
TARGET_DISTANCE_X = 5.0 # 10.0    # m (si stateEstimate.x está disponible, se usa dist en el plano)

# --- Logging del estimador (para parar por distancia) ---
LOG_PERIOD_MS = 50

# --- Detección de saltos (pasillos abiertos/cambios bruscos) ---
JUMP_ERROR_THRESHOLD = 0.5
JUMP_MEASURE_THRESHOLD = 0.5
JUMP_IGNORE_TIME = 2.0#1.0
KP_JUMP_SCALE = 0.2
SLOW_VX_IN_JUMP = 0.02

# --- Seguridad general ---
MIN_DISTANCE_HAND_UP = 0.2          # “mano encima”
MIN_SAFE_DISTANCE_WALL = 0.30       # si <, prioriza alejarse de pared
MIN_SAFE_DISTANCE_FRONT = 0.30      # si <, no empujar hacia delante

# --- Modo amortiguado (menos sobreoscilación) ---
DAMPED_MODE = True
DAMPED_KP_SCALE = 0.6
DAMPED_KD_SCALE = 2.0
DAMPED_DERIV_ALPHA = 0.85
DAMPED_MAX_LAT_ACCEL = 0.5

# --- Evitación reactiva ("push") ---
AVOID_ENABLE = True
AVOID_DISTANCE = 0.25        # umbral (m): empieza repulsión
AVOID_MAX_VEL = 0.35         # límite de velocidad añadida por evitación (m/s)

# --- Rodeo de obstáculo frontal (sin retroceso) ---
BYPASS_ENABLE = True
BYPASS_FRONT_TRIGGER = AVOID_DISTANCE       # empieza a rodear si front < este umbral
BYPASS_FRONT_CLEAR = AVOID_DISTANCE * 1.3   # deja de rodear cuando front > este umbral (histeresis)
BYPASS_VY = 0.25                            # velocidad lateral fija durante el rodeo (m/s)
BYPASS_VX_SCALE = 0.6                       # reduce vx_base durante el rodeo (0..1)
BYPASS_CENTER_BLEND = 0.2                   # cuánto centrado mantener durante el rodeo (0..1)
AVOID_GAIN = 1.2             # (m/s)/m  => v = gain*(threshold - range)

# --- Mantenimiento de posición/seguridad lateral ---
POSITION_MAINTENANCE = True
KP_POSITION = 0.4            # para “empuje” lateral de seguridad cuando está muy cerca

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7AA')
if len(sys.argv) > 1:
    URI = sys.argv[1]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


def is_close(range_value, threshold):
    if range_value is None:
        return False
    return range_value < threshold


def setup_logging(scf):
    """Configura logging de stateEstimate.x/y (si está disponible)."""
    state = {'x': None, 'y': None}

    log_conf = LogConfig(name='State', period_in_ms=LOG_PERIOD_MS)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')

    def _state_cb(timestamp, data, logconf):
        state['x'] = data.get('stateEstimate.x', state['x'])
        state['y'] = data.get('stateEstimate.y', state['y'])

    log_conf.data_received_cb.add_callback(_state_cb)
    scf.cf.log.add_config(log_conf)
    try:
        log_conf.start()
    except Exception:
        pass

    return state, log_conf


def smooth_distance(current, previous, alpha):
    if current is None:
        return previous
    if previous is None:
        return current
    return alpha * current + (1.0 - alpha) * previous


def process_multiranger_lr(left, right, last_left, last_right):
    prev_left = last_left
    prev_right = last_right

    last_left = smooth_distance(left, last_left, ALPHA)
    last_right = smooth_distance(right, last_right, ALPHA)

    if left is None:
        left = last_left
    if right is None:
        right = last_right

    delta_left = None if (prev_left is None or last_left is None) else (last_left - prev_left)
    delta_right = None if (prev_right is None or last_right is None) else (last_right - prev_right)

    return left, right, last_left, last_right, delta_left, delta_right


def calculate_lateral_error(left, right, last_left, last_right):
    """Error de centrado: ideal => 0 cuando left==right."""
    if left is not None and right is not None:
        return left - right
    if left is not None and last_right is not None:
        return left - last_right
    if right is not None and last_left is not None:
        return last_left - right
    return 0.0


def calculate_position_maintenance_error(left, right, last_left, last_right):
    """Seguridad lateral: si está demasiado cerca de pared, forzar alejamiento.

    Convención MotionCommander:
    - velocity_y > 0 => se desplaza a la izquierda
    - velocity_y < 0 => se desplaza a la derecha
    """
    current_left = left if left is not None else last_left
    current_right = right if right is not None else last_right

    if current_left is None or current_right is None:
        return 0.0

    if current_left < MIN_SAFE_DISTANCE_WALL:
        # cerca de pared izquierda -> mover a la derecha (vy negativa)
        return -(MIN_SAFE_DISTANCE_WALL - current_left) * 2.0

    if current_right < MIN_SAFE_DISTANCE_WALL:
        # cerca de pared derecha -> mover a la izquierda (vy positiva)
        return (MIN_SAFE_DISTANCE_WALL - current_right) * 2.0

    return 0.0


def detect_jump(error, last_error, delta_left, delta_right, now, jump_until):
    kp_local = KP
    jump_detected = False

    if abs(error - last_error) > JUMP_ERROR_THRESHOLD:
        left_jump = (delta_left is not None and abs(delta_left) > JUMP_MEASURE_THRESHOLD)
        right_jump = (delta_right is not None and abs(delta_right) > JUMP_MEASURE_THRESHOLD)
        if left_jump or right_jump:
            jump_until = now + JUMP_IGNORE_TIME
            jump_detected = True
            print(
                f'Jump detected: d_err={error-last_error:.3f}  dL={delta_left}  dR={delta_right}  -> conservative'
            )

    if now < jump_until:
        kp_local = KP * KP_JUMP_SCALE

    return kp_local, jump_until, jump_detected


def compute_centering_vy(error, derivative, kp_local, kd_local):
    vy = 0.0
    if abs(error) > 0.02:
        vy = kp_local * error + kd_local * derivative
        if abs(vy) < MIN_LAT_SPEED:
            vy = MIN_LAT_SPEED * (-1 if vy < 0 else 1)

    if vy > MAX_LAT_VELOCITY:
        vy = MAX_LAT_VELOCITY
    elif vy < -MAX_LAT_VELOCITY:
        vy = -MAX_LAT_VELOCITY
    return vy


def repulsion_component(range_value, threshold, gain, sign):
    """Devuelve contribución de repulsión.

    sign = +1 empuja en dirección positiva cuando está cerca
    sign = -1 empuja en dirección negativa cuando está cerca
    """
    if range_value is None:
        return 0.0
    if range_value >= threshold:
        return 0.0
    return sign * gain * (threshold - range_value)


def compute_avoidance(multiranger):
    """Calcula evitación reactiva tipo 'push' SIN retroceso por obstáculo frontal.

    - vx_avoid: solo reacciona a obstáculo trasero (empuja hacia delante)
    - vy_avoid: reacciona a obstáculos laterales (empuja hacia el lado más libre)

    El rodeo del obstáculo frontal se gestiona en el bucle principal (bypass).
    """
    if not AVOID_ENABLE:
        return 0.0, 0.0

    # vx: back close -> avanzar (positivo). (Front se maneja como bypass lateral)
    vx_avoid = 0.0
    vx_avoid += repulsion_component(multiranger.back, AVOID_DISTANCE, AVOID_GAIN, sign=+1)

    # vy: left close -> mover derecha (negativo); right close -> mover izquierda (positivo)
    vy_avoid = 0.0
    vy_avoid += repulsion_component(multiranger.left, AVOID_DISTANCE, AVOID_GAIN, sign=-1)
    vy_avoid += repulsion_component(multiranger.right, AVOID_DISTANCE, AVOID_GAIN, sign=+1)

    # limitar contribuciones
    vx_avoid = max(-AVOID_MAX_VEL, min(AVOID_MAX_VEL, vx_avoid))
    vy_avoid = max(-AVOID_MAX_VEL, min(AVOID_MAX_VEL, vy_avoid))

    return vx_avoid, vy_avoid


def update_distances(dt, velocity_x, state, last_state_x, last_state_y, dist_dead, dist_state):
    if dt > 0:
        dist_dead += velocity_x * dt

    if state.get('x') is not None and state.get('y') is not None:
        if last_state_x is None or last_state_y is None:
            last_state_x = state['x']
            last_state_y = state['y']
        else:
            dx = state['x'] - last_state_x
            dy = state['y'] - last_state_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 5.0:
                dist_state += dist
            last_state_x = state['x']
            last_state_y = state['y']

    return dist_dead, dist_state, last_state_x, last_state_y


def print_debug(error, vx, vy, left, right, front, back, dist_dead, dist_state):
    print(
        f'errLR={error:+.3f}  vx={vx:+.3f}  vy={vy:+.3f}  '
        f'L={left if left is not None else None}  R={right if right is not None else None}  '
        f'F={front if front is not None else None}  B={back if back is not None else None}  '
        f'dead={dist_dead:.2f}  state={dist_state:.2f} / target={TARGET_DISTANCE_X:.1f}'
    )


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        state, log_conf = setup_logging(scf)

        with MotionCommander(scf) as motion_commander:
            with Multiranger(scf) as multiranger:
                keep_flying = True

                last_left = None
                last_right = None
                last_time = time.time()
                last_error = 0.0
                last_derivative = 0.0
                last_velocity_y = 0.0

                # Estado de rodeo (bypass) de obstáculo frontal
                bypass_active = False
                bypass_dir = 0   # +1 => izquierda (vy+), -1 => derecha (vy-)

                dist_dead = 0.0
                dist_state = 0.0
                last_state_x = None
                last_state_y = None

                jump_until = 0.0

                while keep_flying:
                    now = time.time()
                    dt = now - last_time if last_time is not None else 0.0
                    last_time = now

                    # 1) Lecturas L/R suavizadas para centrado
                    left, right, last_left, last_right, delta_left, delta_right = process_multiranger_lr(
                        multiranger.left, multiranger.right, last_left, last_right
                    )

                    # 2) Error y derivada (filtrada)
                    error = calculate_lateral_error(left, right, last_left, last_right)
                    derivative = 0.0 if dt <= 0 else (error - last_error) / dt
                    deriv_alpha = DAMPED_DERIV_ALPHA if DAMPED_MODE else DERIVATIVE_FILTER_ALPHA
                    derivative = deriv_alpha * derivative + (1.0 - deriv_alpha) * last_derivative
                    last_derivative = derivative

                    # 3) Jump detect: reduce KP y reduce vx base
                    kp_local, jump_until, _jump = detect_jump(error, last_error, delta_left, delta_right, now, jump_until)
                    vx_base = VELOCITY_X_CRUISE
                    if now < jump_until:
                        vx_base = min(vx_base, SLOW_VX_IN_JUMP)

                    # 4) Centrado PD -> vy_center
                    kd_local = KD * (DAMPED_KD_SCALE if DAMPED_MODE else 1.0)
                    if DAMPED_MODE:
                        kp_local = kp_local * DAMPED_KP_SCALE
                    vy_center = compute_centering_vy(error, derivative, kp_local, kd_local)

                    # 5) Seguridad lateral por proximidad a paredes (si activado)
                    if POSITION_MAINTENANCE:
                        pos_err = calculate_position_maintenance_error(left, right, last_left, last_right)
                        if abs(pos_err) > 0.05:
                            vy_center = max(-MAX_LAT_VELOCITY, min(MAX_LAT_VELOCITY, KP_POSITION * pos_err))

                    # 6) Evitación reactiva tipo "push" (sin retroceso frontal)
                    vx_avoid, vy_avoid = compute_avoidance(multiranger)

                    # 6b) Rodeo de obstáculo frontal: girar hacia el lado con más hueco y seguir avanzando
                    front = multiranger.front
                    if BYPASS_ENABLE:
                        if bypass_active:
                            # Salir del modo rodeo cuando el frente está despejado (histeresis)
                            if not is_close(front, BYPASS_FRONT_CLEAR):
                                bypass_active = False
                        else:
                            # Entrar en modo rodeo si hay obstáculo delante
                            if is_close(front, BYPASS_FRONT_TRIGGER):
                                bypass_active = True
                                # Elegir lado con más espacio (si no hay lectura, asumir muy libre)
                                l_clear = multiranger.left if multiranger.left is not None else 2.0
                                r_clear = multiranger.right if multiranger.right is not None else 2.0
                                bypass_dir = +1 if l_clear >= r_clear else -1  # +1 => izquierda (vy+)

                        if bypass_active:
                            # Mantener avance (reducido) y aplicar lateral fijo hacia el lado elegido
                            vx_base = VELOCITY_X_CRUISE * BYPASS_VX_SCALE
                            # Si está demasiado cerca, parar avance (pero NO retroceder)
                            if is_close(front, MIN_SAFE_DISTANCE_FRONT):
                                vx_base = 0.0
                            vy_bypass = bypass_dir * min(BYPASS_VY, MAX_LAT_VELOCITY)
                            vy_desired = (BYPASS_CENTER_BLEND * vy_center) + vy_bypass + vy_avoid
                        else:
                            # Normal: centrado + repulsión lateral. Si está muy cerca al frente, frenar.
                            if is_close(front, MIN_SAFE_DISTANCE_FRONT):
                                vx_base = 0.0
                            vy_desired = vy_center + vy_avoid
                    else:
                        # Sin bypass: solo frenar si muy cerca al frente
                        if is_close(front, MIN_SAFE_DISTANCE_FRONT):
                            vx_base = 0.0
                        vy_desired = vy_center + vy_avoid

                    # 7) Composición de setpoints
                    vx_cmd = vx_base + vx_avoid

                    # limitar vx_cmd de forma suave (mantener simple)
                    vx_cmd = max(-AVOID_MAX_VEL, min(AVOID_MAX_VEL, vx_cmd)) if AVOID_ENABLE else vx_cmd

                    # 8) Limitador de aceleración lateral (slew-rate)
                    if dt > 0:
                        max_lat_acc = DAMPED_MAX_LAT_ACCEL if DAMPED_MODE else MAX_LAT_ACCEL
                        max_delta = max_lat_acc * dt
                        delta_v = vy_desired - last_velocity_y
                        if delta_v > max_delta:
                            vy_cmd = last_velocity_y + max_delta
                        elif delta_v < -max_delta:
                            vy_cmd = last_velocity_y - max_delta
                        else:
                            vy_cmd = vy_desired
                    else:
                        vy_cmd = vy_desired

                    # Saturación final vy
                    vy_cmd = max(-MAX_LAT_VELOCITY, min(MAX_LAT_VELOCITY, vy_cmd))
                    last_velocity_y = vy_cmd
                    last_error = error

                    # 9) Parada por mano encima
                    if is_close(multiranger.up, MIN_DISTANCE_HAND_UP):
                        print('Hand detected above, stopping demo')
                        keep_flying = False

                    # 10) Distancia recorrida
                    dist_dead, dist_state, last_state_x, last_state_y = update_distances(
                        dt, vx_cmd, state, last_state_x, last_state_y, dist_dead, dist_state
                    )

                    print_debug(error, vx_cmd, vy_cmd, left, right, multiranger.front, multiranger.back, dist_dead, dist_state)

                    # Parada por distancia (prioriza estimador si hay datos)
                    if dist_state > 0.0:
                        if dist_state >= TARGET_DISTANCE_X:
                            print(f'Target distance {TARGET_DISTANCE_X} m reached (state), stopping')
                            keep_flying = False
                    else:
                        if dist_dead >= TARGET_DISTANCE_X:
                            print(f'Target distance {TARGET_DISTANCE_X} m reached (dead-reckoning), stopping')
                            keep_flying = False

                    # 11) Comando de movimiento
                    motion_commander.start_linear_motion(vx_cmd, vy_cmd, 0.0)
                    time.sleep(0.1)

        try:
            log_conf.stop()
            scf.cf.log.remove_config(log_conf)
        except Exception:
            pass

        print('Demo terminated!')
