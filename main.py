from dronekit import connect, VehicleMode
import time
import math


TARGET_ALT = 200.0
YAW_LOCK_PWM = 1500
THROTTLE_HOVER = 1545
B_POS = (50.443326, 30.448078)
WIND_ORIGIN = 30

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

def get_distance(a, b):
    return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2) * 1.113195e5

def get_bearing(a, b):
    off_x = b[1] - a[1]
    off_y = b[0] - a[0]
    return math.degrees(math.atan2(off_x, off_y))

def send_stabilize_cmd(p_move, r_move, throttle):
    """Відправка команд з жорстким лімітом"""
    vehicle.channels.overrides = {
        '1': int(max(1200, min(1800, 1500 + r_move))), # Roll
        '2': int(max(1200, min(1800, 1500 + p_move))), # Pitch
        '3': int(throttle),
        '4': YAW_LOCK_PWM
    }

def get_vector_components(target_angle, magnitude):
    """
    Рахує Pitch/Roll відносно поточного Yaw дрона.
    Це дозволяє дрону летіти в правильну сторону, навіть якщо його розвернуло.
    """
    curr_yaw_rad = vehicle.attitude.yaw # Радіани
    target_rad = math.radians(target_angle)
    
    relative_rad = target_rad - curr_yaw_rad
    
    p = -math.cos(relative_rad) * magnitude
    r = math.sin(relative_rad) * magnitude
    return p, r


def takeoff(alt):
    print("Taking off...")
    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.armed = True
    while not vehicle.armed: time.sleep(0.1)
    
    while vehicle.location.global_relative_frame.alt < alt:
        curr_alt = vehicle.location.global_relative_frame.alt
        thr = 1780 if curr_alt < 15 else THROTTLE_HOVER + (alt - curr_alt)*15
        send_stabilize_cmd(0, 0, thr)
        time.sleep(0.1)

def fly_to_b():
    print("Moving to B (High Speed)...")
    while True:
        loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        dist = get_distance(loc, B_POS)
        alt = vehicle.location.global_relative_frame.alt
        
        if dist < 15: break

        brg = get_bearing(loc, B_POS)
        p_move, r_move = get_vector_components(brg, magnitude=220) 

        p_wind, r_wind = get_vector_components(WIND_ORIGIN, magnitude=65)
        
        send_stabilize_cmd(p_move + p_wind, r_move + r_wind, THROTTLE_HOVER + (TARGET_ALT-alt)*10)
        print(f"Dist: {dist:.1f}m | Alt: {alt:.1f}m | Brg: {brg:.1f}", end='\r')
        time.sleep(0.1)

def land_precision():
    print("\nPrecision Landing...")
    for _ in range(25):
        v_n, v_e = vehicle.velocity[0], vehicle.velocity[1]
        send_stabilize_cmd(v_n*40, -v_e*40, THROTTLE_HOVER)
        time.sleep(0.1)

    while vehicle.location.global_relative_frame.alt > 0.4:
        loc = (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        dist = get_distance(loc, B_POS)
        alt = vehicle.location.global_relative_frame.alt
        
        brg = get_bearing(loc, B_POS)
        p_corr, r_corr = get_vector_components(brg, magnitude=dist * 20)
        p_wind, r_wind = get_vector_components(WIND_ORIGIN, magnitude=70)
        
        send_stabilize_cmd(p_corr + p_wind, r_corr + r_wind, THROTTLE_HOVER - 150)
        time.sleep(0.1)

try:
    takeoff(TARGET_ALT)
    fly_to_b()
    land_precision()
    print("\nLanded!")
    vehicle.channels.overrides = {'3': 1000}
    vehicle.armed = False
finally:
    vehicle.close()