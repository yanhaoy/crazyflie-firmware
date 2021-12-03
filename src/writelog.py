import logging
import time
import pickle

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

is_deck_attached = False

log_dict = {'stateEstimate.x': [],
            'stateEstimate.y': [],
            'stateEstimate.z': [],
            'stateEstimate.roll': [],
            'stateEstimate.pitch': [],
            'stateEstimate.yaw': [],
            'stateEstimate.vx': [],
            'stateEstimate.vy': [],
            'stateEstimate.vz': [],
            'gyro.x': [],
            'gyro.y': [],
            'gyro.z': [],
            'motor.m1': [],
            'motor.m2': [],
            'motor.m3': [],
            'motor.m4': []
            }


def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    for key in data:
        log_dict[key].append(data[key])


def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    global is_deck_attached
    if value:
        is_deck_attached = True
        print('Deck is attached!')
    else:
        is_deck_attached = False
        print('Deck is NOT attached!')


def take_off_simple(scf):
    with MotionCommander(scf) as mc:
        time.sleep(20)


if __name__ == '__main__':

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    log_pos = LogConfig(name='pos', period_in_ms=10)
    log_pos.add_variable('stateEstimate.x', 'float')
    log_pos.add_variable('stateEstimate.y', 'float')
    log_pos.add_variable('stateEstimate.z', 'float')

    log_att = LogConfig(name='att', period_in_ms=10)
    log_att.add_variable('stateEstimate.roll', 'float')
    log_att.add_variable('stateEstimate.pitch', 'float')
    log_att.add_variable('stateEstimate.yaw', 'float')

    log_vel = LogConfig(name='vel', period_in_ms=10)
    log_vel.add_variable('stateEstimate.vx', 'float')
    log_vel.add_variable('stateEstimate.vy', 'float')
    log_vel.add_variable('stateEstimate.vz', 'float')

    log_angvel = LogConfig(name='angvel', period_in_ms=10)
    log_angvel.add_variable('gyro.x', 'float')
    log_angvel.add_variable('gyro.y', 'float')
    log_angvel.add_variable('gyro.z', 'float')

    log_motor = LogConfig(name='motor', period_in_ms=10)
    log_motor.add_variable('motor.m1', 'float')
    log_motor.add_variable('motor.m2', 'float')
    log_motor.add_variable('motor.m3', 'float')
    log_motor.add_variable('motor.m4', 'float')

    log_set = [log_pos, log_att, log_vel, log_angvel, log_motor]

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                         cb=param_deck_flow)
        time.sleep(5)

        cf = scf.cf
        for log_handle in log_set:
            cf.log.add_config(log_handle)
            log_handle.data_received_cb.add_callback(log_stab_callback)
            log_handle.start()

        # Just record
        # time.sleep(20)

        # Fly with the default controller and record
        if is_deck_attached:
            take_off_simple(scf)

        for log_handle in log_set:
            log_handle.stop()

    log_file = open(time.strftime("%Y%m%d-%H%M%S"), 'wb')
    pickle.dump(log_dict, log_file)
    log_file.close()
