
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import Any
import json
import time

SENSOR_RETRY = 40
SENSOR_RETRY_SLEEP = 0.01


class RobotModel:
    def __init__(self, name: str, api: Any):
        self._api = api
        self._name = name
        self._sensors = None    # some kind of collection class
        self._actuators = None  # idem


class PioneerP3DX(RobotModel):

    def __init__(self, name: str, api: Any):
        RobotModel.__init__(self, name, api)
        self._actuators = {}
        self._sensors = {}
        # self._signals = {}
        self._actuators['left'] = api.joint.with_velocity_control(name+"_leftMotor")
        self._actuators['right'] = api.joint.with_velocity_control(name+"_rightMotor")
        self._sensors['left'] = api.sensor.proximity(name+"_ultrasonicSensor3")
        self._sensors['center'] = (api.sensor.proximity(name+"_ultrasonicSensor4"), api.sensor.proximity(name+"_ultrasonicSensor5"))
        self._sensors['right'] = api.sensor.proximity(name+"_ultrasonicSensor6")
        self._sensors['vision'] = api.sensor.vision("Vision_sensor")
        self._set_two_motor(0.0, 0.0)

    def turn_right(self, speed=2.0):
        print('turn_right', speed)
        self._set_two_motor(speed, -speed)

    def turn_left(self, speed=2.0):
        print('turn_left', speed)
        self._set_two_motor(-speed, speed)

    def move_forward(self, speed=2.0):
        print('move_forward', speed)
        self._set_two_motor(speed, speed)

    def move_backward(self, speed=2.0):
        self._set_two_motor(-speed, -speed)

    def _set_two_motor(self, left: float, right: float):
        self._actuators['left'].set_target_velocity(left)
        self._actuators['right'].set_target_velocity(right)

    def right_distance(self):
        for _ in range(SENSOR_RETRY):
            # try to read sensor up to 5 times
            dis = self._sensors['right'].read()[1].distance()
            if dis > 0.01:
                break
            time.sleep(SENSOR_RETRY_SLEEP)
        if dis > 9999: dis = 9999
        return dis

    def left_distance(self):
        for _ in range(SENSOR_RETRY):
            # try to read sensor up to 5 times
            dis = self._sensors['left'].read()[1].distance()
            if dis > 0.01:
                break
            time.sleep(SENSOR_RETRY_SLEEP)
        if dis > 9999: dis = 9999
        return dis

    def center_distance(self):
        for _ in range(SENSOR_RETRY):
            a = self._sensors['center'][0].read()[1].distance()
            b = self._sensors['center'][1].read()[1].distance()
            if a > 0.01 and b > 0.01:
                break
            time.sleep(SENSOR_RETRY_SLEEP)
        dis = min(a,b)
        if dis > 9999: dis = 9999
        return dis

    def display(self, code):
        # res = self._api.simxCallScriptFunction(0, 'debug', self._api.sim_scripttype_mainscript,
        #                                          'show_debug', code, self._api.simx_opmode_blocking)
        print(f'display:{code}')

    def get_vision(self, vision_result):
        """
        extract blob data from vision sensor image buffer
            only sees red blobs

        blob_data[0]=blob count
        blob_data[1]=n=value count per blob
        blob_data[2]=blob 1 size
        blob_data[3]=blob 1 orientation
        blob_data[4]=blob 1 position x
        blob_data[5]=blob 1 position y
        blob_data[6]=blob 1 width
        blob_data[7]=blob 1 height
        ...
        :return: POSITION, SIZE
            POSITION: close, center, left, right
            SIZE: 0.0 .. 1.0
        """
        SIZE_THR = 0.3

        if len(vision_result) > 1:
            blob_data = vision_result[1]
            position = ''
            blob_size = 0
            blob_base = 0
            blob_height = 0
            blob_count = int(blob_data[0])
            if blob_count>0:
                # print('blobs: ', blob_count)
                blob_size = blob_data[6]  # red blob width
                blob_base = blob_data[5] - blob_data[7]/2.0
                blob_height = blob_data[7]
                print(f'blob_size:{blob_size}')
                if blob_size >= SIZE_THR:
                    return "close", round(blob_size, 5), round(blob_base, 3), round(blob_height, 3)
                if 0.35 < blob_data[4] < 0.65:
                    return "center", round(blob_size, 5), round(blob_base, 3), round(blob_height, 3)
                if 0.0 < blob_data[4] < 0.35:
                    return "left", round(blob_size, 5), round(blob_base, 3), round(blob_height, 3)
                if 0.65 < blob_data[4] < 1:
                    return "right", round(blob_size, 5), round(blob_base, 3), round(blob_height, 3)
            return position, round(blob_size, 5), round(blob_base, 3), round(blob_height, 3)
        else:
            return '', 0, 0, 0

    def vision(self):
        code, state, vision_result = self._sensors['vision'].read()
        position, size, base, height = self.get_vision(vision_result)
        out = (position, size, base, height)
        return out

    def get_percepts(self):
        out = {'left': self.left_distance(),
               'center': self.center_distance(),
               'right': self.right_distance(),
               'vision': self.vision()
               }
        #print(126, 'percepts:', out)
        print(json.dumps(out))
        return out

    def get_signal(self, name):
        returnCode, signalValue = self._api.simxGetIntegerSignal(0, name, 0)
        return signalValue

    def process_commands(self, commands):
        # print(commands)
        for cmd in commands:
            self.invoke(cmd['cmd'], cmd['args'])

    def invoke(self, cmd, args):
        # print('invoke', cmd, args)
        if cmd!= 'illegal_command':
            try:
                getattr(self.__class__, cmd)(self, *args)
            except AttributeError:
                raise NotImplementedError("Class `{}` does not implement `{}`".format(self.__class__.__name__, cmd))
