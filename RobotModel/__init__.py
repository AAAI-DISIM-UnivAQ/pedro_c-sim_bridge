
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from typing import  Any
import time

SENSOR_RETRY = 40
SENSOR_RETRY_SLEEP = 0.01

class SimSensor:

    _name: str
    _sensor_handle: Any
    _value: Any

    def __init__(self, name, sim):
        self._name = name
        self._sensor_handle = sim.getObjectHandle(self._name)
        self._value = None

    def read(self, sim):
        pass

class ProximitySensor(SimSensor):

    def read(self, sim):
        self._value = sim.readProximitySensor(self._sensor_handle)
        return self._value

class VisionSensor(SimSensor):

    def read(self, sim):
        self._value = sim.getVisionSensorCharImage(self._sensor_handle)
        return self._value

class SimActuator:

    _name : str
    _actuator_handle: Any

    def __init__(self, name, sim):
        self._name = name
        self._actuator_handle = sim.getObjectHandle(self._name)

    def actuate(self, sim: Any, speed: float):
        self._sim.setJointTargetVelocity(self._actuator_handle, speed)


class RobotModel:

    _client: RemoteAPIClient
    _sim: Any
    _name: str
    _sensors: dict[str, SimSensor]
    _actuators: dict[str, SimActuator]

    def __init__(self, name: str, host="localhost", sim_stepping=False):
        self._name = name
        self._client = RemoteAPIClient(host=host)
        self._sim = self._client.require("sim")
        self._sim.setStepping(sim_stepping)
        self._sensors = {}
        self._actuators = {}

    def step(self):
        print("step")
        self._sim.step()

    def start_simulation(self):
        self._sim.startSimulation()

    def stop_simulation(self):
        self._sim.stopSimulation()


class PioneerP3DX(RobotModel):

    def __init__(self, name: str, host="localhost", sim_stepping=False):
        RobotModel.__init__(self, name, host, sim_stepping)
        self._actuators['left'] = SimActuator('./leftMotor', self._sim)
        self._actuators['right'] = SimActuator('./rightMotor', self._sim)
        self._sensors['left'] = ProximitySensor("./ultrasonicSensor[2]", self._sim)
        self._sensors['center_l'] = ProximitySensor("./ultrasonicSensor[3]", self._sim)
        self._sensors['center_r'] = ProximitySensor("./ultrasonicSensor[4]", self._sim)
        self._sensors['right'] = ProximitySensor("./ultrasonicSensor[5]", self._sim)
        self._sensors['vision'] = VisionSensor("./visionSensor", self._sim)
        self._set_two_motor(0.0, 0.0)

    def _set_two_motor(self, left_speed: float, right_speed: float):
        self._sim.setJointTargetVelocity(self._actuators['left']._actuator_handle, left_speed)
        self._sim.setJointTargetVelocity(self._actuators['right']._actuator_handle, right_speed)

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

    def _read_sensor(self, name: str):
        assert name in self._sensors, f"Unknown sensor: {name}"
        dis = 9999
        for _ in range(SENSOR_RETRY):
            # try to read sensor up to 5 times
            dis = self._sensors[name].read(sim=self._sim)[1]
            if dis > 0.01:
                break
            time.sleep(SENSOR_RETRY_SLEEP)
        if dis > 9999: dis = 9999
        return dis

    def right_distance(self, name: str):
        return self._read_sensor(name)

    def left_distance(self, name):
        return self._read_sensor(name)

    def center_distance(self):
        a,b = 0, 0
        a = self._read_sensor("center_l")
        b = self._read_sensor("center_r")
        dis = min(a,b)
        if dis > 9999: dis = 9999
        return dis

    def display(self, code):
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
        # code, state, vision_result = self._sensors['vision'].read()
        # position, size, base, height = self.get_vision(vision_result)
        # out = (position, size, base, height)
        return (0, 0, 0, 0)

    def get_percepts(self):
        out = {'left': self.left_distance("left"),
               'center': self.center_distance(),
               'right': self.right_distance("right"),
               'vision': self.vision()
               }
        #print(126, 'percepts:', out)
        print(out)
        return out

    def get_signal(self, name):
        # returnCode, signalValue = self._api.simxGetIntegerSignal(0, name, 0)
        return 0

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
