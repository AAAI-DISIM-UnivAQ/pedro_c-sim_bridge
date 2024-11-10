
'''
    From here the Robot can be controlled by an AI, internal or external through a message system
    (Redis and/or PEDRO)
'''

from RobotModel import PioneerP3DX

import time
# from redis import Redis
import pedroclient
import queue
import threading


class Control(object):
    def __init__(self, sleep_time):
        self._sleep_time = sleep_time
        self._last_percept_str = ''
        self._stop = False

    def run(self):
        r = self.make_robot()
        while True:
            r.process_commands(self.get_commands())
            perceptions = r.get_percepts()
            self.process_percepts(perceptions)
            time.sleep(self._sleep_time)
            sim_stop = False
            if self._stop or sim_stop:
                r.simulation.pause()
                key = input('Press RETURN')
                self._stop = False
                r.simulation.start()


    def make_robot(self, host):
        return None

    def process_initialize(self):
        pass

    def process_percepts(self, percepts_method):
        pass

    def get_commands(self):
        pass

class DemoControl(Control):
    def __init__(self, sleep_time=0.01):
        super().__init__(sleep_time)
        self._rl = 0 # right sensor reading
        self._ll = 0 # left
        self._cl = 0 # center

    def make_robot(self, host="localhost"):
        return PioneerP3DX('Pioneer_p3dx', host)

    def process_percepts(self, percepts):
        self._ll = percepts['left']
        self._rl = percepts['right']
        self._cl = percepts['center']

    def get_commands(self):
        rl = self._rl
        ll = self._ll
        cl = self._cl
        if 0.01 < rl < 10:
            return [{'cmd': 'turn_left', 'args': [0.5]}]
        elif 0.01 < ll < 10:
            return [{'cmd': 'turn_right', 'args': [0.5]}]
        else:
            speed = 10.0
            if cl<10:
                speed = -1
            else:
                speed = 1
            if speed > 5.0:
                speed = 5.0
            return [{'cmd': 'move_forward', 'args': [speed]}]


class KeyboardControl(Control):
    def __init__(self, sleep_time=0.01):
        super().__init__(sleep_time)
        self._rl = 0 # right sensor reading
        self._ll = 0 # left
        self._cl = 0 # center

    def make_robot(self, host="localhost"):
        return PioneerP3DX('Pioneer_p3dx', host)

    def process_percepts(self, percepts):
        self._ll = percepts['left']
        self._rl = percepts['right']
        self._cl = percepts['center']

    def get_commands(self):
        command = input('WSAD? ')
        if len(command)==0:
            return []
        if command[0] .lower() == 'w':
            return [{'cmd': 'move_forward', 'args': [0.5]}]
        elif command[0] .lower() == 'd':
            return [{'cmd': 'turn_right', 'args': [0.2]}]
        elif command[0].lower() == 'a':
            return [{'cmd': 'turn_left', 'args': [0.2]}]
        elif command[0].lower() == 's':
            return [{'cmd': 'move_forward', 'args': [-0.5]}]


# Handling messages from the TR program
class MessageThread(threading.Thread):
    def __init__(self, client, q):
        self.running = True
        self.client = client
        self.queue = q
        threading.Thread.__init__(self)
        self.daemon = True

    def run(self):
        while self.running:
            p2pmsg = self.client.get_term()[0]
            self.queue.put(p2pmsg)

    def stop(self):
        self.running = False


class PedroControl(Control):
    def __init__(self, host='127.0.0.1', port=19997, sleep_time=0.01):
        super().__init__(host, port, sleep_time)
        try:
            self.client = pedroclient.PedroClient()
        except ConnectionRefusedError as e:
            print('PEDRO server not responding', e)
            exit(-1)
        self.client.register("vrep_pedro")
        self.queue = queue.Queue(0)
        self.message_thread = MessageThread(self.client, self.queue)
        self.message_thread.start()
        p2pmsg = self.queue.get()
        self.percepts_addr = p2pmsg.args[1]

    def set_client(self, addr):
        self.percepts_addr = addr

    def send_percept(self, percepts_string):
        print("send_percept", str(self.percepts_addr), percepts_string)
        if self.client.p2p(self.percepts_addr, percepts_string) == 0:
            print("Error", percepts_string)

    def process_initialize(self):
        # Block unitil message arrives
        p2pmsg = self.queue.get()
        print(p2pmsg)
        message = p2pmsg.args[2]
        if str(message) == 'initialise_':
            # get the sender address
            percepts_addr = p2pmsg.args[1]
            print("percepts_addr", str(percepts_addr))
            self.set_client(percepts_addr)
            init_percepts = '[sonar(9999, 9999, 9999)]'
            self.send_percept(init_percepts)
        else:
            print("Didn't get initialise_ message")

    def make_robot(self, api):
        return PioneerP3DX('Pioneer_p3dx', api)

    def process_percepts(self, percepts):
        vision = percepts['vision']
        percept = '['
        percept += 'sonar({0:.3f}, {1:0.3f}, {2:0.3f})'.format(percepts['left'],
                                                               percepts['center'],
                                                               percepts['right'])
        if vision != ('',0,0,0):
            # something has been seen
            vision_pred = f'vision( {vision[0]}, {vision[1]}, {vision[2]}, {vision[3]})'
            percept += ', '
            percept += vision_pred
        percept += ']'

        if percept != self._last_percept_str:
            self.send_percept(percept)
            self._last_percept_str = percept

    def get_commands(self):
        cmds = []
        while not self.queue.empty():
            p2pmsg = self.queue.get()
            msg = p2pmsg.args[2]
            actions = msg
            if isinstance(actions, pedroclient.PList):
                action_list =  actions.toList()
                for a in action_list:
                    cmds.append(self.action_to_command(a, len(action_list)))
            if 'stopped' in str(msg) or 'bottle_found' in str(msg):
                self._stop = True
            else:
                print(164, actions)
        return cmds

    def action_to_command(self, a, num_actions):
        print(181, str(a))
        cmd_type = a.functor.val
        cmd = a.args[0]
        if cmd_type == 'stop_':
            if num_actions == 1:
                # only do a stop if the only action
                return {'cmd': 'move_forward', 'args': [0.0]}
        else:
            if cmd.functor.val == 'move_forward':
                speed = cmd.args[0].val
                return {'cmd': 'move_forward', 'args': [speed]}
            elif cmd.functor.val == 'turn':
                speed = cmd.args[1].val
                if cmd.args[0].val == 'left':
                    return {'cmd': 'turn_left', 'args': [speed]}
                else:
                    return {'cmd': 'turn_right', 'args': [speed]}
            elif cmd.functor.val == 'display':
                task_num = cmd.args[0].val
                return {'cmd': 'display', 'args': [task_num]}
        # maybe raise an exception?
        return {'cmd': 'illegal_command', 'args': [str(a)]}

class TeleoControl(PedroControl):    
    def __init__(self, host='127.0.0.1', port=19997, sleep_time=0.01):
        super().__init__(host, port, sleep_time)

    def vision2dist(self, width) :
        if width > 0.23:
            return "dist0"
        elif width > 0.1563:   #elif width > 0.1875:
            return "dist1"
        elif width > 0.124:
            return "dist2"
        elif width > 0.09374:
            return "dist3"
        elif width > 0.0624:
            return "dist4"
        else:
            return "dist5"

    def sonar2dist(self, dist):
        if dist > 1.0:
            return "dist6"
        elif dist > 0.609:
            return "dist5"
        elif dist > 0.444:
            return "dist4"
        elif dist > 0.33:
            return "dist3"
        elif dist > 0.217:
            return "dist2"
        elif dist > 0.137:
            return "dist1"
        else:
            return "dist0"
        
    def process_initialize(self):
        # Block unitil message arrives
        p2pmsg = self.queue.get()
        print(p2pmsg)
        message = p2pmsg.args[2]
        if str(message) == 'initialise_':
            # get the sender address
            percepts_addr = p2pmsg.args[1]
            print("percepts_addr", str(percepts_addr))
            self.set_client(percepts_addr)
            init_percepts = '[]'
            self.send_percept(init_percepts)
        else:
            print("Didn't get initialise_ message")
            
    def process_percepts(self, percepts):
        vision = percepts['vision']
        sonar_left_dist = self.sonar2dist(percepts['left'])
        sonar_center_dist = self.sonar2dist(percepts['center'])
        sonar_right_dist = self.sonar2dist(percepts['right'])
        
        percept = '['
        percept += 'sonar({0}, {1}, {2})'.format(sonar_left_dist,
                                                 sonar_center_dist,
                                                 sonar_right_dist)
        if vision[1] > 0.2 or vision[1] > 0 and vision[3] > 1.5*vision[1]:
            # bottle in front of wall is seen
            vision_dist = self.vision2dist(vision[1])
            vision_pred = f'vision( {vision[0]}, {vision_dist})'
            percept += ', '
            percept += vision_pred
        percept += ']'

        if percept != self._last_percept_str:
            self.send_percept(percept)
            self._last_percept_str = percept

def redis_control():
    '''
            Percepts are sent to the ROBOT:PERCEPTS redis channel
            Actions are received through the ROBOT redis channel
    :return:
    '''

    with VRep.connect("127.0.0.1", 19997) as api:
        r = PioneerP3DX('Pioneer_p3dx', api)
        Red = Redis()
        pubsub = Red.pubsub()
        pubsub.subscribe('ROBOT')
        speed = 0.0
        print('listening..')
        prev = ''
        while True:
            msg = pubsub.get_message(ignore_subscribe_messages=True)
            if msg:
                cmd = msg['data'].decode('utf-8').split(',')[-1][:-1].strip()
                try:
                    r.invoke(cmd)
                except NotImplementedError:
                    print(cmd, 'not implemented!')
            # build percepts string
            rl = r.right_distance()
            ll = r.left_distance()
            percepts = '[sonar({0:.3f}, {1:0.3f})]'.format(ll, rl)
            if percepts!= prev:
                prev = percepts
                Red.publish('ROBOT:PERCEPTS', percepts)

def demo_control():
    '''
        In process robot control. Just a simple navigation, avoiding to bump into walls.
    :return:
    '''

    print('demo controller active')
    vrep_demo = DemoControl()
    vrep_demo.process_initialize()  # not needed in this case
    vrep_demo.run()


def keyboard_control():
    '''
        In process robot control. Navigate tthe robot from keyboard with WSAD keys.
    :return:
    '''

    print('keyboard controller active')
    vrep_demo = KeyboardControl()
    vrep_demo.process_initialize()  # not needed in this case
    vrep_demo.run()

def pedro_control():
    '''
        Out of process robot control by a teleor AI
    :return:
    '''

    print('pedro/teleor controller active')
    vrep_pedro = PedroControl()
    # wait for and process initialize_ message
    vrep_pedro.process_initialize()
    vrep_pedro.run()

def teleo_control():
    '''
       Like pedro_control except distances are abstracted
    '''

    print('teleor controller active')
    vrep_teleo = TeleoControl(sleep_time=0.05)
    # wait for and process initialize_ message
    vrep_teleo.process_initialize()
    vrep_teleo.run()
