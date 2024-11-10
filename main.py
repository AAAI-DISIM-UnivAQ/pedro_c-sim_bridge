
# To run in Python3

"""
    Copyright @giodegas 2018, 2020, 2024
    Class to control a PioneerP3DX inside Coppelia robot simulator
"""


from RobotControl import demo_control, keyboard_control, redis_control, pedro_control, teleo_control

# TODO: see robot collector example to handle proper simulation intialization
#       file bottles_center_env.py , function process_msg, line 1074

## MAIN - Select your control function

if __name__ == '__main__':
    demo_control()
    # keyboard_control()
    # pedro_control()
    # teleo_control()
