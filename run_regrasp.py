# imports
import brl_gripper as bg
import mujoco as mj
import numpy as np
import atexit
import tty
import termios
import sys
import os
import time
from datetime import datetime as dt

# initialization
print("Starting init.")
init_settings = termios.tcgetattr(sys.stdin)

# add object to model here
xml_path = os.path.join(bg.assets.ASSETS_DIR, 'scene')

obj_type = "cylinder"
obj_pos = [0.23, 0.0, 0.05]
obj_height = 0.06 
obj_radius = 0.04
obj_mass = 0.2
obj_inertia_xy = (1/4)*obj_mass*obj_radius**2 + 1/12*obj_mass*obj_height**2
obj_inertia_z = (1/2)*obj_mass*obj_radius**2

# original
xml_string = """
<mujoco model="scene">
    <include file=\"""" + xml_path + ".xml\"" + """/>
    <!-- CUBE -->
    <worldbody>
        <body name="cube" pos=\""""+str(obj_pos[0])+" "+str(obj_pos[1])+" "+str(obj_pos[2])+"""\">
            <joint type="free" name="cube_joint" group="3" stiffness="0" damping="0" frictionloss="0" armature="0"/>
            <inertial pos="0 0 0" mass=\""""+str(obj_mass)+"""\" diaginertia=\""""+str(obj_inertia_xy)+" "+str(obj_inertia_xy)+" "+str(obj_inertia_z)+"""\"/>
            <geom name="object" type=\""""+obj_type+"""\" group="3" size=\""""+str(obj_radius)+" "+str(obj_height/2.0)+"""\" rgba="0.7 0.2 0.1 0.6" contype="1" conaffinity="1" condim="4" priority="2" friction="0.5 0.02 0.0001" solimp="0.95 0.99 0.001 0.5 2"  solref="0.002 1"/>
        </body>
    </worldbody>
    <option impratio="10" timestep="0.0005" integrator="implicitfast" cone="elliptic" solver="Newton" noslip_iterations="0">
        <flag contact="enable" override="disable" multiccd="disable"/>
    </option>
</mujoco>
"""




# mj_model = mj.MjModel.from_xml_path(os.path.join(bg.assets.ASSETS_DIR, 'scene_with_object.xml'))
mj_model = mj.MjModel.from_xml_string(xml_string)

hw_mode = bg.HardwareEnable.NO_HW
sens_mode = bg.SensorDataMode.NO_PRESSURE_VALS

# hw_mode = bg.HardwareEnable.FINGERS_ONLY
# sens_mode = bg.SensorDataMode.RAW_PRESSURE_VALS
# platform
log_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),'logs/')
GP = bg.GripperPlatform(mj_model, viewer_enable=True, hardware_enable=hw_mode, sensor_mode=sens_mode, log_path=None)

# controller
from controllers.regrasp.RegraspFSM import RegraspFSM
controller = RegraspFSM()

# shutdown
atexit.register(GP.shutdown)
print("Finished init.")

# start experiment
try:
    tty.setcbreak(sys.stdin.fileno())
    GP.initialize() # TODO: make sure that this waits for gripper to be initialized
    controller.begin(GP)
    GP.apply_control()
    GP.sync_viewer()
    print("Starting main loop.")
    real_start_time = time.time()
    while GP.mode==bg.PlatformMode.HW_NO_VIS or GP.mj_viewer.is_running(): # TODO: better way to do this?
        if not GP.paused:
            # step in time to update data from hardware or sim
            GP.step()
            # run controller and update commands
            GP.dt_comp = 0.0 # for real-time simulation
            if GP.run_control:
                control_start_time = GP.time()
                GP.run_control = False
                GP.sync_data()
                controller.update(GP)
                GP.apply_control()
                GP.log_data()
                GP.dt_comp += GP.time() - control_start_time
            # sync viewer
            if GP.run_viewer_sync:
                viewer_sync_start_time = GP.time()
                GP.run_viewer_sync = False
                GP.sync_viewer()
                GP.dt_comp += GP.time() - viewer_sync_start_time

# end experiment
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, init_settings)