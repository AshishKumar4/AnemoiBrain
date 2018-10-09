from SensorAbstraction.VirtualSensors import *
from ControlAbstraction.RawControl import *
from ControlAbstraction.MotorBrain import *

import os 
import subprocess
from Navigation import *

from PurposeAbstraction.VideoClassification import *

VERSION = 0.1

if __name__ == '__main__':
    # The Actual Kernel Code
    print("Drone Kernel "+VERSION+"Initializing...")

    # Initialize Raw Control Servers; The Abstraction Layer should itself perform tests to validate
    
    # Initialize Sensor Control Servers; The Abstraction Layer should itself perform tests to validate

    # Perform some Tests 

    # Initialize Telemetry

    # Establish Connection to Base Systems

    # Wait for Signal/Command/Purpose to execute

    # Initialize The Purpose Control

