# main.py
from robot_model import RobotModel
from robot_ui import RobotUI

if __name__ == "__main__":
    model = RobotModel()

    # <<< FIX: Add your Arduino port here
    SERIAL_PORT = "COM3"   # or COM4, COM5 ... whatever your Arduino uses

    ui = RobotUI(model, SERIAL_PORT)
    ui.run()
