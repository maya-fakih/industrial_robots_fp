# main.py
from robot_model import RobotModel
from robot_ui import RobotUI

if __name__ == "__main__":
    model = RobotModel()
    ui = RobotUI(model)
    ui.run()
