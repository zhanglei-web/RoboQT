import cgitb
import sys
from PyQt5.QtWidgets import QApplication
from main_window import RobotConfigWindow

if __name__=="__main__":
    cgitb.enable(format="text")
    app = QApplication(sys.argv)
    w = RobotConfigWindow()
    w.show()
    sys.exit(app.exec_())