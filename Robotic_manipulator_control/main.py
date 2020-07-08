"""
This is the main script for my simulation program with interface
"""

import sys
from ui_files import *
from PyQt5.QtWidgets import QMessageBox

"""
Create global variables
"""
change_required = False
program_running = False
Form = ""
ui = ""
"""
Define ui functions
"""
def exit_message():
    global ui, Form

    ans = QMessageBox.question(Form, "Extract!", "Get into the chopper?")
    
    if ans == QMessageBox.Yes:
        print("yep")
    elif ans == QMessageBox.No:
        print("Nay")





def run_button():
    global change_required, program_running, Form ,ui
    print("some function to read data in from interface")
    print("some function to pass data and run the desired simulation")
    Form.setWindowTitle("i adjusted the thing")
    program_running = False
    return True







def run():
    """
    setup the program interface
    """
    global change_required, program_running, Form ,ui
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    
    ui.setupUi(Form)
    Form.setWindowTitle("Robot Simulator")
    exit_message()
    """
    Attach functions each of the inputs
    """
    ui.run_button.clicked.connect(run_button)
    
    
    #=====================================================
    Form.show()

    
    sys.exit(app.exec_())


if __name__ == "__main__":

    run()
