import os
from PyQt5.QtWidgets import QMainWindow
from PyQt5 import uic

class MainWindow(QMainWindow):

    def __init__(self, parent=None):
        super().__init__(parent)

        ui_file = os.path.join(
            os.path.dirname(__file__),
            'ui',
            'main_window.ui'
        )
        uic.loadUi(ui_file, self)

        self.controller = None

        self.pushButtonLoad.clicked.connect(self.on_load_button_clicked)

    def set_controller(self, controller):
        self.controller = controller

    def on_load_button_clicked(self):

        if self.controller:
            self.controller.load_data()

    def set_status_text(self, text):

        self.labelStatus.setText(text)
