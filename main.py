import sys
from PyQt5.QtWidgets import QApplication
from app.main_window import MainWindow
from app.models.data_model import DataModel
from app.controllers.data_controller import DataController

def main():
    app = QApplication(sys.argv)

    model = DataModel()

    main_window = MainWindow()

    controller = DataController(model, main_window)

    main_window.set_controller(controller)

    main_window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()