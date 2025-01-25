from PyQt5.QtCore import QTimer
from app.views.loading_dialog import LoadingDialog

class DataController:

    def __init__(self, model, main_window):
        self.model = model
        self.main_window = main_window

        self.loading_dialog = None

    def load_data(self):
        self.loading_dialog = LoadingDialog(parent=self.main_window)
        self.loading_dialog.show()

        QTimer.singleShot(100, self._perform_load_data)

    def _perform_load_data(self):

        data = self.model.load_data()

        if self.loading_dialog:
            self.loading_dialog.close()
            self.loading_dialog = None

        self.main_window.set_status_text(data)