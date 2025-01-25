from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel

class LoadingDialog(QDialog):


    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Loading...")
        self.setModal(True) 

        layout = QVBoxLayout()
        self.label = QLabel("Loading data. Please wait...")
        layout.addWidget(self.label)

        self.setLayout(layout)
        self.resize(300, 100)
