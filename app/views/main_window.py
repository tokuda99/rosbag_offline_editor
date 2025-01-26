import os
from PyQt5.QtWidgets import QMainWindow
from PyQt5 import uic

class MainWindow(QMainWindow):
    """
    メインウィンドウのクラス (View)。
    UIイベントを受け取り、Controllerのメソッドを呼ぶ。
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # main_window.ui をロード (ファイルパスは配置に合わせて修正)
        ui_file = os.path.join(
            os.path.dirname(__file__),
            'ui',
            'main_window.ui'
        )
        uic.loadUi(ui_file, self)

        # コントローラは後で設定
        self.controller = None

        # ボタンにシグナルを接続
        self.pushButtonLoad.clicked.connect(self.on_load_button_clicked)

    def set_controller(self, controller):
        self.controller = controller

    def on_load_button_clicked(self):
        """
        「Load Data (Async)」ボタンが押された時の処理。
        """
        if self.controller:
            self.controller.load_data_async()

    def set_status_text(self, text):
        """
        コントローラまたはモデルから渡されたデータを表示。
        """
        self.labelStatus.setText(text)