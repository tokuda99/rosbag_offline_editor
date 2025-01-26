import sys
import signal
from PyQt5.QtWidgets import QApplication
from app.views.bag_editor_window import BagEditorWindow
from app.models.bag_model import BagModel
from app.controllers.bag_editor_controller import BagEditorController

def sigint_handler(signum, frame):
    """
    Ctrl+C が押された時に呼ばれるハンドラ。
    ここではアプリケーションを終了する動作にする。
    """
    print("SIGINT (Ctrl+C) detected. Exiting application...")
    # 終了処理があればここで実行
    sys.exit(0)
    
def main():
    # signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)

    # Modelインスタンスを生成
    model = BagModel()

    # メインウィンドウ (View) を生成
    window = BagEditorWindow()

    # Controller を生成し、モデルとメインウィンドウを渡す
    controller = BagEditorController(model, window)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()