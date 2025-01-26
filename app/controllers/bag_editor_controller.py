from pathlib import Path
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import QFileDialog, QProgressDialog, QMessageBox
from typing import Any

class BagMetaWorker(QObject):
    """
    バッグのメタデータを読み込む重い処理を「別スレッド」で実行する Worker。
    """
    finished_signal = pyqtSignal(bool, object)
    # (success: bool, result: object) というシグナルを送る

    def __init__(self, model, path_obj: Path):
        super().__init__()
        self.model = model
        self.path_obj = path_obj

    def run(self):
        """
        スレッド開始時に呼ばれる処理。
        Model のメソッドを使ってメタ情報を取得する。
        例外が起きたらシグナルを (False, エラーメッセージ) で返す。
        """
        try:
            rosbag_version, meta_info = self.model.load_bag_metadata(self.path_obj)
            self.finished_signal.emit(True, (rosbag_version, meta_info))
        except Exception as e:
            self.finished_signal.emit(False, str(e))


class BagEditorController:
    """
    View(画面) と Model(バッグ操作ロジック) の仲介役。
    - 「Open Bag」ボタンが押されたらファイル選択ダイアログを出し、ワーカーで非同期読み込み
    - 読込結果を View に反映
    - 「Save」ボタン押下時に、Model の save_bag を呼び出す
    """

    def __init__(self, model, view):
        self.model = model
        self.view = view

        # View からの「ユーザー操作」シグナルを受け取って処理する
        self.view.request_open_bag.connect(self.open_bag_async)
        self.view.request_save_bag.connect(self.save_bag)

        # 非同期処理用
        self.thread = None
        self.worker = None

    def open_bag_async(self):
        """
        Bag (ディレクトリ or .bagファイル) をユーザに選択させて、非同期でメタデータ読込。
        """
        path = QFileDialog.getExistingDirectory(self.view, "Select Bag Directory")
        if not path:
            # ディレクトリが選ばれていなければ、.bag ファイル選択を試みる
            path, _ = QFileDialog.getOpenFileName(
                self.view, "Select .bag File", filter="Bag Files (*.bag)"
            )
            if not path:
                return
        path_obj = Path(path)

        # プログレスダイアログを表示
        self.view.show_progress_dialog("Loading bag...", "Cancel")

        # QThread と Worker を生成
        self.thread = QThread()
        self.worker = BagMetaWorker(self.model, path_obj)
        self.worker.moveToThread(self.thread)

        # スレッド開始時に worker.run() を実行
        self.thread.started.connect(self.worker.run)

        # 読み込み完了シグナルを受け取ったらハンドラ呼び出し
        self.worker.finished_signal.connect(self.on_load_finished)

        # 終了時のクリーンアップ
        self.worker.finished_signal.connect(self.thread.quit)
        self.thread.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)

        # スレッド起動
        self.thread.start()

    def on_load_finished(self, success, result):
        """
        メタデータ読み込み完了シグナルを受けたときの処理。
        """
        # プログレスダイアログを閉じる
        self.view.close_progress_dialog()

        if not success:
            error_msg = result  # 文字列
            self.view.show_error_message(f"Failed to load bag: {error_msg}")
            return

        # 成功した場合
        rosbag_version, meta_info = result
        self.model.rosbag_version = rosbag_version
        self.model.meta_info = meta_info

        # 今回読み込んだバッグのパスを保存しておく
        # (後で Save のときに利用)
        # Worker 内では self.model.bag_path へ代入してもよいですが、ここで明示的に行う方が分かりやすい
        self.model.bag_path = self.worker.path_obj

        # View のテーブルを更新
        self.view.update_meta_table(meta_info)
        self.view.show_info_message("Metadata loaded. Bag is closed now.")

    def save_bag(self):
        """
        「Save Modified Bag」ボタン押下時の処理。
        - ユーザに保存先(ROS1 or ROS2)を選択させる
        - View からテーブル内容を取得
        - Model の save_bag を呼び出して書き込む
        """
        if not self.model.bag_path:
            return

        out_format = self.view.get_output_format()
        in_path = self.model.bag_path

        # 出力パスをユーザーに指定させる
        if out_format == "ROS1":
            save_file, _ = QFileDialog.getSaveFileName(
                self.view, "Save as ROS1 Bag", filter="ROS1 Bag (*.bag)"
            )
            if not save_file:
                return
            out_path = Path(save_file)
        else:
            save_dir, _ = QFileDialog.getSaveFileName(
                self.view, "Specify Directory Name for ROS2 Bag",
                directory=str(Path.home() / "new_ros2_bag"),
                filter="Directories (*)"
            )
            if not save_dir:
                return
            out_path = Path(save_dir)

        # View 側で編集された meta_info を再取得
        edited_meta = self.view.get_edited_meta_info(self.model.meta_info)

        # モデルに書き込みを依頼
        try:
            self.model.save_bag(in_path, out_path, out_format, edited_meta)
            self.view.show_info_message(f"Saved to: {out_path}")
        except Exception as e:
            self.view.show_error_message(f"Failed to save: {e}")
