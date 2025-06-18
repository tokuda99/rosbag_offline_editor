from pathlib import Path
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from PyQt5.QtWidgets import QFileDialog, QProgressDialog, QMessageBox, QDialog
from typing import Any
from app.views.camera_info_editor_dialog import CameraInfoEditorDialog


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

class BagSaveWorker(QObject):
    """
    バッグの保存という重い処理を「別スレッド」で実行する Worker。
    """
    # (success: bool, message: str) というシグナルを送る
    finished_signal = pyqtSignal(bool, str)

    def __init__(self, model, in_path, out_path, out_format, updated_meta):
        super().__init__()
        self.model = model
        self.in_path = in_path
        self.out_path = out_path
        self.out_format = out_format
        self.updated_meta = updated_meta

    def run(self):
        """
        スレッド開始時に呼ばれる処理。
        Model のメソッドを使ってバッグを保存する。
        例外が起きたらシグナルを (False, エラーメッセージ) で返す。
        """
        try:
            self.model.save_bag(
                self.in_path, self.out_path, self.out_format, self.updated_meta
            )
            # 成功メッセージを付けてシグナルを発行
            self.finished_signal.emit(True, f"Successfully saved to: {self.out_path}")
        except Exception as e:
            # 失敗メッセージを付けてシグナルを発行
            self.finished_signal.emit(False, f"Failed to save: {e}")

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
        self.view.request_save_bag.connect(self.save_bag_async)
        self.view.request_detail_edit.connect(self.open_detail_editor)

        # 非同期処理用
        self.thread = None
        self.worker = None

    def open_detail_editor(self, cid: int, msgtype: str):
        """
        ✅ 修正: メッセージ型に応じて専用の編集ダイアログを開く
        """
        if msgtype == "sensor_msgs/msg/CameraInfo":
            # 1. 編集対象の現在のメッセージ内容を取得
            current_msg = self.model.get_first_message(cid)
            if current_msg is None:
                # トピックが空などの理由でメッセージが取得できなかった場合
                self.view.show_warning_message(
                    f"Could not retrieve a message from topic '{self.model.meta_info[cid]['topic']}'.\n"
                    "Cannot open editor for an empty topic."
                )
                return

            # 2. 編集ダイアログを作成し、現在の値を渡して開く
            dialog = CameraInfoEditorDialog(current_msg, self.model.thumbnail_images, self.view)
            
            # 3. ダイアログが「OK」で閉じられたら、編集結果をモデルに保存
            if dialog.exec_() == QDialog.Accepted:
                edited_data = dialog.edited_data
                if edited_data:
                    self.model.detail_edit_data[cid] = {
                        'type': 'camera_info',
                        'data': edited_data
                    }
                    self.view.show_info_message("CameraInfo updated. Changes will be applied on save.")
        
        elif msgtype == "tf2_msgs/msg/TFMessage":
            # 将来のTF編集機能のためのスタブ
            QMessageBox.information(self.view, "Not Implemented", "Editor for TFMessage is not yet implemented.")
        
        else:
            self.view.show_error_message(f"No editor available for {msgtype}")
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
        self.model.bag_path = self.worker.path_obj

        display_meta_info = {}
        for cid, info in meta_info.items():
            info_copy = info.copy()
            # 「詳細編集可能か」のフラグをControllerが判断して追加
            info_copy['is_detail_editable'] = self.model.is_detail_edit_supported(info.get("msgtype", ""))
            display_meta_info[cid] = info_copy

        # View のテーブルを更新
        self.view.update_meta_table(display_meta_info)
        self.view.show_info_message("Metadata loaded. Bag is closed now.")

    def save_bag_async(self): # save_bag からリネームし、非同期化
        """
        「Save Modified Bag」ボタン押下時の処理。
        - ユーザに保存先を選択させ、非同期で保存処理を開始する
        """
        if not self.model.bag_path:
            self.view.show_error_message("Please open a bag file first.")
            return

        out_format = self.view.get_output_format()
        in_path = self.model.bag_path

        # 出力パスをユーザーに指定させる (GUI操作なのでメインスレッドで行う)
        if out_format == "ROS1":
            save_file, _ = QFileDialog.getSaveFileName(
                self.view, "Save as ROS1 Bag", filter="ROS1 Bag (*.bag)"
            )
            if not save_file:
                return
            out_path = Path(save_file)
        else: # ROS2
            # ROS2の場合はディレクトリを選択させるのが一般的
            save_dir = QFileDialog.getExistingDirectory(
                self.view, "Select Directory to Save ROS2 Bag"
            )
            if not save_dir:
                return
            # 新しいバッグ名（ディレクトリ名）を付ける
            out_path = Path(save_dir) / (in_path.stem + "_modified")


        # View 側で編集された meta_info を再取得
        edited_meta = self.view.get_edited_meta_info(self.model.meta_info)

        # プログレスダイアログを表示
        self.view.show_progress_dialog("Saving bag...", "Please wait...")

        # QThread と Worker を生成
        self.save_thread = QThread()
        self.save_worker = BagSaveWorker(
            self.model, in_path, out_path, out_format, edited_meta
        )
        self.save_worker.moveToThread(self.save_thread)

        # スレッド開始時に worker.run() を実行
        self.save_thread.started.connect(self.save_worker.run)

        # 保存完了シグナルを受け取ったらハンドラ呼び出し
        self.save_worker.finished_signal.connect(self.on_save_finished)

        # 終了時のクリーンアップ
        self.save_worker.finished_signal.connect(self.save_thread.quit)
        self.save_thread.finished.connect(self.save_worker.deleteLater)
        self.save_thread.finished.connect(self.save_thread.deleteLater)

        # スレッド起動
        self.save_thread.start()

    def on_save_finished(self, success: bool, message: str): # 新規追加
        """
        保存完了シグナルを受けたときの処理。
        """
        # プログレスダイアログを閉じる
        self.view.close_progress_dialog()

        if success:
            self.view.show_info_message(message)
        else:
            self.view.show_error_message(message)
