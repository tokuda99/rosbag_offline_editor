from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QTableWidget, QTableWidgetItem, QAbstractItemView,
    QComboBox, QMessageBox, QProgressDialog, QHeaderView
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5 import uic
import os
from typing import Dict, Any

class BagEditorWindow(QMainWindow):
    """
    rosbagの編集UIを提供する View クラス。
    MVCの「View」に相当し、ユーザ操作や画面表示を担当。

    Controller に通知するためのシグナル:
    - request_open_bag: 「Open Bag」ボタン押下時
    - request_save_bag: 「Save Bag」ボタン押下時
    """
    request_open_bag = pyqtSignal()
    request_save_bag = pyqtSignal()

    def __init__(self):
        super().__init__()
        ui_path = os.path.join(
            os.path.dirname(__file__),
            'ui',
            'bag_editor.ui'
        )
        uic.loadUi(ui_path, self)
        self.setWindowTitle("ROS Bag Editor (MVC)")

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # --- 操作ボタン行 ---
        button_layout = QHBoxLayout()

        self.open_button = QPushButton("Open Bag or Directory")
        # ボタンクリック → request_open_bagシグナルを送る
        self.open_button.clicked.connect(self.request_open_bag.emit)
        button_layout.addWidget(self.open_button)

        self.save_button = QPushButton("Save Modified Bag")
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.request_save_bag.emit)
        button_layout.addWidget(self.save_button)

        layout.addLayout(button_layout)

        # --- 出力形式選択 (ROS1 / ROS2) ---
        format_layout = QHBoxLayout()
        self.format_label = QLabel("Output Format:")
        format_layout.addWidget(self.format_label)

        self.format_combo = QComboBox()
        self.format_combo.addItems(["ROS1", "ROS2"])
        format_layout.addWidget(self.format_combo)

        layout.addLayout(format_layout)

        # --- メタ情報を表示するテーブル ---
        self.table = QTableWidget()
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        # ダブルクリックで編集可能
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked)
        layout.addWidget(self.table)
        self.table.itemChanged.connect(self._on_item_changed)

        # プログレスダイアログ (読み込み中など)
        self.progress_dialog = None

    # -------------------------------------------------------------------------
    # 以下、Controller から呼ばれるメソッド群
    # -------------------------------------------------------------------------
    def _on_item_changed(self, item: QTableWidgetItem):
        """
        ✅ 新規: テーブルのアイテム変更をハンドルする。
        チェックボックスの列で変更があった場合のみ、Saveボタンの状態を更新。
        """
        if item.column() == 0:  # 0列目=チェックボックス
            self._update_save_button_state()

    def _update_save_button_state(self):
        """
        ✅ 新規: チェックされたトピック数に応じてSaveボタンの有効/無効を切り替える。
        """
        checked_count = 0
        for i in range(self.table.rowCount()):
            item = self.table.item(i, 0)
            if item and item.checkState() == Qt.Checked:
                checked_count += 1
        
        self.save_button.setEnabled(checked_count > 0)
    
    
    def update_meta_table(self, meta_info: Dict[int, Dict[str, Any]]):
        """
        ✅ 修正: チェックボックス列を追加し、表示ロジックを更新。
        """
        # テーブル更新中の不要なシグナル発火を防ぐ
        self.table.blockSignals(True)
        self.table.clear()
        
        conn_ids = list(meta_info.keys())
        has_ros2_info = any("serialization_format" in info for info in meta_info.values())
        
        column_count = 4 + (4 if has_ros2_info else 0)
        self.table.setColumnCount(column_count)
        
        headers = ["Include", "Topic", "MsgType", "Frame ID"]
        if has_ros2_info:
            headers.extend(["Serialization Format", "QoS Durability", "QoS History", "QoS Reliability"])
        self.table.setHorizontalHeaderLabels(headers)
        self.table.setRowCount(len(conn_ids))

        qos_durability_options = ["SYSTEM_DEFAULT", "TRANSIENT_LOCAL", "VOLATILE", "UNKNOWN", "BEST_AVAILABLE"]
        qos_history_options = ["SYSTEM_DEFAULT", "KEEP_LAST", "KEEP_ALL", "UNKNOWN"]
        qos_reliability_options = ["SYSTEM_DEFAULT", "RELIABLE", "BEST_EFFORT", "UNKNOWN", "BEST_AVAILABLE"]

        for row, cid in enumerate(conn_ids):
            info = meta_info[cid]
            
            # 列0: チェックボックス
            checkbox_item = QTableWidgetItem()
            checkbox_item.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            checkbox_item.setCheckState(Qt.Checked)
            self.table.setItem(row, 0, checkbox_item)
            
            # 列1-3: 基本情報
            self.table.setItem(row, 1, QTableWidgetItem(info.get("topic", "")))
            self.table.setItem(row, 2, QTableWidgetItem(info.get("msgtype", "")))
            
            frame_id_item = QTableWidgetItem(info.get("frame_id", ""))
            # frame_idが無いトピックは編集不可に
            if "frame_id" not in info:
                frame_id_item.setFlags(frame_id_item.flags() & ~Qt.ItemIsEditable)
                frame_id_item.setBackground(Qt.lightGray)
            self.table.setItem(row, 3, frame_id_item)

            # 列4-7: ROS2追加情報
            if has_ros2_info and "serialization_format" in info:
                sf_item = QTableWidgetItem(info["serialization_format"])
                sf_item.setFlags(sf_item.flags() & ~Qt.ItemIsEditable) # 編集不可
                self.table.setItem(row, 4, sf_item)

                qos = info.get("qos")
                if qos:
                    # Durability
                    durability_combo = QComboBox()
                    durability_combo.addItems(qos_durability_options)
                    durability_combo.setCurrentText(qos.durability.name)
                    self.table.setCellWidget(row, 5, durability_combo)
                    # History
                    history_combo = QComboBox()
                    history_combo.addItems(qos_history_options)
                    history_combo.setCurrentText(qos.history.name)
                    self.table.setCellWidget(row, 6, history_combo)
                    # Reliability
                    reliability_combo = QComboBox()
                    reliability_combo.addItems(qos_reliability_options)
                    reliability_combo.setCurrentText(qos.reliability.name)
                    self.table.setCellWidget(row, 7, reliability_combo)

        # シグナルのブロックを解除し、Saveボタンの状態を初期化
        self.table.blockSignals(False)
        self._update_save_button_state()

    def get_edited_meta_info(self, original_meta_info: Dict) -> Dict:
        """
        ✅ 修正: チェックされたトピックの情報と、編集されたframe_idを収集して返す。
        """
        edited_meta = {}
        conn_ids = list(original_meta_info.keys())
        
        for row, cid in enumerate(conn_ids):
            if self.table.item(row, 0).checkState() == Qt.Checked:
                info_copy = original_meta_info[cid].copy()
                info_copy["topic"] = self.table.item(row, 1).text()
                info_copy["msgtype"] = self.table.item(row, 2).text()
                
                # frame_idが存在すれば更新
                if "frame_id" in info_copy:
                    info_copy["frame_id"] = self.table.item(row, 3).text()
                
                # QoSが存在すれば更新 (今回は実装省略)
                if "qos" in info_copy:
                    # ToDo: QComboBoxから値を取得してQoSプロファイルを更新する
                    pass

                edited_meta[cid] = info_copy
        return edited_meta

    def get_output_format(self):
        """
        現在選択中の「出力形式 (ROS1/ROS2)」を返す
        """
        return self.format_combo.currentText()

    def show_progress_dialog(self, label_text, cancel_text):
        """
        読み込み中などで操作をブロックしたいときに表示するプログレスダイアログ。
        """
        self.progress_dialog = QProgressDialog(label_text, cancel_text, 0, 0, self)
        self.progress_dialog.setWindowModality(Qt.WindowModal)
        self.progress_dialog.setValue(0)  # 進捗バー不定状態
        # self.progress_dialog.canceled.connect(...) -> キャンセル処理はお好みで
        self.progress_dialog.show()

    def close_progress_dialog(self):
        if self.progress_dialog:
            self.progress_dialog.close()
            self.progress_dialog = None

    def show_info_message(self, text: str):
        QMessageBox.information(self, "Info", text)

    def show_error_message(self, text: str):
        QMessageBox.critical(self, "Error", text)
