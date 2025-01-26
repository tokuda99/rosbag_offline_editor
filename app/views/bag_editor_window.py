from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QTableWidget, QTableWidgetItem, QAbstractItemView,
    QComboBox, QMessageBox, QProgressDialog, QHeaderView
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5 import uic
import os

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

        # プログレスダイアログ (読み込み中など)
        self.progress_dialog = None

    # -------------------------------------------------------------------------
    # 以下、Controller から呼ばれるメソッド群
    # -------------------------------------------------------------------------
    def update_meta_table(self, meta_info):
        conn_ids = list(meta_info.keys())

        # 必要な列数を計算する
        extra_columns = 0
        for cid in conn_ids:
            info = meta_info[cid]
            if "serialization_format" in info:
                extra_columns = max(extra_columns, 4)

        column_count = 3 + extra_columns
        self.table.setColumnCount(column_count)

        # ヘッダーを設定
        headers = ["Topic", "MsgType", "Frame ID"]
        if extra_columns > 0:
            headers.extend(["Serialization Format", "QoS History", "Qos Reliability", "Qos Durability"])
        self.table.setHorizontalHeaderLabels(headers)

        self.table.setRowCount(len(conn_ids))

        # QoSオプションを定義
        qos_history_options = ["SystemDefault", "KeepLast", "KeepAll", "Unknown"]
        qos_reliability_options = ["SystemDefault", "Reliable", "BestEffort", "Unknown"]
        qos_durability_options = ["SystemDefault", "TransientLocal", "Volatile", "Unknown"]

        for row, cid in enumerate(conn_ids):
            info = meta_info[cid]
            topic = info["topic"]
            msgtype = info["msgtype"]
            frame_id = info["frame_id"] if "frame_id" in info else ""

            self.table.setItem(row, 0, QTableWidgetItem(topic))
            self.table.setItem(row, 1, QTableWidgetItem(msgtype))
            self.table.setItem(row, 2, QTableWidgetItem(frame_id))

            if "serialization_format" in info:
                self.table.setItem(row, 3, QTableWidgetItem(info["serialization_format"]))

                # QoS History
                history_combo = QComboBox()
                history_combo.addItems(qos_history_options)
                history_combo.setCurrentText(info["qos"].history)
                self.table.setCellWidget(row, 4, history_combo)

                # QoS Reliability
                reliability_combo = QComboBox()
                reliability_combo.addItems(qos_reliability_options)
                reliability_combo.setCurrentText(info["qos"].reliability)
                self.table.setCellWidget(row, 5, reliability_combo)

                # QoS Durability
                durability_combo = QComboBox()
                durability_combo.addItems(qos_durability_options)
                durability_combo.setCurrentText(info["qos"].durability)
                self.table.setCellWidget(row, 6, durability_combo)

        self.save_button.setEnabled(True)

    def get_edited_meta_info(self, meta_info):
        """
        テーブルで編集されたトピック名/型を取り出して、
        meta_info ディクショナリに反映して返す。
        """
        conn_ids = list(meta_info.keys())
        for row, cid in enumerate(conn_ids):
            t_item = self.table.item(row, 0)
            m_item = self.table.item(row, 1)
            if t_item and m_item:
                meta_info[cid]["topic"] = t_item.text()
                meta_info[cid]["msgtype"] = m_item.text()
        return meta_info

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
