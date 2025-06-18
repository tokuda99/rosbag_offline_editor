import yaml
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QLineEdit, QPushButton, QDialogButtonBox,
    QFileDialog, QMessageBox, QPlainTextEdit, QLabel, QHBoxLayout, QGroupBox
)
from PyQt5.QtCore import Qt
import numpy as np

class CameraInfoEditorDialog(QDialog):
    """
    sensor_msgs/msg/CameraInfo を編集するためのダイアログ。
    """
    def __init__(self, camera_info_msg, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit CameraInfo")
        self.setMinimumWidth(600)

        # メインレイアウト
        layout = QVBoxLayout(self)

        # --- 1. 外部からの読み込み機能 ---
        load_group = QGroupBox("Load from External Source")
        load_layout = QVBoxLayout()
        
        # ファイルから読み込み
        file_button_layout = QHBoxLayout()
        self.load_file_button = QPushButton("Load from camera_info.yaml...")
        self.load_file_button.clicked.connect(self._load_from_file)
        file_button_layout.addWidget(self.load_file_button)
        file_button_layout.addStretch()
        load_layout.addLayout(file_button_layout)
        
        # テキストから読み込み
        load_layout.addWidget(QLabel("Or paste YAML content here:"))
        self.yaml_text_edit = QPlainTextEdit()
        self.yaml_text_edit.setPlaceholderText("Paste camera_info.yaml content here...")
        load_layout.addWidget(self.yaml_text_edit)
        
        parse_button_layout = QHBoxLayout()
        parse_button_layout.addStretch()
        self.parse_text_button = QPushButton("Parse Pasted Text")
        self.parse_text_button.clicked.connect(self._parse_from_text)
        parse_button_layout.addWidget(self.parse_text_button)
        load_layout.addLayout(parse_button_layout)
        
        load_group.setLayout(load_layout)
        layout.addWidget(load_group)

        # --- 2. CameraInfo フィールド編集機能 ---
        fields_group = QGroupBox("Edit Fields Directly")
        form_layout = QFormLayout()

        self.width_edit = QLineEdit()
        self.height_edit = QLineEdit()
        self.model_edit = QLineEdit()
        self.k_edit = QLineEdit() # Camera Matrix K
        self.d_edit = QLineEdit() # Distortion Coeffs D
        self.r_edit = QLineEdit() # Rectification Matrix R
        self.p_edit = QLineEdit() # Projection Matrix P

        form_layout.addRow("Image Width:", self.width_edit)
        form_layout.addRow("Image Height:", self.height_edit)
        form_layout.addRow("Distortion Model:", self.model_edit)
        form_layout.addRow("Camera Matrix (K):", self.k_edit)
        form_layout.addRow("Distortion Coeffs (D):", self.d_edit)
        form_layout.addRow("Rectification Matrix (R):", self.r_edit)
        form_layout.addRow("Projection Matrix (P):", self.p_edit)
        
        fields_group.setLayout(form_layout)
        layout.addWidget(fields_group)

        # --- 3. OK / Cancel ボタン ---
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box)

        # 初期値をUIに設定
        self._populate_fields(camera_info_msg)
        self.edited_data = None

    def _populate_fields(self, msg):
        """メッセージオブジェクトからUIフィールドに値を設定する"""
        if not msg:
            return
        self.width_edit.setText(str(getattr(msg, 'width', '')))
        self.height_edit.setText(str(getattr(msg, 'height', '')))
        self.model_edit.setText(getattr(msg, 'distortion_model', ''))
        
        # numpy配列をカンマ区切りの文字列に変換
        self.k_edit.setText(', '.join(map(str, getattr(msg, 'k', []).flatten())))
        self.d_edit.setText(', '.join(map(str, getattr(msg, 'd', []).flatten())))
        self.r_edit.setText(', '.join(map(str, getattr(msg, 'r', []).flatten())))
        self.p_edit.setText(', '.join(map(str, getattr(msg, 'p', []).flatten())))

    def _load_from_file(self):
        """YAMLファイルを読み込み、パースしてUIに反映する"""
        path, _ = QFileDialog.getOpenFileName(self, "Open camera_info.yaml", "", "YAML files (*.yaml *.yml)")
        if not path:
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self._populate_from_dict(data)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load or parse YAML file: {e}")

    def _parse_from_text(self):
        """テキストエリアのYAMLをパースしてUIに反映する"""
        text = self.yaml_text_edit.toPlainText()
        if not text:
            QMessageBox.warning(self, "Warning", "Text area is empty.")
            return
        try:
            data = yaml.safe_load(text)
            self._populate_from_dict(data)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to parse YAML text: {e}")

    def _populate_from_dict(self, data_dict):
        """YAMLから読み込んだ辞書データでUIフィールドを埋める"""
        if not isinstance(data_dict, dict):
            raise ValueError("YAML content is not a valid dictionary.")

        self.width_edit.setText(str(data_dict.get('image_width', '')))
        self.height_edit.setText(str(data_dict.get('image_height', '')))
        self.model_edit.setText(data_dict.get('distortion_model', ''))
        
        # 'camera_matrix' や 'distortion_coefficients' のようなキーに対応
        self.k_edit.setText(', '.join(map(str, data_dict.get('camera_matrix', {}).get('data', []))))
        self.d_edit.setText(', '.join(map(str, data_dict.get('distortion_coefficients', {}).get('data', []))))
        self.r_edit.setText(', '.join(map(str, data_dict.get('rectification_matrix', {}).get('data', []))))
        self.p_edit.setText(', '.join(map(str, data_dict.get('projection_matrix', {}).get('data', []))))
        
        QMessageBox.information(self, "Success", "Values loaded successfully from source.")

    def get_edited_data_as_dict(self):
        """UIフィールドから値を取得し、辞書として返す"""
        try:
            data = {
                'width': int(self.width_edit.text()),
                'height': int(self.height_edit.text()),
                'distortion_model': self.model_edit.text(),
                # 文字列をfloatのリストに変換
                'k': [float(x.strip()) for x in self.k_edit.text().split(',') if x.strip()],
                'd': [float(x.strip()) for x in self.d_edit.text().split(',') if x.strip()],
                'r': [float(x.strip()) for x in self.r_edit.text().split(',') if x.strip()],
                'p': [float(x.strip()) for x in self.p_edit.text().split(',') if x.strip()],
            }
            # バリデーション
            if len(data['k']) != 9: raise ValueError("Camera Matrix (K) must have 9 elements.")
            if len(data['r']) != 9: raise ValueError("Rectification Matrix (R) must have 9 elements.")
            if len(data['p']) != 12: raise ValueError("Projection Matrix (P) must have 12 elements.")
            return data
        except ValueError as e:
            QMessageBox.critical(self, "Validation Error", f"Invalid input: {e}")
            return None

    def accept(self):
        """OKボタンが押された時の処理"""
        data = self.get_edited_data_as_dict()
        if data:
            self.edited_data = data
            super().accept()