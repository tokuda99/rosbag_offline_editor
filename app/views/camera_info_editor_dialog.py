import cv2
import numpy as np
import yaml
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
)


def numpy_to_pixmap(np_image: np.ndarray, target_width=480) -> QPixmap:
    """NumPy配列(BGR)をQPixmapに変換するヘルパー関数"""
    if np_image is None or np_image.size == 0:
        return QPixmap()
        
    # グレースケール画像の場合、カラーに変換
    if len(np_image.shape) == 2:
        np_image = cv2.cvtColor(np_image, cv2.COLOR_GRAY2BGR)

    height, width, channel = np_image.shape
    bytes_per_line = channel * width
    
    # QImage.Format_BGR888 はQt 5.14以降
    # 古いバージョンでも動作するように、RGBに変換してから作成する
    if channel == 3:
        image_rgb = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        q_image = QImage(image_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
    else: # Fallback for other formats
        q_image = QImage(np_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

    pixmap = QPixmap.fromImage(q_image)
    return pixmap.scaledToWidth(target_width, Qt.SmoothTransformation)

class CameraInfoEditorDialog(QDialog):
    """
    sensor_msgs/msg/CameraInfo を編集するためのダイアログ。
    """
    def __init__(self, initial_camera_info, thumbnail_images, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit CameraInfo")
        self.setMinimumSize(1200, 800)

        # 編集前の初期状態を保存 (リセット機能のため)
        self.initial_msg = initial_camera_info
        self.thumbnails = thumbnail_images
        self.current_preview_image = None
        self.edited_data = None

        # --- メインレイアウト (左右分割) ---
        main_layout = QHBoxLayout(self)

        # --- 左パネル: コントロール ---
        left_panel = QVBoxLayout()

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

        # 3. OK / Cancel / Reset ボタン
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.reset_button = self.button_box.addButton("Reset", QDialogButtonBox.ResetRole)
        self.reset_button.clicked.connect(self._reset_fields)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

        # 左パネルにウィジェットを追加
        left_panel.addWidget(load_group)
        left_panel.addWidget(fields_group)
        left_panel.addStretch()
        left_panel.addWidget(self.button_box)

        # --- 右パネル: 画像プレビュー ---
        right_panel = QVBoxLayout()
        preview_group = QGroupBox("Preview")
        preview_layout = QVBoxLayout()
        
        self.image_selector = QComboBox()
        self.image_selector.addItems(self.thumbnails.keys())
        preview_layout.addWidget(QLabel("Select Image for Preview:"))
        preview_layout.addWidget(self.image_selector)
        
        self.before_label = QLabel("Before (Original)")
        self.before_label.setAlignment(Qt.AlignCenter)
        self.before_label.setMinimumSize(480, 320)
        self.before_label.setStyleSheet("border: 1px solid grey; background-color: #333;")
        preview_layout.addWidget(self.before_label)

        self.after_label = QLabel("After (Undistorted)")
        self.after_label.setAlignment(Qt.AlignCenter)
        self.after_label.setMinimumSize(480, 320)
        self.after_label.setStyleSheet("border: 1px solid grey; background-color: #333;")
        preview_layout.addWidget(self.after_label)
        
        preview_group.setLayout(preview_layout)
        right_panel.addWidget(preview_group)

        # メインレイアウトに左右パネルを追加
        main_layout.addLayout(left_panel, 1) # 比率1
        main_layout.addLayout(right_panel, 2) # 比率2
        
        # --- イベント接続と初期化 ---
        self.image_selector.currentIndexChanged.connect(self._on_image_selection_changed)
        for editor in [self.width_edit, self.height_edit, self.model_edit, self.k_edit, self.d_edit, self.r_edit, self.p_edit]:
            editor.textChanged.connect(self._update_preview)
        
        # 初期値をUIに設定
        self._populate_fields(self.initial_msg)
        if self.thumbnails:
            self._on_image_selection_changed()
        else:
            preview_group.setEnabled(False)
            self.before_label.setText("No compatible image topics found in bag.")

    def _populate_fields(self, msg):
        """メッセージオブジェクトからUIフィールドに値を設定する"""
        if not msg:
            return
        self.width_edit.setText(str(getattr(msg, 'width', 0)))
        self.height_edit.setText(str(getattr(msg, 'height', 0)))
        self.model_edit.setText(getattr(msg, 'distortion_model', ''))
        
        self.k_edit.setText(', '.join(f"{x:.6f}" for x in getattr(msg, 'k', []).flatten()))
        self.d_edit.setText(', '.join(f"{x:.6f}" for x in getattr(msg, 'd', []).flatten()))
        self.r_edit.setText(', '.join(f"{x:.6f}" for x in getattr(msg, 'r', []).flatten()))
        self.p_edit.setText(', '.join(f"{x:.6f}" for x in getattr(msg, 'p', []).flatten()))

    def _load_from_file(self):
        """YAMLファイルを読み込み、UIに反映する"""
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
        """YAML辞書データでUIフィールドを埋める"""
        if not isinstance(data_dict, dict):
            raise ValueError("YAML content is not a valid dictionary.")

        self.width_edit.setText(str(data_dict.get('image_width', '')))
        self.height_edit.setText(str(data_dict.get('image_height', '')))
        self.model_edit.setText(data_dict.get('distortion_model', ''))
        
        self.k_edit.setText(', '.join(map(str, data_dict.get('camera_matrix', {}).get('data', []))))
        self.d_edit.setText(', '.join(map(str, data_dict.get('distortion_coefficients', {}).get('data', []))))
        self.r_edit.setText(', '.join(map(str, data_dict.get('rectification_matrix', {}).get('data', []))))
        self.p_edit.setText(', '.join(map(str, data_dict.get('projection_matrix', {}).get('data', []))))
        
        QMessageBox.information(self, "Success", "Values loaded successfully from source.")

    def _reset_fields(self):
        """全てのフィールドをダイアログを開いた時点の初期値に戻す"""
        self._populate_fields(self.initial_msg)
        self._update_preview()
        QMessageBox.information(self, "Reset", "Parameters have been reset to their initial values.")

    def _on_image_selection_changed(self):
        """プレビュー用画像が変更されたときの処理"""
        topic_name = self.image_selector.currentText()
        if not topic_name:
            self.current_preview_image = None
            self.before_label.clear()
            self.after_label.clear()
            return
            
        self.current_preview_image = self.thumbnails.get(topic_name)
        if self.current_preview_image is not None:
            self.before_label.setPixmap(numpy_to_pixmap(self.current_preview_image))
        else:
            self.before_label.setText(f"Preview for\n{topic_name}\nis not available.")
            
        self._update_preview()

    def _update_preview(self):
        """パラメータ変更に応じてプレビューのAfter画像を更新する"""
        if self.current_preview_image is None:
            self.after_label.clear()
            return

        params = self.get_edited_data_as_dict(show_error=False)
        if not params:
            self.after_label.setText("Invalid Parameters")
            return
        
        try:
            k = np.array(params['k']).reshape(3, 3)
            d = np.array(params['d'])
            
            h, w = self.current_preview_image.shape[:2]
            new_k, roi = cv2.getOptimalNewCameraMatrix(k, d, (w,h), 0, (w,h))

            undistorted_img = cv2.undistort(self.current_preview_image, k, d, None, new_k)
            self.after_label.setPixmap(numpy_to_pixmap(undistorted_img))
        except Exception as e:
            self.after_label.setText(f"Error during undistortion:\n{str(e)}")

    def get_edited_data_as_dict(self, show_error=True):
        """UIフィールドから値を取得し、辞書として返す (エラー表示オプション付き)"""
        try:
            data = {
                'width': int(self.width_edit.text()),
                'height': int(self.height_edit.text()),
                'distortion_model': self.model_edit.text(),
                'k': [float(x.strip()) for x in self.k_edit.text().split(',') if x.strip()],
                'd': [float(x.strip()) for x in self.d_edit.text().split(',') if x.strip()],
                'r': [float(x.strip()) for x in self.r_edit.text().split(',') if x.strip()],
                'p': [float(x.strip()) for x in self.p_edit.text().split(',') if x.strip()],
            }
            if len(data['k']) != 9: raise ValueError("Camera Matrix (K) must have 9 elements.")
            if len(data['r']) != 9: raise ValueError("Rectification Matrix (R) must have 9 elements.")
            if len(data['p']) != 12: raise ValueError("Projection Matrix (P) must have 12 elements.")
            return data
        except (ValueError, TypeError) as e:
            if show_error:
                QMessageBox.critical(self, "Validation Error", f"Invalid input value: {e}")
            return None

    def accept(self):
        """OKボタン: バリデーション付きでデータを取得し、ダイアログを閉じる"""
        data = self.get_edited_data_as_dict(show_error=True)
        if data:
            self.edited_data = data
            super().accept()