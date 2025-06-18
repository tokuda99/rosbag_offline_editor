# app/views/tf_static_editor_dialog.py

import yaml
from collections import defaultdict, deque
import numpy as np
from scipy.spatial.transform import Rotation

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QLineEdit, QPushButton, QDialogButtonBox,
    QMessageBox, QPlainTextEdit, QLabel, QHBoxLayout, QGroupBox,
    QTreeWidget, QTreeWidgetItem, QComboBox, QDoubleSpinBox, QSlider, QGridLayout
)
from PyQt5.QtCore import Qt

# PyVistaのQt連携機能をインポート
import pyvista as pv
from pyvistaqt import QtInteractor

# --- ヘルパー関数 ---

def rpy_to_quaternion(roll, pitch, yaw):
    """Roll, Pitch, Yaw (度数法) をクォータニオンに変換"""
    # Scipyはラジアンを期待するので変換
    r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    quat = r.as_quat() # [x, y, z, w]
    return quat

def quaternion_to_rpy(quat):
    """クォータニオン [x, y, z, w] をRoll, Pitch, Yaw (度数法) に変換"""
    if not np.any(quat): return 0.0, 0.0, 0.0
    r = Rotation.from_quat(quat)
    euler = r.as_euler('xyz', degrees=True)
    return euler[0], euler[1], euler[2]


class TFStaticEditorDialog(QDialog):
    def __init__(self, tf_message, thumbnail_pointclouds, meta_info, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit TF Static")
        self.setMinimumSize(1600, 900)

        # データ保持
        self.pointclouds = thumbnail_pointclouds
        self.tf_graph = defaultdict(list)
        self.tf_poses = {} # グローバル座標系での各フレームの姿勢
        self.meta_info = meta_info
        # ✅ トピック名からcidを逆引きするための辞書を作成
        self.topic_to_cid = {info['topic']: cid for cid, info in self.meta_info.items()}
        
        initial_transforms = tf_message.transforms if tf_message else []
        try:
            # ✅【修正箇所】初期状態を専用の変数に保存する
            self.initial_transforms = self._deduplicate_and_validate_transforms(initial_transforms)
            # self.transforms は編集用のワーキングコピー
            self.transforms = list(self.initial_transforms)
        except ValueError as e:
            QMessageBox.critical(self, "Initial TF Data Error", str(e))
            self.initial_transforms = []
            self.transforms = []

        # --- メインレイアウト (左右分割) ---
        main_layout = QHBoxLayout(self)
        self.setLayout(main_layout)

        # --- 左パネル: コントロール ---
        left_panel = QVBoxLayout()
        
        # 1. YAML入力
        yaml_group = QGroupBox("Load TF from Pasted YAML")
        yaml_layout = QVBoxLayout()
        self.yaml_text_edit = QPlainTextEdit()
        self.yaml_text_edit.setPlaceholderText("Paste 'rostopic echo /tf_static' output here...")
        parse_button = QPushButton("Parse Pasted Text and Rebuild Tree")
        parse_button.clicked.connect(self._parse_yaml)
        yaml_layout.addWidget(self.yaml_text_edit)
        yaml_layout.addWidget(parse_button)
        yaml_group.setLayout(yaml_layout)
        left_panel.addWidget(yaml_group)

        # 2. TFツリー表示
        tree_group = QGroupBox("TF Tree")
        tree_layout = QVBoxLayout()
        self.tree_widget = QTreeWidget()
        self.tree_widget.setHeaderLabels(["Frame ID"])
        tree_layout.addWidget(self.tree_widget)
        tree_group.setLayout(tree_layout)
        left_panel.addWidget(tree_group, 1) # 伸縮比率

        # 3. TF編集
        edit_group = QGroupBox("Transform Editor")
        edit_layout = QGridLayout()
        self.transform_selector = QComboBox()
        self.transform_selector.currentIndexChanged.connect(self._on_transform_selected)
        
        # Translation
        self.trans_x_spin = QDoubleSpinBox(); self.trans_x_spin.setRange(-100, 100); self.trans_x_spin.setDecimals(4); self.trans_x_spin.setSingleStep(0.01)
        self.trans_y_spin = QDoubleSpinBox(); self.trans_y_spin.setRange(-100, 100); self.trans_y_spin.setDecimals(4); self.trans_y_spin.setSingleStep(0.01)
        self.trans_z_spin = QDoubleSpinBox(); self.trans_z_spin.setRange(-100, 100); self.trans_z_spin.setDecimals(4); self.trans_z_spin.setSingleStep(0.01)
        
        # Rotation (Roll, Pitch, Yaw)
        self.roll_spin = QDoubleSpinBox(); self.roll_spin.setRange(-180, 180); self.roll_spin.setDecimals(4)
        self.pitch_spin = QDoubleSpinBox(); self.pitch_spin.setRange(-90, 90); self.pitch_spin.setDecimals(4)
        self.yaw_spin = QDoubleSpinBox(); self.yaw_spin.setRange(-180, 180); self.yaw_spin.setDecimals(4)

        edit_layout.addWidget(QLabel("Target Frame:"), 0, 0, 1, 2)
        edit_layout.addWidget(self.transform_selector, 0, 2, 1, 4)
        edit_layout.addWidget(QLabel("Translation (m)"), 1, 0, 1, 2)
        edit_layout.addWidget(QLabel("X:"), 2, 0); edit_layout.addWidget(self.trans_x_spin, 2, 1)
        edit_layout.addWidget(QLabel("Y:"), 3, 0); edit_layout.addWidget(self.trans_y_spin, 3, 1)
        edit_layout.addWidget(QLabel("Z:"), 4, 0); edit_layout.addWidget(self.trans_z_spin, 4, 1)
        
        edit_layout.addWidget(QLabel("Rotation (deg)"), 1, 3, 1, 2)
        edit_layout.addWidget(QLabel("Roll:"), 2, 3); edit_layout.addWidget(self.roll_spin, 2, 4)
        edit_layout.addWidget(QLabel("Pitch:"), 3, 3); edit_layout.addWidget(self.pitch_spin, 3, 4)
        edit_layout.addWidget(QLabel("Yaw:"), 4, 3); edit_layout.addWidget(self.yaw_spin, 4, 4)
        edit_group.setLayout(edit_layout)
        left_panel.addWidget(edit_group)

        # 4. OK / Cancel ボタン
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.reset_button = self.button_box.addButton("Reset to Initial State", QDialogButtonBox.ResetRole)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.reset_button.clicked.connect(self._reset_to_initial_state)
        left_panel.addWidget(self.button_box)

        # --- 右パネル: 3Dプレビュー ---
        right_panel = QVBoxLayout()
        preview_group = QGroupBox("3D Preview")
        preview_layout = QVBoxLayout()
        
        # PointCloudセレクタ
        self.pointcloud_selector = QComboBox()
        self.pointcloud_selector.addItems(["None"] + list(self.pointclouds.keys()))
        self.pointcloud_selector.currentIndexChanged.connect(self._update_3d_preview)
        preview_layout.addWidget(QLabel("Display PointCloud:"))
        preview_layout.addWidget(self.pointcloud_selector)
        
        # PyVista 3D Viewer
        self.plotter = QtInteractor(self)
        preview_layout.addWidget(self.plotter.interactor)
        preview_group.setLayout(preview_layout)
        right_panel.addWidget(preview_group)

        # メインレイアウトにパネルを追加
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 2)
        
        # --- イベント接続 ---
        for spin in [self.trans_x_spin, self.trans_y_spin, self.trans_z_spin, self.roll_spin, self.pitch_spin, self.yaw_spin]:
            spin.valueChanged.connect(self._on_transform_edited)

        # --- 初期化 ---
        self._rebuild_all()

    # --- メインロジック ---
    def _rebuild_all(self):
        """現在のself.transformsから全てを再構築する"""
        self._build_tf_graph()
        self._populate_transform_selector()
        self._populate_tree_widget()
        self._update_3d_preview()
        
    def _parse_yaml(self):
        """YAMLテキストをパースして self.transforms を更新し、再構築する"""
        text = self.yaml_text_edit.toPlainText()
        if not text:
            return
        try:
            # '---'で区切られた複数のYAMLドキュメントを読み込む
            docs = yaml.safe_load_all(text)
            parsed_transforms = []
            for doc in docs:
                if doc and 'transforms' in doc:
                    for tf_dict in doc['transforms']:
                        parsed_transforms.append(self._dict_to_transform_stamped(tf_dict))
            self.transforms = self._deduplicate_and_validate_transforms(parsed_transforms)
            self._rebuild_all()
            QMessageBox.information(self, "Success", f"Parsed and rebuilt tree with {len(self.transforms)} transforms.")
        except ValueError as e:
            # 重複排除中に発生した矛盾エラーをキャッチ
            QMessageBox.critical(self, "TF Data Error", str(e))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to parse YAML: {e}")
            
    # --- TFツリー関連 ---
    def _build_tf_graph(self):
        """self.transformsから親子関係のグラフを構築する"""
        self.tf_graph.clear()
        all_frames = set()
        for tf in self.transforms:
            self.tf_graph[tf.header.frame_id].append(tf.child_frame_id)
            all_frames.add(tf.header.frame_id)
            all_frames.add(tf.child_frame_id)
        
        # 親がいないルートフレームを見つける
        children = {child for child_list in self.tf_graph.values() for child in child_list}
        self.root_frames = [frame for frame in all_frames if frame not in children]

    def _populate_tree_widget(self):
        """self.tf_graphからQTreeWidgetを構築する"""
        self.tree_widget.clear()
        for root in self.root_frames:
            root_item = QTreeWidgetItem(self.tree_widget, [root])
            self._add_tree_children(root_item, root)
        self.tree_widget.expandAll()

    def _add_tree_children(self, parent_item, parent_frame):
        """TFツリーを再帰的に構築するヘルパー"""
        if parent_frame in self.tf_graph:
            for child_frame in sorted(self.tf_graph[parent_frame]):
                child_item = QTreeWidgetItem(parent_item, [child_frame])
                self._add_tree_children(child_item, child_frame)
    
    # --- 編集関連 ---
    def _populate_transform_selector(self):
        """✅【改善2】編集対象のTFを親子関係でソートしてComboBoxに設定する"""
        self.transform_selector.blockSignals(True)
        self.transform_selector.clear()

        # TFをトポロジカルソートする
        sorted_transforms = []
        q = deque(sorted(list(self.root_frames)))
        processed_parents = set()

        # TFをchild_frame_idで素早く引けるように辞書を作成
        tf_map_child_key = {tf.child_frame_id: tf for tf in self.transforms}

        while q:
            parent_frame = q.popleft()
            if parent_frame in processed_parents:
                continue
            processed_parents.add(parent_frame)

            # この親から出る子TFを、子の名前でソートする
            child_frames = sorted(self.tf_graph.get(parent_frame, []))
            
            for child_frame in child_frames:
                if child_frame in tf_map_child_key:
                    sorted_transforms.append(tf_map_child_key[child_frame])
                    q.append(child_frame) # 次の探索対象としてキューに追加
        
        # ソート結果をself.transformsに反映（表示と実データを一致させる）
        self.transforms = sorted_transforms
        
        items = [f"{tf.header.frame_id} -> {tf.child_frame_id}" for tf in self.transforms]
        self.transform_selector.addItems(items)
        
        self.transform_selector.blockSignals(False)
        self._on_transform_selected()
        
    def _on_transform_selected(self):
        """ComboBoxでTFが選択されたときに呼ばれる"""
        idx = self.transform_selector.currentIndex()
        if idx < 0 or idx >= len(self.transforms):
            return
        
        tf = self.transforms[idx]
        t = tf.transform.translation
        r = tf.transform.rotation
        
        # 値の更新中はシグナルをブロック
        for spin in [self.trans_x_spin, self.trans_y_spin, self.trans_z_spin, self.roll_spin, self.pitch_spin, self.yaw_spin]:
            spin.blockSignals(True)
            
        self.trans_x_spin.setValue(t.x)
        self.trans_y_spin.setValue(t.y)
        self.trans_z_spin.setValue(t.z)
        
        roll, pitch, yaw = quaternion_to_rpy([r.x, r.y, r.z, r.w])
        self.roll_spin.setValue(roll)
        self.pitch_spin.setValue(pitch)
        self.yaw_spin.setValue(yaw)
        
        for spin in [self.trans_x_spin, self.trans_y_spin, self.trans_z_spin, self.roll_spin, self.pitch_spin, self.yaw_spin]:
            spin.blockSignals(False)

    def _on_transform_edited(self):
        """SpinBoxの値が変更されたときに呼ばれる"""
        idx = self.transform_selector.currentIndex()
        if idx < 0 or idx >= len(self.transforms):
            return

        # UIの値からクォータニオンを再計算
        quat = rpy_to_quaternion(self.roll_spin.value(), self.pitch_spin.value(), self.yaw_spin.value())
        
        # self.transformsの値を更新
        self.transforms[idx].transform.translation.x = self.trans_x_spin.value()
        self.transforms[idx].transform.translation.y = self.trans_y_spin.value()
        self.transforms[idx].transform.translation.z = self.trans_z_spin.value()
        self.transforms[idx].transform.rotation.x = quat[0]
        self.transforms[idx].transform.rotation.y = quat[1]
        self.transforms[idx].transform.rotation.z = quat[2]
        self.transforms[idx].transform.rotation.w = quat[3]

        # 3Dプレビューを更新
        self._update_3d_preview()

    # --- 3Dプレビュー関連 ---
    def _update_3d_preview(self):
        """3Dビューを現在のTFと選択されたPointCloudで更新する"""
        self.plotter.clear()
        self._calculate_global_poses()

        # 2. 各フレームの座標軸を描画
        for frame_id, pose_matrix in self.tf_poses.items():
            axes = pv.Axes(show_actor=False)
            axes.origin = pose_matrix[:3, 3]
            axes.orientation = pose_matrix[:3, :3]
            actor = self.plotter.add_actor(axes.actor, pickable=False)
            self.plotter.add_text(frame_id, position=pose_matrix[:3, 3], font_size=8)

        # 3. PointCloudを描画
        pc_topic = self.pointcloud_selector.currentText()
        if pc_topic != "None" and self.pointclouds.get(pc_topic) is not None:
            pc_cid = self.topic_to_cid.get(pc_topic)
            pc_frame_id = self.meta_info[pc_cid].get('frame_id') if pc_cid else None
            if not pc_frame_id:
                self.plotter.add_text(f"Error: Could not find frame_id for topic\n'{pc_topic}'", position="upper_left", color='red')
            elif pc_frame_id not in self.tf_poses:
                self.plotter.add_text(f"Error: Frame '{pc_frame_id}' not in TF tree!", position="upper_left", color='red')
            else:
                points = self.pointclouds[pc_topic]
                transform_matrix = self.tf_poses[pc_frame_id]
                points_h = np.hstack((points, np.ones((points.shape[0], 1))))
                points_world = (transform_matrix @ points_h.T).T[:, :3]
                
                self.plotter.add_points(points_world, render_points_as_spheres=False, point_size=1)

        self.plotter.reset_camera()

    def _calculate_global_poses(self):
        """TFツリーを辿って、全フレームのワールド座標系での姿勢(4x4行列)を計算"""
        self.tf_poses.clear()
        tf_map = {tf.child_frame_id: tf for tf in self.transforms}
        
        # 再帰的に計算
        for root in self.root_frames:
            self.tf_poses[root] = np.identity(4)
            self._dfs_pose_calculation(root, tf_map)
            
    def _dfs_pose_calculation(self, parent_frame, tf_map):
        if parent_frame in self.tf_graph:
            for child_frame in self.tf_graph[parent_frame]:
                if child_frame in tf_map:
                    tf = tf_map[child_frame]
                    trans = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
                    rot = [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
                    
                    local_matrix = np.identity(4)
                    local_matrix[:3, :3] = Rotation.from_quat(rot).as_matrix()
                    local_matrix[:3, 3] = trans
                    
                    # 親の姿勢と掛け合わせてグローバルな姿勢を計算
                    self.tf_poses[child_frame] = self.tf_poses[parent_frame] @ local_matrix
                    
                    # 再帰呼び出し
                    self._dfs_pose_calculation(child_frame, tf_map)

    # --- データ変換とダイアログ終了処理 ---
    def _dict_to_transform_stamped(self, d):
        """辞書からTransformStampedのような構造体オブジェクトに変換"""
        from types import SimpleNamespace
        # ネストした辞書を再帰的にSimpleNamespaceに変換
        def to_sns(data):
            if isinstance(data, dict):
                return SimpleNamespace(**{k: to_sns(v) for k, v in data.items()})
            return data
        return to_sns(d)

    def accept(self):
        """OKボタン: 編集結果を返す"""
        # self.transforms は既に最新なので、それを返す準備をする
        self.edited_data = self.transforms
        super().accept()
    def _deduplicate_and_validate_transforms(self, incoming_transforms):
        """
        TFのリストを受け取り、重複を排除し、矛盾がないか検証する。
        問題なければクリーンなリストを、問題あればValueErrorを発生させる。
        """
        unique_tfs = {}  # キー: (parent, child), 値: transformオブジェクト
        
        for tf in incoming_transforms:
            parent = tf.header.frame_id
            child = tf.child_frame_id
            key = (parent, child)

            if key not in unique_tfs:
                # 初めて見るTFリンクなので、そのまま登録
                unique_tfs[key] = tf
            else:
                # 重複が見つかった場合、内容が同一か検証する
                existing_tf = unique_tfs[key]
                
                # translationとrotationをnumpy配列に変換して比較
                t_existing = existing_tf.transform.translation
                r_existing = existing_tf.transform.rotation
                vec_existing = np.array([t_existing.x, t_existing.y, t_existing.z, r_existing.x, r_existing.y, r_existing.z, r_existing.w])
                
                t_new = tf.transform.translation
                r_new = tf.transform.rotation
                vec_new = np.array([t_new.x, t_new.y, t_new.z, r_new.x, r_new.y, r_new.z, r_new.w])
                
                # np.allcloseで浮動小数点数を安全に比較
                if not np.allclose(vec_existing, vec_new):
                    # 値が異なる場合は矛盾データとしてエラーを発生させる
                    raise ValueError(
                        f"Conflicting transforms found for link '{parent} -> {child}'.\n\n"
                        "Please ensure all transforms for the same link are identical."
                    )
                # 値が同じであれば、何もしない（重複分を無視する）

        return list(unique_tfs.values())

    def _reset_to_initial_state(self):
        """
        現在の変更をすべて破棄し、ダイアログを開いた時点の初期状態に戻す
        """
        reply = QMessageBox.question(self, 'Confirm Reset',
                                     "Are you sure you want to discard all current changes and reset to the initial state?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            # ワーキングコピーを初期状態で上書き
            self.transforms = list(self.initial_transforms)
            # YAML入力欄もクリア
            self.yaml_text_edit.clear()
            # UI全体を再構築
            self. _rebuild_all()
            QMessageBox.information(self, "Reset Complete", "All transforms have been reset.")
