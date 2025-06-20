
from collections import defaultdict, deque

import numpy as np

# PyVista & Qt 連携
import pyvista as pv
import yaml
from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QStackedWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)
from pyvistaqt import QtInteractor
from scipy.spatial.transform import Rotation


# ─────────────────────────────────────────────
# ヘルパー
# ─────────────────────────────────────────────
def rpy_to_quaternion(roll, pitch, yaw, degrees=True):
    """Roll, Pitch, Yaw → quaternion [x, y, z, w]"""
    return Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=degrees).as_quat()


def quaternion_to_rpy(quat, degrees=True):
    """quaternion [x, y, z, w] → Roll, Pitch, Yaw"""
    if not np.any(quat):
        return 0.0, 0.0, 0.0
    e = Rotation.from_quat(quat).as_euler("xyz", degrees=degrees)
    return float(e[0]), float(e[1]), float(e[2])


# ─────────────────────────────────────────────
# メインクラス
# ─────────────────────────────────────────────
class TFStaticEditorDialog(QDialog):
    """TF_Static 編集 + 3D プレビュー"""

    # ==============================================================
    # コンストラクタ
    # ==============================================================
    def __init__(self, tf_message, thumbnail_pointclouds, meta_info, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit TF Static")
        self.setMinimumSize(1600, 900)

        # ---------- データ ----------
        self.pointclouds = thumbnail_pointclouds
        self.meta_info = meta_info
        self.topic_to_cid = {info["topic"]: cid for cid, info in self.meta_info.items()}

        self.tf_graph = defaultdict(list)   # {parent: [child]}
        self.tf_poses = {}                  # {frame: 4x4}

        # 初期 TF
        init_tfs = tf_message.transforms if tf_message else []
        try:
            self.initial_transforms = self._deduplicate_and_validate_transforms(init_tfs)
            self.transforms = list(self.initial_transforms)
        except ValueError as e:
            QMessageBox.critical(self, "Initial TF Data Error", str(e))
            self.initial_transforms = []
            self.transforms = []

        # ---------- UI ----------
        self._build_ui()

        # ---------- 3D 状態 ----------
        self._axis_actors: list[pv.Actor] = []
        self._grid_actor: pv.Actor | None = None
        self._prev_view_mag: float | None = None
        self._prev_axis_len: float | None = None
        self._prev_grid_spacing: float | None = None

        # 連続イベント束ねタイマー
        self._refresh_timer = QtCore.QTimer(self)
        self._refresh_timer.setInterval(30)       # ms
        self._refresh_timer.setSingleShot(True)
        self._refresh_timer.timeout.connect(self._refresh_axes_grid)

        # 初期ビルド
        self._rebuild_all()

    # ==============================================================
    # UI セットアップ
    # ==============================================================
    def _build_ui(self):
        main = QHBoxLayout(self)
        self.setLayout(main)

        # ─── 左パネル ───
        left = QVBoxLayout()

        # YAML
        g_yaml = QGroupBox("Load TF from Pasted YAML")
        v_yaml = QVBoxLayout()
        self.yaml_edit = QPlainTextEdit(placeholderText="Paste 'rostopic echo /tf_static' output …")
        btn_parse = QPushButton("Parse Text & Rebuild Tree")
        btn_parse.clicked.connect(self._parse_yaml)
        v_yaml.addWidget(self.yaml_edit)
        v_yaml.addWidget(btn_parse)
        g_yaml.setLayout(v_yaml)
        left.addWidget(g_yaml)

        # TF Tree
        g_tree = QGroupBox("TF Tree")
        v_tree = QVBoxLayout()
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Frame ID"])
        v_tree.addWidget(self.tree)
        g_tree.setLayout(v_tree)
        left.addWidget(g_tree, 1)

        # Transform Editor
        g_edit = QGroupBox("Transform Editor")
        grid = QGridLayout()
        self.sel_tf = QComboBox()
        self.sel_tf.currentIndexChanged.connect(self._on_tf_selected)
        # spin
        self.txs = self._spin(-100, 100); self.tys = self._spin(-100, 100); self.tzs = self._spin(-100, 100)
        self.rs = self._spin(-180, 180); self.ps = self._spin(-90, 90); self.ys = self._spin(-180, 180)
        # layout
        grid.addWidget(QLabel("Target Frame:"), 0, 0, 1, 2)
        grid.addWidget(self.sel_tf, 0, 2, 1, 4)
        grid.addWidget(QLabel("Translation (m)"), 1, 0, 1, 2)
        grid.addWidget(QLabel("X:"), 2, 0); grid.addWidget(self.txs, 2, 1)
        grid.addWidget(QLabel("Y:"), 3, 0); grid.addWidget(self.tys, 3, 1)
        grid.addWidget(QLabel("Z:"), 4, 0); grid.addWidget(self.tzs, 4, 1)
        
        self.rot_mode_combo = QComboBox()
        self.rot_mode_combo.addItems(["Euler (deg)", "Euler (rad)", "Quaternion"])
        self.rot_mode_combo.currentIndexChanged.connect(self._on_rot_mode_changed)
        self.rot_stack = QStackedWidget()
        self._build_rotation_editors()
        grid.addWidget(QLabel("Rotation"), 1, 3, 1, 1)
        grid.addWidget(self.rot_mode_combo, 1, 4, 1, 2)
        grid.addWidget(self.rot_stack, 2, 3, 3, 3)
        g_edit.setLayout(grid)
        left.addWidget(g_edit)

        # OK / Cancel / Reset
        box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btn_reset = box.addButton("Reset to Initial State", QDialogButtonBox.ResetRole)
        box.accepted.connect(self.accept)
        box.rejected.connect(self.reject)
        btn_reset.clicked.connect(self._reset_to_initial_state)
        left.addWidget(box)

        # ─── 右パネル ───
        right = QVBoxLayout()
        g_prev = QGroupBox("3D Preview")
        v_prev = QVBoxLayout()

        # controls
        ctrl = QGridLayout()
        ctrl.addWidget(QLabel("Fixed Frame:"), 0, 0)
        self.sel_fixed = QComboBox(); self.sel_fixed.currentIndexChanged.connect(self._update_3d_preview)
        ctrl.addWidget(self.sel_fixed, 0, 1)
        ctrl.addWidget(QLabel("Display PointCloud:"), 0, 2)
        self.sel_pc = QComboBox(); self.sel_pc.addItems(["None"]); self.sel_pc.currentIndexChanged.connect(self._update_3d_preview)
        ctrl.addWidget(self.sel_pc, 0, 3)
        ctrl.addWidget(QLabel("Color by:"), 1, 0)
        self.sel_color = QComboBox(); self.sel_color.addItems(["Single Color", "Height", "Intensity"])
        self.sel_color.currentIndexChanged.connect(self._update_3d_preview)
        ctrl.addWidget(self.sel_color, 1, 1)
        v_prev.addLayout(ctrl)

        # plotter
        self.plot = QtInteractor(self); self.plot.set_background("black")
        v_prev.addWidget(self.plot.interactor)
        g_prev.setLayout(v_prev)
        right.addWidget(g_prev)

        # main layout
        main.addLayout(left, 1); main.addLayout(right, 2)

        # spin signals
        for s in (self.txs, self.tys, self.tzs, self.rs, self.ps, self.ys):
            s.valueChanged.connect(self._on_spin_changed)

        # camera / wheel events
        for ev in ("EndInteractionEvent", "MouseWheelForwardEvent", "MouseWheelBackwardEvent"):
            self._safe_add_observer(ev, self._schedule_refresh)

    # --------------------------------------------------------------
    @staticmethod
    def _spin(minv, maxv):
        sb = QDoubleSpinBox(); sb.setRange(minv, maxv); sb.setDecimals(4); sb.setSingleStep(0.01)
        return sb

    def _build_rotation_editors(self):
        # Page 0: Euler (deg)
        page_deg = QWidget()
        layout_deg = QGridLayout(page_deg)
        self.rs = self._spin(-180, 180); self.ps = self._spin(-90, 90); self.ys = self._spin(-180, 180)
        layout_deg.addWidget(QLabel("Roll:"), 0, 0); layout_deg.addWidget(self.rs, 0, 1)
        layout_deg.addWidget(QLabel("Pitch:"), 1, 0); layout_deg.addWidget(self.ps, 1, 1)
        layout_deg.addWidget(QLabel("Yaw:"), 2, 0); layout_deg.addWidget(self.ys, 2, 1)
        self.rot_stack.addWidget(page_deg)

        # Page 1: Euler (rad)
        page_rad = QWidget()
        layout_rad = QGridLayout(page_rad)
        self.rad_r = self._spin(-np.pi, np.pi); self.rad_p = self._spin(-np.pi/2, np.pi/2); self.rad_y = self._spin(-np.pi, np.pi)
        layout_rad.addWidget(QLabel("R(rad):"), 0, 0); layout_rad.addWidget(self.rad_r, 0, 1)
        layout_rad.addWidget(QLabel("P(rad):"), 1, 0); layout_rad.addWidget(self.rad_p, 1, 1)
        layout_rad.addWidget(QLabel("Y(rad):"), 2, 0); layout_rad.addWidget(self.rad_y, 2, 1)
        self.rot_stack.addWidget(page_rad)

        # Page 2: Quaternion
        page_quat = QWidget()
        layout_quat = QGridLayout(page_quat)
        self.qx = self._spin(-1, 1); self.qy = self._spin(-1, 1)
        self.qz = self._spin(-1, 1); self.qw = self._spin(-1, 1)
        layout_quat.addWidget(QLabel("X:"), 0, 0); layout_quat.addWidget(self.qx, 0, 1)
        layout_quat.addWidget(QLabel("Y:"), 1, 0); layout_quat.addWidget(self.qy, 1, 1)
        layout_quat.addWidget(QLabel("Z:"), 2, 0); layout_quat.addWidget(self.qz, 2, 1)
        layout_quat.addWidget(QLabel("W:"), 3, 0); layout_quat.addWidget(self.qw, 3, 1)
        self.rot_stack.addWidget(page_quat)
    # ==============================================================
    # Rebuild
    # ==============================================================
    def _rebuild_all(self):
        self._build_graph()
        self._populate_tf_selector()
        self._populate_tree()
        self._calc_global_poses()
        self._update_fixed_selector()
        self._update_pc_selector()
        self._update_3d_preview()

    # --------------------------------------------------------------
    # YAML parse
    # --------------------------------------------------------------
    def _parse_yaml(self):
        text = self.yaml_edit.toPlainText()
        if not text:
            return
        try:
            docs = yaml.safe_load_all(text)
            parsed = [self._dict_to_tf(d) for doc in docs if doc for d in doc.get("transforms", [])]
            self.transforms = self._deduplicate_and_validate_transforms(parsed)
            self._rebuild_all()
            QMessageBox.information(self, "Success", f"Loaded {len(self.transforms)} transforms.")
        except Exception as e:
            QMessageBox.critical(self, "YAML Error", str(e))

    # --------------------------------------------------------------
    # TF graph
    # --------------------------------------------------------------
    def _build_graph(self):
        self.tf_graph.clear()
        frames = set()
        for tf in self.transforms:
            self.tf_graph[tf.header.frame_id].append(tf.child_frame_id)
            frames.update((tf.header.frame_id, tf.child_frame_id))
        children = {c for lst in self.tf_graph.values() for c in lst}
        self.roots = [f for f in frames if f not in children]

    def _populate_tree(self):
        self.tree.clear()
        for r in self.roots:
            root_item = QTreeWidgetItem(self.tree, [r])
            self._add_children(root_item, r)
        self.tree.expandAll()

    def _add_children(self, parent_item, parent_frame):
        for ch in sorted(self.tf_graph.get(parent_frame, [])):
            child_item = QTreeWidgetItem(parent_item, [ch])
            self._add_children(child_item, ch)

    # --------------------------------------------------------------
    # TF selector
    # --------------------------------------------------------------
    def _populate_tf_selector(self):
        self.sel_tf.blockSignals(True)
        self.sel_tf.clear()
        q = deque(sorted(self.roots))
        order = []; tfmap = {tf.child_frame_id: tf for tf in self.transforms}
        seen = set()
        while q:
            p = q.popleft(); seen.add(p)
            for ch in sorted(self.tf_graph.get(p, [])):
                if ch in tfmap:
                    order.append(tfmap[ch]); q.append(ch)
        self.transforms = order
        self.sel_tf.addItems([f"{tf.header.frame_id}->{tf.child_frame_id}" for tf in self.transforms])
        self.sel_tf.blockSignals(False)
        self._on_tf_selected()

    def _on_rot_mode_changed(self, index):
        self.rot_stack.setCurrentIndex(index)
        self._on_tf_selected() # モード切替時に現在のTF値を新しいUIに反映

    def _on_tf_selected(self):
        i = self.sel_tf.currentIndex()
        if i < 0 or i >= len(self.transforms): return
        tf = self.transforms[i]
        t = tf.transform.translation
        r = tf.transform.rotation
        quat = [r.x, r.y, r.z, r.w]

        # --- Translation ---
        all_spins = (self.txs, self.tys, self.tzs, self.rs, self.ps, self.ys, 
                     self.rad_r, self.rad_p, self.rad_y,
                     self.qx, self.qy, self.qz, self.qw)
        for s in all_spins: s.blockSignals(True)

        self.txs.setValue(t.x)
        self.tys.setValue(t.y)
        self.tzs.setValue(t.z)

        # --- Rotation (by mode) ---
        mode = self.rot_mode_combo.currentIndex()
        if mode == 0: # Euler (deg)
            roll, pitch, yaw = quaternion_to_rpy(quat, degrees=True)
            self.rs.setValue(roll)
            self.ps.setValue(pitch)
            self.ys.setValue(yaw)
        elif mode == 1: # Euler (rad)
            roll, pitch, yaw = quaternion_to_rpy(quat, degrees=False)
            self.rad_r.setValue(roll)
            self.rad_p.setValue(pitch)
            self.rad_y.setValue(yaw)
        elif mode == 2: # Quaternion
            self.qx.setValue(r.x)
            self.qy.setValue(r.y)
            self.qz.setValue(r.z)
            self.qw.setValue(r.w)
            
        for s in all_spins: s.blockSignals(False)

    # --------------------------------------------------------------
    # spin → TF
    # --------------------------------------------------------------
    def _on_spin_changed(self):
        i = self.sel_tf.currentIndex()
        if i < 0 or i >= len(self.transforms): return
        tf = self.transforms[i]

        # --- Translation ---
        tf.transform.translation.x = self.txs.value()
        tf.transform.translation.y = self.tys.value()
        tf.transform.translation.z = self.tzs.value()
        
        # --- Rotation (by mode) ---
        mode = self.rot_mode_combo.currentIndex()
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        try:
            if mode == 0: # Euler (deg)
                quat = rpy_to_quaternion(self.rs.value(), self.ps.value(), self.ys.value(), degrees=True)
            elif mode == 1: # Euler (rad)
                quat = rpy_to_quaternion(self.rad_r.value(), self.rad_p.value(), self.rad_y.value(), degrees=False)
            elif mode == 2: # Quaternion
                q_in = np.array([self.qx.value(), self.qy.value(), self.qz.value(), self.qw.value()])
                norm = np.linalg.norm(q_in)
                if norm > 1e-6: # ゼロベクトルでなければ正規化
                    quat = q_in / norm
        except Exception: # scipyが不正な値でエラーを出す場合がある
             pass # 不正な中間値は無視

        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = quat

        # --- Update 3D View (NOT rebuild all) ---
        # これがバグ修正の核心部分。UI全体を再構築せず、データと3Dビューのみ更新
        self._calc_global_poses()
        self._update_3d_preview()

    # --------------------------------------------------------------
    # fixed / pc selector
    # --------------------------------------------------------------
    def _update_fixed_selector(self):
        self.sel_fixed.blockSignals(True)
        cur = self.sel_fixed.currentText()
        self.sel_fixed.clear(); self.sel_fixed.addItems(sorted(self.tf_poses.keys()))
        if cur in self.tf_poses: self.sel_fixed.setCurrentText(cur)
        elif self.roots: self.sel_fixed.setCurrentText(sorted(self.roots)[0])
        self.sel_fixed.blockSignals(False)

    def _update_pc_selector(self):
        self.sel_pc.blockSignals(True)
        cur = self.sel_pc.currentText()
        self.sel_pc.clear(); self.sel_pc.addItems(["None"] + sorted(self.pointclouds.keys()))
        if cur in self.pointclouds: self.sel_pc.setCurrentText(cur)
        self.sel_pc.blockSignals(False)

    # --------------------------------------------------------------
    # global poses
    # --------------------------------------------------------------
    def _calc_global_poses(self):
        self.tf_poses.clear()
        tfmap = {tf.child_frame_id: tf for tf in self.transforms}
        for r in self.roots:
            self.tf_poses[r] = np.identity(4); self._dfs(r, tfmap)

    def _dfs(self, parent, tfmap):
        for ch in self.tf_graph.get(parent, []):
            if ch not in tfmap: continue
            tf = tfmap[ch]; t = tf.transform.translation; r = tf.transform.rotation
            local = np.identity(4)
            local[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            local[:3, 3] = [t.x, t.y, t.z]
            self.tf_poses[ch] = self.tf_poses[parent] @ local
            self._dfs(ch, tfmap)

    # ==============================================================
    # 3D Preview
    # ==============================================================
    def _update_3d_preview(self):
        self.plot.clear()
        self._axis_actors = []        # ← 矢印はここで描かない
        self._grid_actor = None

        fixed = self.sel_fixed.currentText()
        if fixed not in self.tf_poses:
            self.plot.add_text("No valid TF / fixed frame.", position="upper_left", color="yellow")
            self.plot.reset_camera(); return

        inv = np.linalg.inv(self.tf_poses[fixed])
        poses = {fid: inv @ P for fid, P in self.tf_poses.items()}
        positions = np.array([P[:3, 3] for P in poses.values()])

        # labels
        if positions.size:
            self.plot.add_point_labels(list(positions), list(poses.keys()),
                                       font_size=16, always_visible=True,
                                       shape_opacity=0.0, text_color="white")

        # TF edges
        for tf in self.transforms:
            p, c = tf.header.frame_id, tf.child_frame_id
            if p in poses and c in poses:
                a = poses[p][:3, 3]; b = poses[c][:3, 3]
                self.plot.add_lines(np.vstack([a, b]), color="yellow", width=2)

        # pointcloud
        self._draw_pc(poses)

        # camera
        self.plot.enable_parallel_projection(); self.plot.reset_camera()
        self.plot.camera.focal_point = (0, 0, 0)

        # cache & refresh
        self._transformed_poses = poses
        self._schedule_refresh(force=True)

    # --------------------------------------------------------------
    def _draw_pc(self, poses):
        topic = self.sel_pc.currentText()
        if topic == "None" or topic not in self.pointclouds: return
        pts = self.pointclouds[topic]
        cid = self.topic_to_cid.get(topic); pc_frame = self.meta_info[cid].get("frame_id") if cid else None
        if pc_frame not in poses:
            self.plot.add_text(f"Frame '{pc_frame}' not in TF", position="upper_left", color="red"); return
        hpts = np.c_[pts[:, :3], np.ones(len(pts))]
        world = (poses[pc_frame] @ hpts.T).T[:, :3]
        mode = self.sel_color.currentText()
        kw = {"point_size": 2, "render_points_as_spheres": False}
        if mode == "Height":
            kw |= {"scalars": world[:, 2], "cmap": "jet", "scalar_bar_args": {"title": "Z", "color": "white"}}
        elif mode == "Intensity" and pts.shape[1] > 3:
            kw |= {"scalars": pts[:, 3], "cmap": "viridis", "scalar_bar_args": {"title": "I", "color": "white"}}
        else:
            kw["color"] = "white"
        self.plot.add_points(world, **kw)

    # ==============================================================
    # 軸・グリッド更新（改良＆バグフィックス済み）
    # ==============================================================
    def _schedule_refresh(self, caller=None, event=None, force=False):
        if force: self._refresh_axes_grid(); return
        if not self._refresh_timer.isActive(): self._refresh_timer.start()

    def _refresh_axes_grid(self):
        if not hasattr(self, "_transformed_poses"): return

        # 1) view scale
        cam = self.plot.camera
        view_mag = cam.GetParallelScale() if cam.GetParallelProjection() else np.linalg.norm(np.array(cam.position) - np.array(cam.focal_point))

        # 2) skip small change
        if self._prev_view_mag is not None and abs(view_mag - self._prev_view_mag) < 0.05 * self._prev_view_mag:
            return
        self._prev_view_mag = view_mag

        # 3) axis length & grid spacing
        axis_len = np.clip(view_mag * 0.07, 0.05, 10.0)

        def pick_spacing(mag, target=20):
            exp = int(np.floor(np.log10(mag)))
            for b in (1, 2, 5, 10):
                s = b * 10 ** exp
                if mag / s <= target: return s
            return 10 ** (exp + 1)

        grid_spacing = pick_spacing(view_mag)
        grid_extent = grid_spacing * np.ceil((view_mag * 1.2) / grid_spacing)

        # 4) skip if axis & grid basically same
        if (self._prev_axis_len is not None and abs(axis_len - self._prev_axis_len) < 0.05 * self._prev_axis_len
            and self._prev_grid_spacing == grid_spacing):
            return
        self._prev_axis_len = axis_len; self._prev_grid_spacing = grid_spacing

        # 5) clear old
        for a in self._axis_actors: self.plot.remove_actor(a, render=False)
        self._axis_actors = []
        if self._grid_actor: self.plot.remove_actor(self._grid_actor, render=False); self._grid_actor = None

        # 6) draw axes
        for P in self._transformed_poses.values():
            p = P[:3, 3]; x, y, z = P[:3, 0], P[:3, 1], P[:3, 2]
            self._axis_actors += [
                self.plot.add_arrows(p, x, mag=axis_len, color="red",   render=False),
                self.plot.add_arrows(p, y, mag=axis_len, color="green", render=False),
                self.plot.add_arrows(p, z, mag=axis_len, color="blue",  render=False),
            ]

        # 7) draw grid
        div = int((grid_extent * 2) / grid_spacing)
        plane = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1), i_size=grid_extent * 2,
                         j_size=grid_extent * 2, i_resolution=div, j_resolution=div)
        self._grid_actor = self.plot.add_mesh(
            plane, color="grey", style="wireframe", line_width=1, opacity=0.4, render=False
        )

        # 8) render once
        self.plot.render()

    # ==============================================================
    # reset / util
    # ==============================================================
    def _reset_to_initial_state(self):
        if QMessageBox.question(self, "Confirm Reset", "Reset all changes?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No) == QMessageBox.Yes:
            self.transforms = list(self.initial_transforms); self.yaml_edit.clear(); self._rebuild_all()

    def _safe_add_observer(self, ev, cb):
        if hasattr(self.plot, "add_observer"): self.plot.add_observer(ev, cb); return
        iren = getattr(self.plot, "iren", None) or getattr(self.plot, "interactor", None)
        if not iren: return
        if hasattr(iren, "add_observer"): iren.add_observer(ev, cb)
        elif hasattr(iren, "AddObserver"): iren.AddObserver(ev, cb)

    # ==============================================================
    # helper for YAML TF dict→SimpleNamespace
    # ==============================================================
    def _dict_to_tf(self, d):
        from types import SimpleNamespace
        def _nest(o): return SimpleNamespace(**{k: _nest(v) for k, v in o.items()}) if isinstance(o, dict) else o
        return _nest(d)

    # ==============================================================
    # validate
    # ==============================================================
    def _deduplicate_and_validate_transforms(self, incoming):
        uniq = {}
        for tf in incoming:
            k = (tf.header.frame_id, tf.child_frame_id)
            if k not in uniq: uniq[k] = tf; continue
            a, b = uniq[k], tf
            v1 = np.array([a.transform.translation.x, a.transform.translation.y, a.transform.translation.z,
                           a.transform.rotation.x, a.transform.rotation.y, a.transform.rotation.z, a.transform.rotation.w])
            v2 = np.array([b.transform.translation.x, b.transform.translation.y, b.transform.translation.z,
                           b.transform.rotation.x, b.transform.rotation.y, b.transform.rotation.z, b.transform.rotation.w])
            if not np.allclose(v1, v2):
                raise ValueError(f"Conflicting transforms for {k[0]}->{k[1]}")
        return list(uniq.values())

    # ==============================================================
    def accept(self):
        self.edited_data = self.transforms
        super().accept()
