from pathlib import Path
from typing import Dict, Any, Tuple

# rosbags関連
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer as Rosbag1Writer
from rosbags.rosbag2 import Writer as Rosbag2Writer
from typing import TYPE_CHECKING, cast
import yaml
from fnmatch import fnmatch

from rosbags.interfaces import Connection, ConnectionExtRosbag2
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

typestore = get_typestore(Stores.ROS2_JAZZY)
pkg_share = Path(__file__).parent / 'types' / 'pandar_msgs' / 'msg'

packet_def = (pkg_share / 'PandarPacket.msg').read_text(encoding='utf-8')
typestore.register(
    get_types_from_msg(packet_def, 'pandar_msgs/msg/PandarPacket')
)

scan_def = (pkg_share / 'PandarScan.msg').read_text(encoding='utf-8')
typestore.register(
    get_types_from_msg(scan_def, 'pandar_msgs/msg/PandarScan')
)

class BagModel:
    """
    rosbag(ROS1/ROS2) のメタ情報や実際の読み込み・書き込み処理を担う Model。
    """
    # ✅ 新規: 詳細編集をサポートするメッセージ型のリスト
    SUPPORTED_DETAIL_EDIT_TYPES = [
        "sensor_msgs/msg/CameraInfo",
        "tf2_msgs/msg/TFMessage",  # /tf_static は通常この型
    ]
    
    def __init__(self):
        self.bag_path: Path = None
        self.rosbag_version: str = None
        self.meta_info: Dict[int, Dict[str, Any]] = {}
        self.detail_edit_data: Dict[int, Dict[str, Any]] = {}

    def is_detail_edit_supported(self, msgtype: str) -> bool:
        """
        ✅ 新規: 指定されたメッセージ型が詳細編集をサポートしているか判定する。
        """
        return msgtype in self.SUPPORTED_DETAIL_EDIT_TYPES

    def load_bag_metadata(self, path_obj: Path) -> Tuple[bool, Dict[int, Dict[str, Any]]]:
        """
        指定パスが ROS1(.bag) か ROS2(ディレクトリ内にmetadata.yaml + *.db3) かを判定し、
        connections からメタデータ(トピック,型など)を収集して返す。

        戻り値: (is_ros1, meta_info)
        例外があれば raise する。
        """
        # 1) ROS1 か ROS2 か判定
        if path_obj.is_dir():
            # ROS2 の可能性
            meta_file = path_obj / "metadata.yaml"
            db3_files = list(path_obj.glob("*.db3"))
            if meta_file.exists() and db3_files:
                rosbag_version = "ROS2"
                bag_path = path_obj
                print("ROS2 directory detected.")
            else:
                raise ValueError("Not a valid ROS2 directory.")
        else:
            # ファイル(.bag) → ROS1 だと仮定
            if path_obj.suffix == ".bag":
                rosbag_version = "ROS1"
                bag_path = path_obj
            else:
                raise ValueError("Not a valid .bag file or ROS2 folder.")

        # 2) connections からメタ情報取得
        meta_info = {}
        frame_ids = set()
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            for connection in reader.connections:
                meta_info[connection.id] = {
                    "topic": connection.topic,
                    "msgtype": connection.msgtype
                }
                if rosbag_version == "ROS2":
                    ext = cast('ConnectionExtRosbag2', connection.ext)
                    meta_info[connection.id]["serialization_format"] = ext.serialization_format
                    meta_info[connection.id]["qos"] = ext.offered_qos_profiles[0] # rosbag2は途中でQosが変わるためリストであるが初期Qosをとりあえず使用することにする。
            
            
                try:
                    _conn, _ts, data = next(reader.messages(connections=[connection]))
                except StopIteration:
                    # まれにメッセージが 0 件の connection があるため
                    continue

                msg = reader.typestore.deserialize_cdr(data, connection.msgtype)

                if hasattr(msg, "header"):
                    # header があれば frame_id を追加
                    meta_info[connection.id]["frame_id"] = msg.header.frame_id
            
        return rosbag_version, meta_info

    def save_bag(
        self,
        in_path: Path,
        out_path: Path,
        out_format: str,
        updated_meta: Dict[int, Dict[str, Any]]
    ) -> None:
        """
        既存の bag (in_path) を読み込み、トピック/型情報を updated_meta に基づいて上書きしながら、
        指定の形式(ROS1 / ROS2)で out_path に保存する。
        """
        with AnyReader([in_path], default_typestore=typestore) as reader:
            if out_format == "ROS1":
                with Rosbag1Writer(out_path) as writer:
                    self._write_bag1(reader, writer, updated_meta)
            else:
                with Rosbag2Writer(out_path) as writer:
                    self._write_bag2(reader, writer, updated_meta)

    def _process_and_write_messages(self, reader: AnyReader, writer, meta_info: Dict[int, Dict[str, Any]], conn_map: Dict[int, Connection]):
        """
        メッセージを走査し、必要ならframe_idを書き換え、書き込む共通ヘルパー関数。
        """
        # 保存対象のconnectionのみを効率的に読み込む
        target_connections = [c for c in reader.connections if c.id in conn_map]

        for conn, ts, data in reader.messages(connections=target_connections):
            cid = conn.id
            wconn = conn_map[cid]
            msg_modified = False

            try:
                # メッセージの書き換えが必要な場合のみデシリアライズ
                needs_deserialize = ("frame_id" in meta_info[cid]) or (cid in self.detail_edit_data)

                if needs_deserialize:
                    msg = reader.typestore.deserialize_cdr(data, conn.msgtype)

                    # 1. frame_idの書き換え
                    if "frame_id" in meta_info[cid] and hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
                        new_frame_id = meta_info[cid]["frame_id"]
                        if msg.header.frame_id != new_frame_id:
                            msg.header.frame_id = new_frame_id
                            msg_modified = True
                    
                    # ✅ 2. 詳細編集データの適用 (将来の拡張ポイント)
                    if cid in self.detail_edit_data:
                        edit_info = self.detail_edit_data[cid]
                        msgtype = conn.msgtype

                        # --- ここにメッセージ型ごとの編集ロジックを実装 ---
                        if msgtype == "sensor_msgs/msg/CameraInfo" and edit_info['type'] == 'camera_info':
                            # 例: msg.k = edit_info['data']['k'] ...
                            print(f"Applying detailed edits for CameraInfo on topic {conn.topic}")
                            msg_modified = True
                        
                        elif msgtype == "tf2_msgs/msg/TFMessage" and edit_info['type'] == 'tf_static':
                            # 例: msg.transforms を削除、変更、追加する ...
                            print(f"Applying detailed edits for TFMessage on topic {conn.topic}")
                            msg_modified = True
                        # ----------------------------------------------------

                    # メッセージが変更されていたら再シリアライズ
                    if msg_modified:
                        data = reader.typestore.serialize_cdr(msg, conn.msgtype)

            except Exception as e:
                print(f"Warning: Could not process message for topic {conn.topic}. Error: {e}")
            
            writer.write(wconn, ts, data)

    def _write_bag1(self, reader: AnyReader, writer: Rosbag1Writer, meta_info: Dict[int, Dict[str, Any]]):
        """
        ROS1形式のバッグに接続情報とメッセージを書き込む。
        """
        conn_map = {}
        filtered_patterns = ["rcl_interfaces/*", "rosbag2_interfaces/*"]
        connections_by_id = {c.id: c for c in reader.connections}
        # 1) ROS1用の接続情報を追加
        for cid, info in meta_info.items():
            connection = connections_by_id[cid]
            if any(fnmatch(connection.msgtype, pat) for pat in filtered_patterns):
                continue
            
            conn_map[cid] = writer.add_connection(
                topic=info["topic"],
                msgtype=info["msgtype"],
                typestore=reader.typestore
            )
        
        # 2) メッセージの処理と書き込みを共通ヘルパーに委譲
        self._process_and_write_messages(reader, writer, meta_info, conn_map)

    def _write_bag2(self, reader: AnyReader, writer: Rosbag2Writer, meta_info: Dict[int, Dict[str, Any]]):
        """
        ROS2形式のバッグに接続情報とメッセージを書き込む。
        """
        conn_map = {}
        filtered_patterns = ["rcl_interfaces/*", "rosbag2_interfaces/*"]
        connections_by_id = {c.id: c for c in reader.connections}
        
        # 1) ROS2用の接続情報を追加
        for cid, info in meta_info.items():
            connection = connections_by_id[cid]
            if any(fnmatch(connection.msgtype, pat) for pat in filtered_patterns):
                continue
            
            qos = None
            ext = cast('ConnectionExtRosbag2', connection.ext)
            qos = info.get("qos", ext.offered_qos_profiles[0] if ext.offered_qos_profiles else None)
            conn_map[cid] = writer.add_connection(
                topic=info["topic"],
                msgtype=info["msgtype"],
                typestore=reader.typestore,
                serialization_format=ext.serialization_format,
                offered_qos_profiles=[qos] if qos else []
            )

        # 2) メッセージの処理と書き込みを共通ヘルパーに委譲
        self._process_and_write_messages(reader, writer, meta_info, conn_map)