from fnmatch import fnmatch
from pathlib import Path
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, Dict, Tuple, cast

import numpy as np
import yaml

# rosbags関連
from rosbags.highlevel import AnyReader
from rosbags.interfaces import Connection, ConnectionExtRosbag2
from rosbags.rosbag1 import Writer as Rosbag1Writer
from rosbags.rosbag2 import Writer as Rosbag2Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from rosbags.typesys.types import builtin_interfaces__msg__Time as Time
from rosbags.typesys.types import geometry_msgs__msg__Quaternion as Quaternion
from rosbags.typesys.types import geometry_msgs__msg__TransformStamped as TransformStamped
from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
from rosbags.typesys.types import std_msgs__msg__Header as Header
from rosbags.typesys.types import tf2_msgs__msg__TFMessage as TFMessage

from app.utils.thumbnail_converters import IMAGE_THUMBNAIL_CONVERTERS, POINTCLOUD_CONVERTERS

typestore = get_typestore(Stores.ROS2_JAZZY)
pkg_share = Path(__file__).parent / "types" / "pandar_msgs" / "msg"

packet_def = (pkg_share / "PandarPacket.msg").read_text(encoding="utf-8")
typestore.register(get_types_from_msg(packet_def, "pandar_msgs/msg/PandarPacket"))

scan_def = (pkg_share / "PandarScan.msg").read_text(encoding="utf-8")
typestore.register(get_types_from_msg(scan_def, "pandar_msgs/msg/PandarScan"))


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
        self.thumbnail_images: Dict[str, np.ndarray] = {}
        self.thumbnail_pointclouds: Dict[str, np.ndarray] = {}

    def get_first_message(self, connection_id: int):
        """
        ✅ 新規: 指定したconnectionの最初のメッセージを取得してデシリアライズする
        """
        if not self.bag_path:
            return None

        with AnyReader([self.bag_path], default_typestore=typestore) as reader:
            target_connection = next((c for c in reader.connections if c.id == connection_id), None)

            if not target_connection:
                print(f"Error: Connection with id {connection_id} not found.")
                return None

            try:
                # 最初のメッセージだけを読み込む
                conn, ts, data = next(reader.messages(connections=[target_connection]))
                msg = reader.typestore.deserialize_cdr(data, conn.msgtype)
                return msg
            except StopIteration:
                # メッセージが空のトピック
                return None
            except Exception as e:
                print(f"Error getting first message for cid {connection_id}: {e}")
                return None

    def get_all_tf_static_transforms(self, connection_id: int):
        """
        ✅ 修正: 指定したconnectionの全メッセージを読み込むが、
        すでに追加済みのTFが現れた時点で処理を打ち切るように最適化。
        """
        if not self.bag_path:
            return None

        all_transforms = []
        # ✅ 追加済みTFリンクを記録するset (parent, child)
        seen_tf_links = set()
        # ✅ 外側のループを抜けるためのフラグ
        stop_processing = False

        with AnyReader([self.bag_path], default_typestore=typestore) as reader:
            target_connection = next((c for c in reader.connections if c.id == connection_id), None)
            if not target_connection:
                return None

            for conn, ts, data in reader.messages(connections=[target_connection]):
                try:
                    msg = reader.typestore.deserialize_cdr(data, conn.msgtype)
                    if not hasattr(msg, "transforms"):
                        continue

                    for tf in msg.transforms:
                        key = (tf.header.frame_id, tf.child_frame_id)

                        # ✅ このTFリンクがすでに追加済みかチェック
                        if key in seen_tf_links:
                            # 見つかった場合、ユニークなTFは全て読み終わったと判断
                            stop_processing = True
                            break  # 内側のループを抜ける

                        # 新しいTFリンクなので追加
                        seen_tf_links.add(key)
                        all_transforms.append(tf)

                except Exception as e:
                    print(f"Warning: Could not process a TF message. Error: {e}")

                # ✅ フラグが立っていたら外側のループも抜ける
                if stop_processing:
                    break

        composite_tf_message = SimpleNamespace(transforms=all_transforms)
        return composite_tf_message

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
        # ここ本当はfile pickerでパスを取得した直後にうけとるべき。
        if path_obj.is_dir():
            if (path_obj / "metadata.yaml").exists() and (
                list(path_obj.glob("*.db3")) or list(path_obj.glob("*.mcap"))
            ):
                rosbag_version = "ROS2"
                bag_path = path_obj
            else:
                raise ValueError("Not a valid ROS2 directory.")

        elif path_obj.is_file():
            if path_obj.suffix == ".bag":
                rosbag_version = "ROS1"
                bag_path = path_obj
            elif path_obj.suffix == ".mcap" or path_obj.suffix == ".db3":
                rosbag_version = "ROS2"
                bag_path = path_obj.parent
            else:
                raise ValueError("Unsupported bag file type.")
        else:
            raise ValueError("Path is neither a file nor a directory.")

        # 2) connections からメタ情報取得
        meta_info = {}
        self.thumbnail_images.clear()
        self.thumbnail_pointclouds.clear()  # ✅ クリア処理を追加
        sampled_image_topics = set()
        sampled_pc_topics = set()
        
        print(f"Loading bag metadata from {bag_path} as {rosbag_version}...")

        with AnyReader([bag_path], default_typestore=typestore) as reader:
            for connection in reader.connections:
                topic_name = connection.topic
                msgtype = connection.msgtype
                meta_info[connection.id] = {
                    "topic": topic_name,
                    "msgtype": msgtype,
                }
                if rosbag_version == "ROS2":
                    ext = cast("ConnectionExtRosbag2", connection.ext)

                    qos_profiles = getattr(ext, "offered_qos_profiles", [])
                    serialization_format = getattr(ext, "serialization_format", "cdr")
                    meta_info[connection.id]["serialization_format"] = serialization_format
                    meta_info[connection.id]["qos"] = qos_profiles[0] if qos_profiles else None # rosbag2は途中でQosが変わるためリストであるが初期Qosをとりあえず使用することにする。


                try:
                    _conn, _ts, data = next(reader.messages(connections=[connection]))
                except StopIteration:
                    # まれにメッセージが 0 件の connection があるため
                    continue

                msg = reader.typestore.deserialize_cdr(data, connection.msgtype)
                if hasattr(msg, "header"):
                    # header があれば frame_id を追加
                    meta_info[connection.id]["frame_id"] = msg.header.frame_id

                if msgtype in IMAGE_THUMBNAIL_CONVERTERS and topic_name not in sampled_image_topics:
                    converter_func = IMAGE_THUMBNAIL_CONVERTERS[msgtype]
                    image_np = converter_func(msg)
                    if image_np is not None:
                        self.thumbnail_images[topic_name] = image_np
                        sampled_image_topics.add(topic_name)

                if msgtype in POINTCLOUD_CONVERTERS and topic_name not in sampled_pc_topics:
                    converter_func = POINTCLOUD_CONVERTERS[msgtype]
                    pc_np = converter_func(msg)
                    if pc_np is not None:
                        self.thumbnail_pointclouds[topic_name] = pc_np
                        sampled_pc_topics.add(topic_name)

        self.bag_path = bag_path # 参照透過性が低なるので本当は良くない

        return rosbag_version, meta_info

    def save_bag(self, in_path: Path, out_path: Path, out_format: str, updated_meta: Dict[int, Dict[str, Any]]) -> None:
        """
        既存の bag (in_path) を読み込み、トピック/型情報を updated_meta に基づいて上書きしながら、
        指定の形式(ROS1 / ROS2)で out_path に保存する。
        """
        with AnyReader([in_path], default_typestore=typestore) as reader:
            if out_format == "ROS1":
                with Rosbag1Writer(out_path) as writer:
                    self._write_bag1(reader, writer, updated_meta)

            else:  # ROS2 系
                # ★ .mcap のときは storage_id='mcap' を明示
                if out_path.suffix == ".mcap":
                    with Rosbag2Writer(out_path, storage_id="mcap") as writer:
                        self._write_bag2(reader, writer, updated_meta)
                else:  # 既定は sqlite3(.db3)
                    with Rosbag2Writer(out_path) as writer:
                        self._write_bag2(reader, writer, updated_meta)

    def _process_and_write_messages(
        self, reader: AnyReader, writer, meta_info: Dict[int, Dict[str, Any]], conn_map: Dict[int, Connection]
    ):
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
                    if "frame_id" in meta_info[cid] and hasattr(msg, "header") and hasattr(msg.header, "frame_id"):
                        new_frame_id = meta_info[cid]["frame_id"]
                        if msg.header.frame_id != new_frame_id:
                            msg.header.frame_id = new_frame_id
                            msg_modified = True

                    # ✅ 2. 詳細編集データの適用 (将来の拡張ポイント)
                    if cid in self.detail_edit_data:
                        edit_info = self.detail_edit_data[cid]
                        msgtype = conn.msgtype

                        # --- ここにメッセージ型ごとの編集ロジックを実装 ---
                        if msgtype == "sensor_msgs/msg/CameraInfo" and edit_info["type"] == "camera_info":
                            edit_data = edit_info["data"]
                            msg.width = np.uint32(edit_data["width"])
                            msg.height = np.uint32(edit_data["height"])
                            msg.distortion_model = edit_data["distortion_model"]
                            msg.k = np.array(edit_data["k"], dtype=np.float64)
                            msg.d = np.array(edit_data["d"], dtype=np.float64)
                            msg.r = np.array(edit_data["r"], dtype=np.float64)
                            msg.p = np.array(edit_data["p"], dtype=np.float64)
                            msg_modified = True

                        elif msgtype == "tf2_msgs/msg/TFMessage" and edit_info["type"] == "tf_static":
                            # 新しいtransformのリストを作成
                            new_tf_list = []
                            for tf_data in edit_info["data"]:
                                # ダイアログから来たデータ (SimpleNamespace) をrosbagsの型に変換
                                h = tf_data.header
                                t = tf_data.transform
                                new_tf = TransformStamped(
                                    header=Header(
                                        stamp=Time(sec=h.stamp.sec, nanosec=h.stamp.nanosec), frame_id=h.frame_id
                                    ),
                                    child_frame_id=tf_data.child_frame_id,
                                    transform=t,
                                )
                                new_tf_list.append(new_tf)

                            # メッセージのtransformsを丸ごと入れ替える
                            msg.transforms = np.array(new_tf_list)
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
                topic=info["topic"], msgtype=info["msgtype"], typestore=reader.typestore
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
            ext = cast("ConnectionExtRosbag2", connection.ext)
            serialization_format = getattr(ext, "serialization_format", "cdr")
            qos = info.get("qos", (getattr(ext, "offered_qos_profiles", []) or [None])[0])
            conn_map[cid] = writer.add_connection(
                topic=info["topic"],
                msgtype=info["msgtype"],
                typestore=reader.typestore,
                serialization_format=serialization_format,
                offered_qos_profiles=[qos] if qos else [],
            )

        # 2) メッセージの処理と書き込みを共通ヘルパーに委譲
        self._process_and_write_messages(reader, writer, meta_info, conn_map)
