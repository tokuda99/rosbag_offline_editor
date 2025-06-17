from pathlib import Path
from typing import Dict, Any, Tuple

# rosbags関連
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer as Rosbag1Writer
from rosbags.rosbag2 import Writer as Rosbag2Writer
from typing import TYPE_CHECKING, cast
import yaml
from fnmatch import fnmatch

from rosbags.interfaces import ConnectionExtRosbag2


class BagModel:
    """
    rosbag(ROS1/ROS2) のメタ情報や実際の読み込み・書き込み処理を担う Model。
    """

    def __init__(self):
        self.bag_path: Path = None
        self.rosbag_version: str = None
        self.meta_info: Dict[int, Dict[str, Any]] = {}

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
        with AnyReader([bag_path]) as reader:
            pass
            for connection in reader.connections:
                meta_info[connection.id] = {
                    "topic": connection.topic,
                    "msgtype": connection.msgtype
                }
                if rosbag_version == "ROS2":
                    ext = cast('ConnectionExtRosbag2', connection.ext)
                    meta_info[connection.id]["serialization_format"] = ext.serialization_format
                    qos = Qos(ext.offered_qos_profiles)
                    meta_info[connection.id]["qos"] = qos
            
            for i, (conn, ts, data) in enumerate(reader.messages()):
                msg = reader.typestore.deserialize_cdr(data, conn.msgtype)
                if header := getattr(msg, 'header', None):
                    meta_info[conn.id]["frame_id"] = header.frame_id
                    frame_ids.add(conn.id)
                if len(frame_ids) == len(meta_info):
                    break

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
        with AnyReader([in_path]) as reader:
            if out_format == "ROS1":
                with Rosbag1Writer(out_path) as writer:
                    self._write_bag1(reader, writer, updated_meta)
            else:
                with Rosbag2Writer(out_path) as writer:
                    self._write_bag2(reader, writer, updated_meta)

    def _write_bag1(self, reader: AnyReader, writer, meta_info: Dict[int, Dict[str, Any]]):
        """
        メッセージを1つずつ読み出して書き込み。
        """
        filtered_msg = ["rcl_interfaces/*", "rosbag2_interfaces/*"]
        conn_map = {}
        # 1) 接続情報を追加
        for connection in reader.connections:
            cid = connection.id
            if cid not in meta_info:
                continue
            if any([fnmatch(connection.msgtype, pat) for pat in filtered_msg]):
                print(f"Skip: {connection.msgtype}")
                continue
            new_topic = meta_info[cid]["topic"]
            new_msgtype = meta_info[cid]["msgtype"]
            conn_map[cid] = writer.add_connection(
                topic=new_topic,
                msgtype=new_msgtype,
                typestore=reader.typestore  # 必要に応じて変更
            )
        # 2) メッセージ実体をコピー
        for conn, ts, data in reader.messages():
            if conn.id not in conn_map:
                continue
            wconn = conn_map[conn.id]
            writer.write(wconn, ts, data)

    def _write_bag2(self, reader: AnyReader, writer, meta_info: Dict[int, Dict[str, Any]]):
        """
        メッセージを1つずつ読み出して書き込み。
        """
        conn_map = {}
        filtered_msg = ["rcl_interfaces/*", "rosbag2_interfaces/*"]

        # 1) 接続情報を追加
        for connection in reader.connections:
            cid = connection.id
            if cid not in meta_info:
                continue
            if any([fnmatch(connection.msgtype, pat) for pat in filtered_msg]):
                print(f"Skip: {connection.msgtype}")
                continue
            
            new_topic = meta_info[cid]["topic"]
            new_msgtype = meta_info[cid]["msgtype"]
            ext = cast('ConnectionExtRosbag2', connection.ext)
            qos = meta_info[cid]["qos"].qos_str
            conn_map[cid] = writer.add_connection(
                topic=new_topic,
                msgtype=new_msgtype,
                typestore=reader.typestore,
                serialization_format=ext.serialization_format,
                offered_qos_profiles=qos
            )
        # 2) メッセージ実体をコピー
        for conn, ts, data in reader.messages():
            if conn.id not in conn_map:
                continue
            wconn = conn_map[conn.id]
            writer.write(wconn, ts, data)