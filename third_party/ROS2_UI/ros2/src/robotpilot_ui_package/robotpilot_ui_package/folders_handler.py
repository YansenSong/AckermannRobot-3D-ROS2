#!/usr/bin/env python3
import webbrowser
import rclpy
from rclpy.node import Node
import os
import shutil
import json
import time
import subprocess
import threading
import uuid
import yaml
import csv
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from robotpilot_ui_msgs.msg import ArrayPoseStampedWithCovariance
from nav2_msgs.srv import LoadMap


class UIFoldersHandler(Node):
    def __init__(self):
        super().__init__('ui_folders')
        self.declare_parameter('map_management_only', False)
        self.map_management_only = self.get_parameter('map_management_only').value
        self.declare_parameter('open_browser', True)
        self.WPs = []
        self.waypoints = []
        self.position = 0

        self.get_logger().info("------------ UI folders handler started ------------")

        config_file = os.path.join(get_package_share_directory('robotpilot_ui_package'), 'param', 'config.yaml')
        with open(config_file, 'r') as file:
            data = yaml.safe_load(file) or {}
        self.local_ip = data["ui/flask_app"]["ros__parameters"]["appAddress"]
        self.local_port = data["ui/flask_app"]["ros__parameters"]["portApp"]

        self.odomsub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.uiopsub = self.create_subscription(String, "ui_operation", self.ui_callback, 10)
        self.waysub = self.create_subscription(PoseWithCovarianceStamped, "/new_way_point", self.new_way_point_callback, 10)
        self.navsub = self.create_subscription(Empty, "/nav_data_req", self.nav_data_callback, 10)
        self.reqsub = self.create_subscription(Empty, "WP_req", self.WP_req_callback, 10)

        self.ui_pub = self.create_publisher(String, 'ui_message', 1)
        self.poseArray_publisher = self.create_publisher(ArrayPoseStampedWithCovariance, "/WayPoints_topic", 1)
        self.set_pose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        self.nav_data_pub = self.create_publisher(String, 'nav_data_resp', 1)

        package_share_dir = get_package_share_directory('robotpilot_ui_package')
        project_root = os.environ.get('ACKERMANN_ROBOT_WS', '')
        self.maps_folder = (
            os.path.join(project_root, 'maps', 'ui')
            if project_root and os.path.isdir(project_root)
            else os.path.join(package_share_dir, 'maps')
        )
        self.routs_folder = os.path.join(package_share_dir, 'paths')
        self.route_store_folder = os.path.join(
            os.path.expanduser(os.environ.get('ROS_HOME', '~/.ros')),
            'ackermann_robot', 'routes',
        )
        self.current_files = os.path.join(
            os.path.dirname(self.route_store_folder), 'current_map_route.yaml'
        )
        os.makedirs(os.path.dirname(self.current_files), exist_ok=True)
        if not os.path.exists(self.current_files):
            with open(self.current_files, 'w') as file:
                yaml.safe_dump({"map_file": "", "route_file": ""}, file)

        if not self.map_management_only:
            try:
                with open(self.current_files, 'r') as file:
                    cur_data = yaml.safe_load(file) or {}

                needs_update = False
                if cur_data:
                    map_path = cur_data.get("map_file", "")
                    route_path = cur_data.get("route_file", "")

                    if map_path and not os.path.exists(os.path.expanduser(map_path)):
                        cur_data["map_file"] = ""
                        needs_update = True
                    if route_path and not os.path.exists(os.path.expanduser(route_path)):
                        cur_data["route_file"] = ""
                        needs_update = True

                if needs_update:
                    with open(self.current_files, 'w') as file:
                        yaml.dump(cur_data, file)
                    self.get_logger().info("Automatically corrected current_map_route.yaml paths.")
            except Exception as e:
                self.get_logger().error(f"Error checking current_map_route.yaml: {e}")

        self.dict_cmd = None
        self._save_map_busy = False

        # Connect to the map_server already managed by Nav2's lifecycle manager.
        self.change_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.get_logger().info('Checking map_server availability...')
        if self.change_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('CONNECTED to map_server')
        else:
            self.get_logger().warn('map_server/load_map is not available yet; map changes will retry later.')
        if self.get_parameter('open_browser').value:
            webbrowser.open(f"http://{self.local_ip}:{self.local_port}")

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _pub(self, text: str):
        self.ui_pub.publish(String(data=text))

    def _pub_nav_data(self):
        self.nav_data_pub.publish(String(data=self.get_paths()))

    def _saved_map_entries(self):
        entries = []
        if not os.path.isdir(self.maps_folder):
            return entries
        for group in sorted(os.listdir(self.maps_folder)):
            group_path = os.path.join(self.maps_folder, group)
            if not os.path.isdir(group_path):
                continue
            for filename in sorted(os.listdir(group_path)):
                if not filename.endswith(".yaml") or filename.endswith("_ros.yaml"):
                    continue
                entries.append(
                    (group, filename[:-5], os.path.join(group_path, filename))
                )
        return entries

    @staticmethod
    def _path_is_within(path, parent):
        if not path:
            return False
        try:
            return os.path.commonpath(
                [os.path.abspath(path), os.path.abspath(parent)]
            ) == os.path.abspath(parent)
        except ValueError:
            return False

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_callback(self, data: Odometry):
        self.position = data.pose.pose

    def nav_data_callback(self, data: Empty):
        time.sleep(0.5)
        self._pub_nav_data()

    def new_way_point_callback(self, data: PoseWithCovarianceStamped):
        line  = f"{data.pose.pose.position.x},"
        line += f"{data.pose.pose.position.y},"
        line += f"{data.pose.pose.position.z},"
        line += f"{data.pose.pose.orientation.x},"
        line += f"{data.pose.pose.orientation.y},"
        line += f"{data.pose.pose.orientation.z},"
        line += f"{data.pose.pose.orientation.w},"
        line += f"{data.pose.covariance[0]},"
        line += f"{data.pose.covariance[1]},"
        line += f"{data.pose.covariance[2]}"
        if line not in self.WPs:
            self.WPs.append(line)
            count = len(self.WPs)
            self._pub("1 waypoint added to the route" if count == 1 else f"{count} waypoints added to the route")
        else:
            self.get_logger().info("Waypoint already exists in the route")

    def convert_PoseArray(self, waypoints: list):
        poses = PoseArray()
        poses.header.frame_id = 'map'
        poses.poses = [pose.pose.pose for pose, purpose in waypoints]
        return poses

    def convert_PoseWithCovArray_to_PoseArrayCov(self, waypoints: list):
        poses = ArrayPoseStampedWithCovariance()
        for pose_arg, purpose in waypoints:
            poses.poses.append(pose_arg)
        return poses

    def WP_req_callback(self, data: Empty):
        time.sleep(0.5)
        self.read_wp()
        self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))

    def read_wp(self):
        route_file = os.path.expanduser(self.get_cur_files().get("route_file", ""))
        del self.waypoints[:]
        if route_file:
            if not os.path.isfile(route_file):
                self.get_logger().warn(f"Route file does not exist: {route_file}")
            else:
                with open(route_file, 'r') as file:
                    reader = csv.reader(file, delimiter=',')
                    for row_number, line in enumerate(reader, start=1):
                        if len(line) < 11:
                            self.get_logger().warn(f"Skipping malformed waypoint row {row_number} in {route_file}")
                            continue
                        try:
                            values = [float(value) for value in line[:11]]
                        except ValueError:
                            self.get_logger().warn(f"Skipping non-numeric waypoint row {row_number} in {route_file}")
                            continue

                        current_pose = PoseWithCovarianceStamped()
                        current_pose.header.frame_id = 'map'
                        current_pose.pose.pose.position.x = values[0]
                        current_pose.pose.pose.position.y = values[1]
                        current_pose.pose.pose.position.z = values[2]
                        current_pose.pose.pose.orientation.x = values[3]
                        current_pose.pose.pose.orientation.y = values[4]
                        current_pose.pose.pose.orientation.z = values[5]
                        current_pose.pose.pose.orientation.w = values[6]
                        current_pose.pose.covariance[0] = values[7]
                        current_pose.pose.covariance[1] = values[8]
                        current_pose.pose.covariance[2] = values[9]
                        self.waypoints.append((current_pose, values[10]))

        if not self.waypoints:
            self._pub("The waypoint queue is empty.")

    def get_paths(self):
        files = {}
        route_identities = {}
        try:
            with open(os.path.join(self.route_store_folder, 'map_identities.json'), 'r') as stream:
                route_identities = json.load(stream)
        except (OSError, ValueError):
            pass
        if not os.path.isdir(self.maps_folder):
            self.get_logger().warn(f"Maps folder does not exist: {self.maps_folder}")
            return json.dumps({"catalog_source": "maps", "structure": [], "active_files": {"group": "Null", "map": "Null", "route": "Null"}})

        for group in sorted(os.listdir(self.maps_folder)):
            group_path = os.path.join(self.maps_folder, group)
            if not os.path.isdir(group_path):
                continue
            files[group] = []
            for filename in sorted(os.listdir(group_path)):
                if not filename.endswith('.yaml') or filename.endswith('_ros.yaml'):
                    continue
                map_name = filename[:-5]
                route_dir = os.path.join(self.routs_folder, group, map_name)
                routes = {
                    name for name in os.listdir(route_dir) if name.endswith('.csv')
                } if os.path.isdir(route_dir) else set()
                for map_key, identity in route_identities.items():
                    if identity.get('group') != group or identity.get('map') != map_name:
                        continue
                    stored_dir = os.path.join(self.route_store_folder, map_key)
                    if os.path.isdir(stored_dir):
                        routes.update(
                            entry for entry in os.listdir(stored_dir)
                            if entry.endswith('.csv')
                        )
                files[group].append({map_name: sorted(routes)})

        data = self.get_cur_files()
        route_file = data.get("route_file", "")
        map_file = data.get("map_file", "")
        if route_file:
            route_parts = os.path.expanduser(route_file).split("/")[-3:]
            if len(route_parts) == 3:
                group, map_name, route = route_parts[0], route_parts[1], route_parts[2].split(".")[0]
            else:
                group, map_name, route = "Null", "Null", "Null"
        elif map_file:
            map_parts = os.path.expanduser(map_file).split("/")[-2:]
            if len(map_parts) == 2:
                group, map_name, route = map_parts[0], map_parts[1].split(".")[0], "Null"
            else:
                group, map_name, route = "Null", "Null", "Null"
        else:
            group, map_name, route = "Null", "Null", "Null"

        response = {"catalog_source": "maps", "structure": [], "active_files": {"group": group, "map": map_name, "route": route}}
        for i, j in files.items():
            response["structure"].append({i: j})
        return json.dumps(response)

    # ── Map commands ─────────────────────────────────────────────────────────

    def save_map_func(self):
        try:
            if self._save_map_busy:
                raise RuntimeError('A map save is already in progress')
            group = self.dict_cmd['group']
            name = self.dict_cmd['map']
            if not group or not name or any(part in ('.', '..') or '/' in part or '\\' in part for part in (group, name)):
                raise ValueError('Invalid map group or name')
            map_group = os.path.join(self.maps_folder, group)
            map_path_to_save = os.path.join(map_group, name)
            route_folder_path_to_save = os.path.join(self.routs_folder, group, name)
            if any(os.path.exists(path) for path in (
                map_path_to_save, f"{map_path_to_save}.yaml",
                f"{map_path_to_save}.pgm", route_folder_path_to_save,
            )):
                raise FileExistsError(f'Map already exists: {group}/{name}')
            project_root = os.environ.get('ACKERMANN_ROBOT_WS', '')
            converter = os.path.join(project_root, 'third_party', 'pcd2pgm', 'build', 'pcd2gridmap')
            if not project_root or not os.access(converter, os.X_OK):
                raise RuntimeError(
                    'PCD converter unavailable; build it in AckermannRobot with '
                    'cmake -S third_party/pcd2pgm -B third_party/pcd2pgm/build '
                    'and cmake --build third_party/pcd2pgm/build'
                )
            try:
                from lio_sam.srv import SaveMap
            except ImportError as error:
                raise RuntimeError('lio_sam is unavailable in the UI ROS environment') from error
            if not hasattr(self, '_lio_save_client'):
                self._lio_save_client = self.create_client(SaveMap, '/lio_sam/save_map')
            if not self._lio_save_client.wait_for_service(timeout_sec=0.2):
                raise RuntimeError('/lio_sam/save_map is unavailable; start LIO-SAM mapping first')

            os.makedirs(map_group, exist_ok=True)
            staging = os.path.join(map_group, f'.{name}.saving-{uuid.uuid4().hex}')
            request = SaveMap.Request()
            request.resolution = 0.2
            request.destination = staging
            future = self._lio_save_client.call_async(request)
            self._save_map_busy = True
            self._pub('Saving LIO-SAM point cloud...')

            def finish(finished):
                threading.Thread(
                    target=self._finish_lio_map_save,
                    args=(finished, staging, map_path_to_save, route_folder_path_to_save, converter, name),
                    daemon=True,
                ).start()

            future.add_done_callback(finish)
        except Exception as e:
            self.get_logger().error(f"Error in save_map_func: {e}")
            self._pub(f"Map save failed: {e}")

    def _finish_lio_map_save(self, future, staging, map_base, route_folder, converter, name):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError('LIO-SAM could not save the point cloud')
            snapshots = [entry.path for entry in os.scandir(staging) if entry.is_dir()]
            if len(snapshots) != 1:
                raise RuntimeError(f'Expected one LIO-SAM map in {staging}')
            saved_dir = snapshots[0]
            pcd = os.path.join(saved_dir, 'GlobalMap.pcd')
            if not os.path.isfile(pcd):
                raise RuntimeError(f'LIO-SAM did not create {pcd}')

            prefix = os.path.join(staging, name)
            result = subprocess.run(
                [converter, pcd, '-o', prefix],
                capture_output=True, text=True, timeout=180, check=False,
            )
            if result.returncode != 0:
                raise RuntimeError((result.stderr or result.stdout or 'PCD conversion failed').strip())
            if not all(os.path.isfile(f'{prefix}{ext}') for ext in ('.yaml', '.pgm')):
                raise RuntimeError('PCD converter did not create map.yaml and map.pgm')

            os.rename(saved_dir, map_base)
            shutil.copy2(f'{prefix}.pgm', os.path.join(map_base, 'map.pgm'))
            with open(f'{prefix}.yaml', encoding='utf-8') as source:
                navigation_map = yaml.safe_load(source) or {}
            navigation_map['image'] = 'map.pgm'
            with open(os.path.join(map_base, 'map.yaml'), 'w', encoding='utf-8') as target:
                yaml.safe_dump(navigation_map, target)
            os.replace(f'{prefix}.pgm', f'{map_base}.pgm')
            os.replace(f'{prefix}.yaml', f'{map_base}.yaml')
            os.rmdir(staging)
            os.makedirs(route_folder, exist_ok=True)
            self._pub_nav_data()
            self._pub(f'Map saved "{name}"')
        except Exception as error:
            self.get_logger().error(f'Map save failed: {error}')
            self._pub(f'Map save failed: {error}')
        finally:
            self._save_map_busy = False

    def change_map_func(self):
        path_to_new_map = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
        self.change_map(path_to_new_map)

    def create_group_func(self):
        try:
            os.mkdir(f"{self.routs_folder}/{self.dict_cmd['group']}")
            os.mkdir(f"{self.maps_folder}/{self.dict_cmd['group']}")
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in create_group_func: {e}")

    def rename_map_func(self):
        try:
            old_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_map_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}"
            old_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}"
            new_route_folder_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}"
            old_ros_folder_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_old']}_ros"
            new_ros_folder_file = f"{self.maps_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map_new']}_ros"

            with open(f"{old_map_file}.yaml", 'r') as file:
                map_cur_path = yaml.safe_load(file) or {}
            image_file = map_cur_path.get("image", f"{self.dict_cmd['map_old']}.png")
            image_extension = os.path.splitext(image_file)[1] or ".png"
            map_cur_path["image"] = f"{self.dict_cmd['map_new']}{image_extension}"
            if os.path.isfile(f"{old_ros_folder_file}.yaml"):
                with open(f"{old_ros_folder_file}.yaml", 'r') as file:
                    ros_cur_path = yaml.safe_load(file) or {}
                if "map_server" in ros_cur_path and "ros__parameters" in ros_cur_path["map_server"]:
                    ros_cur_path["map_server"]["ros__parameters"]["yaml_filename"] = f"{self.dict_cmd['map_new']}.yaml"
                else:
                    ros_cur_path["yaml_filename"] = f"{self.dict_cmd['map_new']}.yaml"
                with open(f"{old_ros_folder_file}.yaml", 'w') as file:
                    yaml.dump(ros_cur_path, file)
            with open(f"{old_map_file}.yaml", 'w') as file:
                yaml.dump(map_cur_path, file)

            os.rename(f"{old_map_file}.yaml", f"{new_map_file}.yaml")
            os.rename(f"{old_map_file}{image_extension}", f"{new_map_file}{image_extension}")
            if os.path.isdir(old_map_file):
                os.rename(old_map_file, new_map_file)
            if os.path.isfile(f"{old_ros_folder_file}.yaml"):
                os.rename(f"{old_ros_folder_file}.yaml", f"{new_ros_folder_file}.yaml")
            os.rename(old_route_folder_file, new_route_folder_file)

            data = self.get_cur_files()
            if os.path.expanduser(data["map_file"]) == f"{old_map_file}.yaml":
                self.set_cur_map(new_map_file)
                self.set_cur_route(f"{new_route_folder_file}/{data['route_file'].split('/')[-1].split('.')[0]}")

            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in rename_map_func: {e}")

    def delete_map_func(self):
        try:
            group = self.dict_cmd["group"]
            map_name = self.dict_cmd["map"]
            map_base = os.path.join(self.maps_folder, group, map_name)
            route_map_folder = os.path.join(self.routs_folder, group, map_name)
            removed = False

            for extension in (".yaml", "_ros.yaml", ".png", ".pgm"):
                map_file = f"{map_base}{extension}"
                if os.path.isfile(map_file):
                    os.remove(map_file)
                    removed = True
            if os.path.isdir(map_base):
                shutil.rmtree(map_base)
                removed = True
            if os.path.isdir(route_map_folder):
                shutil.rmtree(route_map_folder)
                removed = True
            if not removed:
                raise FileNotFoundError(
                    f"No saved map or route data for {group}/{map_name}"
                )

            current = self.get_cur_files()
            active_map = os.path.expanduser(current.get("map_file", ""))
            active_route = os.path.expanduser(current.get("route_file", ""))
            deleted_active_map = os.path.abspath(active_map) == os.path.abspath(
                f"{map_base}.yaml"
            )
            deleted_active_route = self._path_is_within(
                active_route, route_map_folder
            )

            if deleted_active_map:
                candidates = self._saved_map_entries()
                if candidates:
                    next_group, next_map, next_file = candidates[0]
                    self.dict_cmd = {"group": next_group, "map": next_map}
                    self.change_map(next_file, yaml=True)
                else:
                    self.set_cur_map("")
                    self.set_cur_route("")
                    self._pub("No maps remain")
            elif deleted_active_route:
                self.set_cur_route("")
                self._pub("No routes on the map")

            self.WP_req_callback(Empty())
            self._pub_nav_data()
            self._pub(f'Deleted map "{map_name}"')
        except Exception as e:
            self.get_logger().error(f"Error in delete_map_func: {e}")
            self._pub(f"Map deletion failed: {e}")

    def delete_group_func(self):
        try:
            group = self.dict_cmd["group"]
            map_group = os.path.join(self.maps_folder, group)
            route_group = os.path.join(self.routs_folder, group)
            if not os.path.isdir(map_group) and not os.path.isdir(route_group):
                raise FileNotFoundError(f"No saved map group named {group}")

            current = self.get_cur_files()
            active_map = os.path.expanduser(current.get("map_file", ""))
            active_route = os.path.expanduser(current.get("route_file", ""))
            deleted_active_map = self._path_is_within(active_map, map_group)
            deleted_active_route = self._path_is_within(active_route, route_group)

            if os.path.isdir(map_group):
                shutil.rmtree(map_group)
            if os.path.isdir(route_group):
                shutil.rmtree(route_group)

            if deleted_active_map:
                candidates = self._saved_map_entries()
                if candidates:
                    next_group, next_map, next_file = candidates[0]
                    self.dict_cmd = {"group": next_group, "map": next_map}
                    self.change_map(next_file, yaml=True)
                else:
                    self.set_cur_map("")
                    self.set_cur_route("")
                    self._pub("No maps remain")
            elif deleted_active_route:
                self.set_cur_route("")

            self.WP_req_callback(Empty())
            self._pub_nav_data()
            self._pub(f'Deleted group "{group}"')
        except Exception as e:
            self.get_logger().error(f"Error in delete_group_func: {e}")
            self._pub(f"Group deletion failed: {e}")

    # ── Route commands ────────────────────────────────────────────────────────

    def clear_route_func(self):
        try:
            del self.WPs[:]
            self._pub("Waypoints cleared, please set new points on the map")
        except Exception as e:
            self.get_logger().info(f"Error in clear_route_func: {e}")

    def save_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"
            with open(f"{path_to_route}.csv", 'w') as file:
                for WP in self.WPs:
                    file.write(WP + ", 1\n")
            self._pub(f"{len(self.WPs)} waypoints saved")
            self.set_cur_route(path_to_route)
            del self.WPs[:]
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in save_route_func: {e}")

    def edit_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route'].split('.')[0]}"
            self.set_cur_route(path_to_route)
            self.read_wp()
            for point, purpose in self.waypoints:
                if purpose == 2:
                    continue
                line = f"{point.pose.pose.position.x},{point.pose.pose.position.y},{point.pose.pose.position.z},{point.pose.pose.orientation.x},{point.pose.pose.orientation.y},{point.pose.pose.orientation.z},{point.pose.pose.orientation.w},{point.pose.covariance[0]},{point.pose.covariance[1]},{point.pose.covariance[2]}"
                if line not in self.WPs:
                    self.WPs.append(line)
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in edit_route_func: {e}")

    def delete_route_func(self):
        try:
            file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}.csv"
            os.remove(file)
            routes_on_map = os.listdir(f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}")
            if routes_on_map:
                path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{routes_on_map[0].split('.')[0]}"
                self.set_cur_route(path_to_route)
            else:
                self.set_cur_route("")
                self._pub("No routes on the map")
            self.WP_req_callback(Empty())
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in delete_route_func: {e}")

    def change_route_func(self):
        try:
            path_to_route = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route']}"
            self.set_cur_route(path_to_route)
            self.read_wp()
            self.poseArray_publisher.publish(self.convert_PoseWithCovArray_to_PoseArrayCov(self.waypoints))
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in change_route_func: {e}")

    def rename_route_func(self):
        try:
            old_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_old']}.csv"
            new_file = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}/{self.dict_cmd['route_new']}.csv"
            os.rename(old_file, new_file)
            self.set_cur_route(new_file.split(".")[0])
            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in rename_route_func: {e}")

    def rename_group_func(self):
        try:
            old_maps = f"{self.maps_folder}/{self.dict_cmd['group_old']}"
            new_maps = f"{self.maps_folder}/{self.dict_cmd['group_new']}"
            old_routes = f"{self.routs_folder}/{self.dict_cmd['group_old']}"
            new_routes = f"{self.routs_folder}/{self.dict_cmd['group_new']}"
            os.rename(old_maps, new_maps)
            os.rename(old_routes, new_routes)

            data = self.get_cur_files()
            if self.dict_cmd['group_old'] in data.get("map_file", ""):
                data["map_file"] = data["map_file"].replace(old_maps, new_maps)
            if self.dict_cmd['group_old'] in data.get("route_file", ""):
                data["route_file"] = data["route_file"].replace(old_routes, new_routes)
            with open(self.current_files, 'w') as file:
                yaml.dump(data, file)

            self._pub_nav_data()
        except Exception as e:
            self.get_logger().info(f"Error in rename_group_func: {e}")

    # ── UI command dispatcher ─────────────────────────────────────────────────

    def ui_callback(self, data: String):
        self.get_logger().info(f"COMMAND RECEIVED: {data}")
        try:
            command = data.data.split("/")
            map_commands = {
                "save_map",
                "change_map",
                "create_group",
                "rename_map",
                "delete_map",
                "delete_group",
                "rename_group",
            }
            if self.map_management_only and command[0] not in map_commands:
                return
            if len(command) > 1:
                self.dict_cmd = json.loads(command[1])

            dispatch = {
                "save_map":     self.save_map_func,
                "change_map":   self.change_map_func,
                "create_group": self.create_group_func,
                "rename_map":   self.rename_map_func,
                "delete_map":   self.delete_map_func,
                "delete_group": self.delete_group_func,
                "clear_route":  self.clear_route_func,
                "save_route":   self.save_route_func,
                "edit_route":   self.edit_route_func,
                "delete_route": self.delete_route_func,
                "change_route": self.change_route_func,
                "rename_route": self.rename_route_func,
                "rename_group": self.rename_group_func,
            }
            fn = dispatch.get(command[0])
            if fn:
                self.get_logger().info(command[0])
                fn()
            else:
                self.get_logger().warn(f"Unknown UI command: {command[0]}")
        except Exception as e:
            self.get_logger().error(f"Error in ui_callback: {e}")

    # ── Map service helpers ───────────────────────────────────────────────────

    def change_map(self, map_name: str, yaml: bool = False, manual: bool = False):
        self.get_logger().info(f"\n ==========[CHANGING MAP TO {map_name}]======== \n")
        map_yaml_file = map_name if yaml else f"{map_name}.yaml"
        self.set_cur_map(map_yaml_file)

        if manual:
            parts = map_name.split("/")
            self.dict_cmd['group'] = parts[-2]
            self.dict_cmd['map'] = parts[-1].split(".")[0]

        route_dir = f"{self.routs_folder}/{self.dict_cmd['group']}/{self.dict_cmd['map']}"
        routes_on_map = os.listdir(route_dir) if os.path.isdir(route_dir) else []
        if routes_on_map:
            path_to_route = f"{route_dir}/{routes_on_map[0].split('.')[0]}"
            self.set_cur_route(path_to_route)
        else:
            self.set_cur_route("")
            self._pub("No routes on the map")

        self.WP_req_callback(Empty())

        req = LoadMap.Request()
        req.map_url = map_yaml_file
        if self.change_map_cli.service_is_ready() or self.change_map_cli.wait_for_service(timeout_sec=1.0):
            self.change_map_cli.call_async(req)
        else:
            self.get_logger().warn("Cannot load map because /map_server/load_map is unavailable.")
            self._pub("Map server is unavailable; map selection was saved but not loaded.")
        self._pub_nav_data()

    def set_cur_map(self, map_name: str):
        data = self.get_cur_files()
        data["map_file"] = map_name
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)

    def set_cur_route(self, route_name: str):
        data = self.get_cur_files()
        data["route_file"] = f"{route_name}.csv" if route_name else ""
        with open(self.current_files, 'w') as file:
            yaml.dump(data, file)

    def get_cur_files(self):
        with open(self.current_files, 'r') as file:
            return yaml.safe_load(file) or {"map_file": "", "route_file": ""}


def main():
    rclpy.init()
    controller = UIFoldersHandler()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
