#!/usr/bin/env python3
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rospy
import rospkg
from PyQt5.QtCore import QObject, QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFrame,
    QGraphicsPathItem,
    QGraphicsPixmapItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ackermann_car_gazebo.msg import RtkImuEkfState
from ackermann_car_gazebo.srv import PlanCoveragePath, PlanCoveragePathRequest
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, String


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    stamp: Optional[rospy.Time] = None
    source: str = ""
    valid: bool = False


class UiDispatcher(QObject):
    callback = pyqtSignal(object)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def finite_values(*values: float) -> bool:
    return all(math.isfinite(value) for value in values)


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class TerrainView(QGraphicsView):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent_window = parent
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setRenderHint(QPainter.SmoothPixmapTransform, True)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.parent_window.add_polygon_point(self.mapToScene(event.pos()))
            event.accept()
            return
        if event.button() == Qt.RightButton:
            self.parent_window.close_polygon()
            event.accept()
            return
        super().mousePressEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.parent_window.fit_map_to_view()
        self.parent_window.position_overlay_items()


class CoverageNavigationGui(QMainWindow):
    def __init__(self):
        super().__init__()
        rospy.init_node("coverage_navigation_gui", anonymous=True, disable_signals=True)

        self.render_path = self.resolve_resource_path(rospy.get_param(
            "~render_path",
            "/home/nav/nav_ws/src/ackermann_car_gazebo/worlds/render.png",
        ))
        self.map_world_size_m = float(rospy.get_param("~map_world_size_m", 129.0))
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.state_topic = rospy.get_param("~state_topic", "/ackermann_car/ekf_state")
        self.odom_topic = rospy.get_param("~odom_topic", "/ackermann_car/odom")
        self.fallback_odom_topic = rospy.get_param("~fallback_odom_topic", "/odom")
        self.planning_service = rospy.get_param(
            "~planning_service", "/coverage_path_planner/plan_coverage_path"
        )
        self.plan_topic = rospy.get_param("~plan_topic", "/plan_path")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/ackermann_car/cmd_vel")
        self.tracker_launch = rospy.get_param(
            "~tracker_launch", "ackermann_car_gazebo mpc_path_tracker.launch"
        )
        self.tracker_launch_args = rospy.get_param("~tracker_launch_args", [])
        self.sim_launch = rospy.get_param(
            "~sim_launch", "ackermann_car_gazebo gazebo_ackermann_car.launch"
        )
        self.sim_launch_args = rospy.get_param("~sim_launch_args", [])

        self.robot_state = RobotState()
        self.last_ekf_wall_time: Optional[float] = None
        self.last_odom_wall_time: Optional[float] = None
        self.last_fallback_odom_wall_time: Optional[float] = None
        self.origin_world: Optional[Tuple[float, float, float]] = None
        self.coverage_polygon_world: List[Tuple[float, float]] = []
        self.coverage_polygon_closed = False
        self.planned_path_local: List[Tuple[float, float]] = []
        self.planned_path_world: List[Tuple[float, float]] = []
        self.planned_path_indices: List[int] = []
        self.navigation_active = False
        self.navigation_progress: Optional[int] = None
        self.tracking_status = "idle"
        self.sim_process: Optional[subprocess.Popen] = None
        self.tracker_process: Optional[subprocess.Popen] = None
        self.dispatcher = UiDispatcher()
        self.dispatcher.callback.connect(lambda callback: callback())

        self.plan_pub = rospy.Publisher(self.plan_topic, Path, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.ekf_sub = rospy.Subscriber(self.state_topic, RtkImuEkfState, self.ekf_callback)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.fallback_odom_sub = rospy.Subscriber(
            self.fallback_odom_topic, Odometry, self.fallback_odom_callback
        )
        self.status_sub = rospy.Subscriber("/tracking_status", String, self.tracking_status_callback)

        self.setWindowTitle("Ackermann 全覆盖导航用户端")
        self.resize(1180, 980)
        self.build_ui()
        self.setup_scene()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(100)

    def resolve_resource_path(self, path: str) -> str:
        if not path.startswith("package://"):
            return path
        relative = path[len("package://") :]
        package_name, separator, package_path = relative.partition("/")
        if not package_name or not separator:
            return path
        try:
            package_root = rospkg.RosPack().get_path(package_name)
        except rospkg.ResourceNotFound:
            return path
        return os.path.join(package_root, package_path)

    def roslaunch_command(self, launch_spec: str, extra_args) -> List[str]:
        args = launch_spec.split()
        if isinstance(extra_args, list):
            args.extend(self.resolve_launch_arg(str(item)) for item in extra_args)
        elif extra_args:
            args.extend(self.resolve_launch_arg(item) for item in str(extra_args).split())
        return ["roslaunch", *args]

    def resolve_launch_arg(self, arg: str) -> str:
        name, separator, value = arg.partition(":=")
        if separator and value.startswith("package://"):
            return f"{name}:={self.resolve_resource_path(value)}"
        return arg

    def build_ui(self):
        central = QWidget(self)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        self.status_bar_frame = QFrame()
        self.status_bar_frame.setFrameShape(QFrame.StyledPanel)
        status_layout = QHBoxLayout(self.status_bar_frame)
        status_layout.setContentsMargins(10, 6, 10, 6)
        status_layout.setSpacing(14)

        self.position_label = QLabel("位置: X: --m  Y: --m  Z: --m")
        self.yaw_label = QLabel("朝向: --°")
        self.velocity_label = QLabel("速度: Vx: --m/s  Vy: --m/s")
        self.yaw_rate_label = QLabel("角速度: --rad/s")
        self.valid_label = QLabel("状态: Invalid")
        self.tracking_label = QLabel("导航: idle")
        for label in (
            self.position_label,
            self.yaw_label,
            self.velocity_label,
            self.yaw_rate_label,
            self.valid_label,
            self.tracking_label,
        ):
            status_layout.addWidget(label)
        status_layout.addStretch(1)
        root.addWidget(self.status_bar_frame)

        self.scene = QGraphicsScene(self)
        self.view = TerrainView(self)
        self.view.setScene(self.scene)
        root.addWidget(self.view, stretch=1)

        controls = QHBoxLayout()
        controls.setSpacing(10)
        controls.addWidget(QLabel("扫描宽度 (m)："))
        self.scan_width_spin = QDoubleSpinBox()
        self.scan_width_spin.setDecimals(2)
        self.scan_width_spin.setRange(0.1, 5.0)
        self.scan_width_spin.setSingleStep(0.1)
        self.scan_width_spin.setValue(0.5)
        controls.addWidget(self.scan_width_spin)

        controls.addWidget(QLabel("覆盖系数："))
        self.coverage_slider = QSlider(Qt.Horizontal)
        self.coverage_slider.setRange(1, 100)
        self.coverage_slider.setValue(80)
        self.coverage_slider.valueChanged.connect(self.update_coverage_label)
        controls.addWidget(self.coverage_slider, stretch=1)
        self.coverage_value_label = QLabel("0.80")
        controls.addWidget(self.coverage_value_label)

        self.clear_polygon_button = QPushButton("清除区域")
        self.clear_polygon_button.clicked.connect(self.clear_polygon)
        controls.addWidget(self.clear_polygon_button)

        self.plan_button = QPushButton("规划")
        self.plan_button.clicked.connect(self.plan_coverage_path)
        controls.addWidget(self.plan_button)

        self.navigation_button = QPushButton("导航")
        self.navigation_button.clicked.connect(self.toggle_navigation)
        self.navigation_button.setStyleSheet("background-color: #2563eb; color: white;")
        controls.addWidget(self.navigation_button)
        root.addLayout(controls)

        self.setCentralWidget(central)

    def setup_scene(self):
        pixmap = QPixmap(self.render_path)
        if pixmap.isNull():
            raise RuntimeError(f"无法加载地形图: {self.render_path}")
        self.pixmap = pixmap
        self.map_width_px = pixmap.width()
        self.map_height_px = pixmap.height()
        self.meters_per_pixel_x = self.map_world_size_m / float(self.map_width_px)
        self.meters_per_pixel_y = self.map_world_size_m / float(self.map_height_px)

        self.scene.clear()
        self.map_item = QGraphicsPixmapItem(pixmap)
        self.map_item.setTransformationMode(Qt.SmoothTransformation)
        self.map_item.setZValue(0)
        self.scene.addItem(self.map_item)
        self.scene.setSceneRect(QRectF(0, 0, self.map_width_px, self.map_height_px))

        self.sim_button = QPushButton("启动仿真")
        self.sim_button.setStyleSheet("background-color: #dc2626; color: white; font-weight: bold;")
        self.sim_button.clicked.connect(self.toggle_simulation)
        self.sim_proxy = self.scene.addWidget(self.sim_button)
        self.sim_proxy.setZValue(30)
        self.sim_proxy.setPos(12, 12)

        self.progress_text = QGraphicsSimpleTextItem("导航进度: --%")
        self.progress_text.setBrush(QBrush(QColor("#111827")))
        self.progress_text.setZValue(40)
        self.scene.addItem(self.progress_text)
        self.progress_bg = self.scene.addRect(QRectF(0, 0, 128, 28))
        self.progress_bg.setBrush(QBrush(QColor(255, 255, 255, 210)))
        self.progress_bg.setPen(QPen(QColor(209, 213, 219), 1))
        self.progress_bg.setZValue(39)

        self.robot_item = QGraphicsPolygonItem()
        self.robot_item.setBrush(QBrush(QColor("#ef4444")))
        self.robot_item.setPen(QPen(QColor("#7f1d1d"), 1.2))
        self.robot_item.setZValue(20)
        self.scene.addItem(self.robot_item)

        self.polygon_item = QGraphicsPathItem()
        self.polygon_item.setPen(QPen(QColor("#f59e0b"), 2.0, Qt.DashLine))
        self.polygon_item.setZValue(12)
        self.scene.addItem(self.polygon_item)

        self.path_item = QGraphicsPathItem()
        self.path_item.setPen(QPen(QColor(144, 238, 144), 2.0))
        self.path_item.setZValue(10)
        self.scene.addItem(self.path_item)

        self.fit_map_to_view()
        self.position_overlay_items()

    def fit_map_to_view(self):
        if not hasattr(self, "view") or not hasattr(self, "scene"):
            return
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def position_overlay_items(self):
        if not hasattr(self, "progress_text"):
            return
        margin = 12
        text_rect = self.progress_text.boundingRect()
        width = text_rect.width() + 18
        height = text_rect.height() + 10
        x = self.map_width_px - width - margin
        y = margin
        self.progress_bg.setRect(QRectF(x, y, width, height))
        self.progress_text.setPos(x + 9, y + 5)

    def update_coverage_label(self, value: int):
        self.coverage_value_label.setText(f"{value / 100.0:.2f}")

    def world_to_scene(self, x: float, y: float) -> QPointF:
        px = self.map_width_px / 2.0 + x / self.meters_per_pixel_x
        py = self.map_height_px / 2.0 - y / self.meters_per_pixel_y
        return QPointF(px, py)

    def scene_to_world(self, point: QPointF) -> Tuple[float, float]:
        x = (point.x() - self.map_width_px / 2.0) * self.meters_per_pixel_x
        y = (self.map_height_px / 2.0 - point.y()) * self.meters_per_pixel_y
        return x, y

    def world_to_local(self, x: float, y: float) -> Tuple[float, float]:
        if self.origin_world is None:
            return x, y
        return x - self.origin_world[0], y - self.origin_world[1]

    def local_to_world(self, x: float, y: float) -> Tuple[float, float]:
        if self.origin_world is None:
            return x, y
        return x + self.origin_world[0], y + self.origin_world[1]

    def add_polygon_point(self, scene_point: QPointF):
        if self.coverage_polygon_closed:
            return
        if not self.scene.sceneRect().contains(scene_point):
            return
        self.coverage_polygon_world.append(self.scene_to_world(scene_point))
        self.update_polygon_item()

    def close_polygon(self):
        if len(self.coverage_polygon_world) < 3:
            self.show_warning("覆盖区域至少需要 3 个点")
            return
        self.coverage_polygon_closed = True
        self.update_polygon_item()

    def clear_polygon(self):
        self.coverage_polygon_world = []
        self.coverage_polygon_closed = False
        self.planned_path_local = []
        self.planned_path_world = []
        self.planned_path_indices = []
        self.navigation_progress = None
        self.plan_button.setText("规划")
        self.path_item.setPath(QPainterPathCompat.empty_path())
        self.update_polygon_item()
        self.update_progress_label()

    def update_polygon_item(self):
        path = QPainterPathCompat.path_from_points(
            [self.world_to_scene(x, y) for x, y in self.coverage_polygon_world],
            closed=self.coverage_polygon_closed,
        )
        self.polygon_item.setPath(path)

    def ekf_callback(self, msg: RtkImuEkfState):
        self.last_ekf_wall_time = time.monotonic()
        state = RobotState()
        state.x = msg.x
        state.y = msg.y
        state.z = 0.0
        state.yaw = msg.yaw
        state.vx = msg.vx
        state.vy = msg.vy
        state.yaw_rate = 0.0
        state.stamp = msg.header.stamp
        state.source = "ekf"
        state.valid = finite_values(state.x, state.y, state.yaw, state.vx, state.vy)
        self.robot_state = state

    def odom_callback(self, msg: Odometry):
        now = time.monotonic()
        self.last_odom_wall_time = now
        if self.last_ekf_wall_time is not None and now - self.last_ekf_wall_time < 0.75:
            return
        self.robot_state = self.state_from_odom(msg, "odom")

    def fallback_odom_callback(self, msg: Odometry):
        now = time.monotonic()
        self.last_fallback_odom_wall_time = now
        if self.last_ekf_wall_time is not None and now - self.last_ekf_wall_time < 0.75:
            return
        if self.last_odom_wall_time is not None and now - self.last_odom_wall_time < 0.75:
            return
        self.robot_state = self.state_from_odom(msg, "fallback_odom")

    def state_from_odom(self, msg: Odometry, source: str) -> RobotState:
        pose = msg.pose.pose
        twist = msg.twist.twist
        state = RobotState()
        state.x = pose.position.x
        state.y = pose.position.y
        state.z = pose.position.z
        state.yaw = quaternion_to_yaw(pose.orientation)
        state.vx = twist.linear.x
        state.vy = twist.linear.y
        state.yaw_rate = twist.angular.z
        state.stamp = msg.header.stamp
        state.source = source
        state.valid = finite_values(state.x, state.y, state.z, state.yaw, state.vx, state.vy, state.yaw_rate)
        return state

    def tracking_status_callback(self, msg: String):
        self.tracking_status = msg.data
        if msg.data == "arrived":
            self.navigation_progress = 100
        elif msg.data.startswith("error"):
            self.navigation_active = False
        self.update_progress_label()

    def robot_state_is_fresh(self) -> bool:
        if not self.robot_state.valid:
            return False
        receipt_time = None
        if self.robot_state.source == "ekf":
            receipt_time = self.last_ekf_wall_time
        elif self.robot_state.source == "odom":
            receipt_time = self.last_odom_wall_time
        elif self.robot_state.source == "fallback_odom":
            receipt_time = self.last_fallback_odom_wall_time

        if receipt_time is not None:
            return time.monotonic() - receipt_time < 2.0

        stamp = self.robot_state.stamp
        if stamp and not stamp.is_zero():
            try:
                return abs((rospy.Time.now() - stamp).to_sec()) < 2.0
            except Exception:
                return True
        return True

    def refresh_ui(self):
        state = self.robot_state
        valid = self.robot_state_is_fresh()
        self.position_label.setText(f"位置: X: {state.x:.2f}m  Y: {state.y:.2f}m  Z: {state.z:.2f}m")
        self.yaw_label.setText(f"朝向: {math.degrees(state.yaw):.1f}°")
        self.velocity_label.setText(f"速度: Vx: {state.vx:.2f}m/s  Vy: {state.vy:.2f}m/s")
        self.yaw_rate_label.setText(f"角速度: {state.yaw_rate:.2f}rad/s")
        self.valid_label.setText("状态: Valid" if valid else "状态: Invalid")
        self.valid_label.setStyleSheet("color: #15803d;" if valid else "color: #b91c1c;")
        self.tracking_label.setText(f"导航: {self.tracking_status}")
        self.update_robot_item(valid)
        self.update_navigation_progress()
        self.update_progress_label()
        self.update_process_buttons()

    def update_robot_item(self, valid: bool):
        if not valid:
            self.robot_item.setVisible(False)
            return
        self.robot_item.setVisible(True)
        center = self.world_to_scene(self.robot_state.x, self.robot_state.y)
        size = 12.0
        yaw = self.robot_state.yaw
        points_local = [(size, 0.0), (-0.65 * size, 0.55 * size), (-0.65 * size, -0.55 * size)]
        polygon = QPolygonF()
        for lx, ly in points_local:
            wx = lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = lx * math.sin(yaw) + ly * math.cos(yaw)
            polygon.append(QPointF(center.x() + wx, center.y() - wy))
        self.robot_item.setPolygon(polygon)

    def update_navigation_progress(self):
        if not self.navigation_active:
            return
        if not self.robot_state_is_fresh() or len(self.planned_path_world) < 2:
            self.navigation_progress = None
            return
        progress = self.compute_path_progress(
            (self.robot_state.x, self.robot_state.y),
            self.planned_path_world,
        )
        if progress is not None:
            self.navigation_progress = progress

    def compute_path_progress(
        self,
        robot_xy: Tuple[float, float],
        path: List[Tuple[float, float]],
    ) -> Optional[int]:
        if len(path) < 2:
            return None
        segment_lengths = [distance(path[i], path[i + 1]) for i in range(len(path) - 1)]
        total = sum(segment_lengths)
        if total <= 1.0e-6:
            return None

        best_distance = float("inf")
        best_s = 0.0
        accumulated = 0.0
        rx, ry = robot_xy
        for i, seg_len in enumerate(segment_lengths):
            if seg_len <= 1.0e-9:
                continue
            ax, ay = path[i]
            bx, by = path[i + 1]
            vx = bx - ax
            vy = by - ay
            t = ((rx - ax) * vx + (ry - ay) * vy) / (seg_len * seg_len)
            t = max(0.0, min(1.0, t))
            px = ax + t * vx
            py = ay + t * vy
            d = math.hypot(rx - px, ry - py)
            if d < best_distance:
                best_distance = d
                best_s = accumulated + t * seg_len
            accumulated += seg_len

        percent = int(round(100.0 * best_s / total))
        return max(0, min(100, percent))

    def update_progress_label(self):
        value = "--" if self.navigation_progress is None else str(int(self.navigation_progress))
        self.progress_text.setText(f"导航进度: {value}%")
        self.position_overlay_items()

    def plan_coverage_path(self):
        if len(self.coverage_polygon_world) < 3 or not self.coverage_polygon_closed:
            self.show_warning("请先左键点选覆盖区域，并右键闭合多边形")
            return
        if not self.robot_state_is_fresh():
            self.show_warning("当前没有有效机器人状态，无法设置 ENU 原点")
            return

        self.origin_world = (self.robot_state.x, self.robot_state.y, self.robot_state.z)
        polygon_local = [self.world_to_local(x, y) for x, y in self.coverage_polygon_world]
        req = PlanCoveragePathRequest()
        req.header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        req.polygon = [Point(x=x, y=y, z=0.0) for x, y in polygon_local]
        req.use_robot_start = True
        req.robot_start = Pose2D(x=0.0, y=0.0, theta=self.robot_state.yaw)
        req.use_robot_start_heading = True
        req.scan_width = float(self.scan_width_spin.value())
        req.coverage_ratio = float(self.coverage_slider.value()) / 100.0
        req.turn_radius = 0.0
        req.turn_step = 0.0
        req.start_connection_weight = -1.0

        self.plan_button.setEnabled(False)
        self.plan_button.setText("规划中...")
        threading.Thread(target=self.call_planning_service, args=(req,), daemon=True).start()

    def call_planning_service(self, req: PlanCoveragePathRequest):
        try:
            rospy.wait_for_service(self.planning_service, timeout=5.0)
            proxy = rospy.ServiceProxy(self.planning_service, PlanCoveragePath)
            response = proxy(req)
            self.run_on_ui(lambda: self.handle_plan_response(response))
        except Exception as exc:
            self.run_on_ui(lambda: self.handle_plan_error(str(exc)))

    def run_on_ui(self, callback):
        self.dispatcher.callback.emit(callback)

    def handle_plan_response(self, response):
        self.plan_button.setEnabled(True)
        if not response.success:
            self.plan_button.setText("规划")
            self.show_warning(f"规划失败: {response.message}")
            return

        self.planned_path_local = [
            (pose.pose.position.x, pose.pose.position.y) for pose in response.path.poses
        ]
        self.planned_path_world = [self.local_to_world(x, y) for x, y in self.planned_path_local]
        self.planned_path_indices = list(range(len(self.planned_path_world)))
        self.draw_planned_path()
        self.publish_plan_path()
        self.plan_button.setText("重规划")
        self.navigation_progress = None
        self.update_progress_label()

    def handle_plan_error(self, error: str):
        self.plan_button.setEnabled(True)
        self.plan_button.setText("规划")
        self.show_warning(f"规划服务调用失败: {error}")

    def draw_planned_path(self):
        scene_points = [self.world_to_scene(x, y) for x, y in self.planned_path_world]
        self.path_item.setPath(QPainterPathCompat.path_from_points(scene_points, closed=False))

    def toggle_navigation(self):
        if self.navigation_active:
            self.stop_navigation()
        else:
            self.start_navigation()

    def start_navigation(self):
        if len(self.planned_path_world) < 2:
            self.show_warning("请先完成规划")
            return
        self.publish_plan_path()
        self.start_tracker_process()
        self.navigation_active = True
        self.navigation_progress = 0
        self.navigation_button.setText("停止")
        self.navigation_button.setStyleSheet("background-color: #dc2626; color: white;")
        self.update_progress_label()

    def stop_navigation(self):
        self.navigation_active = False
        self.navigation_progress = None
        self.publish_zero_cmd()
        self.stop_process(self.tracker_process)
        self.tracker_process = None
        self.navigation_button.setText("导航")
        self.navigation_button.setStyleSheet("background-color: #2563eb; color: white;")
        self.update_progress_label()

    def publish_plan_path(self):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        for i, (x, y) in enumerate(self.planned_path_world):
            pose = PoseStamped()
            pose.header = path.header
            pose.header.seq = i
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            if i + 1 < len(self.planned_path_world):
                nx, ny = self.planned_path_world[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif i > 0:
                px, py = self.planned_path_world[i - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        self.plan_pub.publish(path)

    def start_tracker_process(self):
        if self.tracker_process and self.tracker_process.poll() is None:
            return
        self.tracker_process = subprocess.Popen(
            self.roslaunch_command(self.tracker_launch, self.tracker_launch_args),
            preexec_fn=os.setsid,
        )

    def publish_zero_cmd(self):
        self.cmd_pub.publish(Twist())

    def toggle_simulation(self):
        if self.sim_process and self.sim_process.poll() is None:
            self.stop_process(self.sim_process)
            self.sim_process = None
            return
        self.sim_process = subprocess.Popen(
            self.roslaunch_command(self.sim_launch, self.sim_launch_args),
            preexec_fn=os.setsid,
        )

    def update_process_buttons(self):
        sim_running = self.sim_process is not None and self.sim_process.poll() is None
        self.sim_button.setText("仿真运行中" if sim_running else "启动仿真")
        if sim_running:
            self.sim_button.setStyleSheet("background-color: #16a34a; color: white; font-weight: bold;")
        else:
            self.sim_button.setStyleSheet("background-color: #dc2626; color: white; font-weight: bold;")

    def stop_process(self, process: Optional[subprocess.Popen]):
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5.0)
        except Exception:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                pass

    def show_warning(self, message: str):
        QMessageBox.warning(self, "提示", message)

    def closeEvent(self, event):
        self.publish_zero_cmd()
        self.stop_process(self.tracker_process)
        self.stop_process(self.sim_process)
        event.accept()


class QPainterPathCompat:
    @staticmethod
    def empty_path():
        from PyQt5.QtGui import QPainterPath

        return QPainterPath()

    @staticmethod
    def path_from_points(points: List[QPointF], closed: bool):
        from PyQt5.QtGui import QPainterPath

        path = QPainterPath()
        if not points:
            return path
        path.moveTo(points[0])
        for point in points[1:]:
            path.lineTo(point)
        if closed:
            path.closeSubpath()
        return path


def main():
    app = QApplication(sys.argv)
    window = CoverageNavigationGui()
    window.show()
    exit_code = app.exec_()
    rospy.signal_shutdown("GUI closed")
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
