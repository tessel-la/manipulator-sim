"""ROS2 action server for reusable manipulator motions and YAML routines."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from tf2_ros import TransformException

from manipulator_action_interfaces.action import (
    DetectObject,
    GraspObject,
    MoveEndEffector,
    PlaceObject,
    RunSequence,
)
from manipulator_actions.controller import ServoMotionController
from manipulator_actions.motion_math import PoseTarget, direction_offsets
from manipulator_actions.sequences import (
    MOVE_ABSOLUTE,
    MOVE_RELATIVE,
    WAIT,
    SequenceStep,
    SequenceValidationError,
    load_sequence_file,
    resolve_sequence_file,
    sequence_search_paths,
)
from manipulator_actions.simulated_scene import (
    attached_position,
    attachment_offset,
    placement_position,
)


class ManipulatorActionServer(Node):
    def __init__(self) -> None:
        super().__init__("manipulator_action_server")
        self._callback_group = ReentrantCallbackGroup()

        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("ee_frame", "panda_link8")
        self.declare_parameter("twist_topic", "servo_node/delta_twist_cmds")
        self.declare_parameter("pause_servo_service", "servo_node/pause_servo")
        self.declare_parameter(
            "switch_command_type_service", "servo_node/switch_command_type"
        )
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("timeout", 10.0)
        self.declare_parameter("tf_timeout", 5.0)
        self.declare_parameter("sequence_directories", "")
        self.declare_parameter("scene_topic", "/pick_place_scene/objects")
        self.declare_parameter("scene_fixed_frame", "world")
        self.declare_parameter(
            "set_entity_pose_service", "/world/default/set_pose"
        )
        self.declare_parameter("held_object_update_rate", 10.0)
        self.declare_parameter("grasp_hover_height", 0.25)
        self.declare_parameter("grasp_approach_height", 0.13)
        self.declare_parameter("place_hover_height", 0.25)
        self.declare_parameter("place_release_height", 0.13)
        self.declare_parameter("lift_height", 0.25)
        self.declare_parameter("gripper_pause", 0.5)

        self._controller = ServoMotionController(
            node=self,
            base_frame=self.get_parameter("base_frame").value,
            ee_frame=self.get_parameter("ee_frame").value,
            twist_topic=self.get_parameter("twist_topic").value,
            pause_servo_service=self.get_parameter("pause_servo_service").value,
            switch_command_type_service=self.get_parameter(
                "switch_command_type_service"
            ).value,
            tf_timeout_sec=float(self.get_parameter("tf_timeout").value),
        )
        self._scene_objects: List[Dict] = []
        self._held_object: Optional[Dict] = None
        self._held_object_offset: Optional[tuple[float, float, float]] = None
        self._held_object_orientation = None
        self._object_world_poses: Dict[str, Pose] = {}
        self._held_object_lock = threading.RLock()
        self._warned_about_set_pose = False
        self._set_pose_client = self.create_client(
            SetEntityPose,
            str(self.get_parameter("set_entity_pose_service").value),
            callback_group=self._callback_group,
        )
        held_object_update_rate = max(
            float(self.get_parameter("held_object_update_rate").value),
            1.0,
        )
        self._held_object_timer = self.create_timer(
            1.0 / held_object_update_rate,
            self._update_held_object_pose,
            callback_group=self._callback_group,
        )
        scene_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._scene_sub = self.create_subscription(
            String,
            str(self.get_parameter("scene_topic").value),
            self._on_scene,
            scene_qos,
            callback_group=self._callback_group,
        )

        self._move_server = ActionServer(
            self,
            MoveEndEffector,
            "move_end_effector",
            self._execute_move,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._sequence_server = ActionServer(
            self,
            RunSequence,
            "run_sequence",
            self._execute_sequence,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._detect_server = ActionServer(
            self,
            DetectObject,
            "detect_object",
            self._execute_detect,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._grasp_server = ActionServer(
            self,
            GraspObject,
            "grasp_object",
            self._execute_grasp,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self._place_server = ActionServer(
            self,
            PlaceObject,
            "place_object",
            self._execute_place,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info("Manipulator action server ready")

    def destroy_node(self) -> bool:
        self._controller.destroy()
        return super().destroy_node()

    def _cancel_callback(self, _cancel_request):
        return CancelResponse.ACCEPT

    @property
    def _default_position_tolerance(self) -> float:
        return float(self.get_parameter("position_tolerance").value)

    @property
    def _default_yaw_tolerance(self) -> float:
        return float(self.get_parameter("yaw_tolerance").value)

    @property
    def _default_timeout(self) -> float:
        return float(self.get_parameter("timeout").value)

    def _execute_move(self, goal_handle):
        goal = goal_handle.request
        position_tolerance = (
            goal.position_tolerance
            if goal.position_tolerance > 0.0
            else self._default_position_tolerance
        )
        yaw_tolerance = (
            goal.yaw_tolerance
            if goal.yaw_tolerance > 0.0
            else self._default_yaw_tolerance
        )
        timeout = goal.timeout if goal.timeout > 0.0 else self._default_timeout

        def publish_feedback(distance: float, yaw_error: float, state: str) -> None:
            feedback = MoveEndEffector.Feedback()
            feedback.remaining_distance = distance
            feedback.remaining_yaw = yaw_error
            feedback.state = state
            goal_handle.publish_feedback(feedback)

        if not goal.relative:
            success, message, final_pose = self._controller.move_absolute(
                target=PoseTarget(goal.x, goal.y, goal.z, goal.yaw),
                position_tolerance=position_tolerance,
                yaw_tolerance=yaw_tolerance,
                timeout_sec=timeout,
                feedback=publish_feedback,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )
        else:
            success, message, final_pose = self._controller.move_relative(
                dx=goal.x,
                dy=goal.y,
                dz=goal.z,
                dyaw=goal.yaw,
                position_tolerance=position_tolerance,
                yaw_tolerance=yaw_tolerance,
                timeout_sec=timeout,
                feedback=publish_feedback,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )

        result = self._move_result(success, message, final_pose)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _execute_sequence(self, goal_handle):
        name = goal_handle.request.name
        result = RunSequence.Result()
        completed_steps = 0

        try:
            sequence_file = resolve_sequence_file(name, self._sequence_paths())
            steps = load_sequence_file(sequence_file)
        except (FileNotFoundError, SequenceValidationError) as exc:
            result.success = False
            result.message = str(exc)
            result.completed_steps = 0
            goal_handle.abort()
            return result

        self.get_logger().info(f"Running sequence '{name}' with {len(steps)} step(s)")
        for index, step in enumerate(steps, start=1):
            if goal_handle.is_cancel_requested:
                self._controller.halt()
                result.success = False
                result.message = "Sequence canceled"
                result.completed_steps = completed_steps
                goal_handle.canceled()
                return result

            feedback = RunSequence.Feedback()
            feedback.current_step = index
            feedback.current_action = step.kind
            goal_handle.publish_feedback(feedback)

            success, message = self._run_step(
                step, cancel_requested=lambda: goal_handle.is_cancel_requested
            )
            if not success:
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = "Sequence canceled"
                    result.completed_steps = completed_steps
                    goal_handle.canceled()
                    return result
                result.success = False
                result.message = f"Step {index} ({step.kind}) failed: {message}"
                result.completed_steps = completed_steps
                goal_handle.abort()
                return result
            completed_steps += 1

        result.success = True
        result.message = f"Sequence '{name}' completed"
        result.completed_steps = completed_steps
        goal_handle.succeed()
        return result

    def _execute_detect(self, goal_handle):
        goal = goal_handle.request
        result = DetectObject.Result()
        timeout = goal.timeout if goal.timeout > 0.0 else self._default_timeout
        target_frame = self._target_frame(goal.target_frame)
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = "Detection canceled"
                goal_handle.canceled()
                return result

            feedback = DetectObject.Feedback()
            feedback.state = "resolving scene object"
            goal_handle.publish_feedback(feedback)

            scene_object = self._resolve_scene_object(goal.query, goal.kind)
            if scene_object is None and goal.query:
                scene_object = {
                    "name": goal.query,
                    "frame_id": goal.query,
                    "kind": goal.kind or "unknown",
                }

            if scene_object is not None:
                pose, message = self._pose_for_object(scene_object, target_frame)
                if pose is not None:
                    result.success = True
                    result.message = message
                    result.name = str(scene_object.get("name") or scene_object.get("frame_id") or "")
                    result.kind = str(scene_object.get("kind") or "")
                    result.object_frame = str(scene_object.get("frame_id") or result.name)
                    result.pose = pose
                    result.confidence = 1.0
                    goal_handle.succeed()
                    return result

            time.sleep(0.05)

        result.success = False
        result.message = (
            f"Object '{goal.query or goal.kind}' was not detected before timeout"
        )
        result.confidence = 0.0
        goal_handle.abort()
        return result

    def _execute_grasp(self, goal_handle):
        goal = goal_handle.request
        result = GraspObject.Result()
        timeout = goal.timeout if goal.timeout > 0.0 else self._default_timeout
        hover_height = self._positive_or_default(goal.hover_height, "grasp_hover_height")
        approach_height = self._positive_or_default(
            goal.approach_height, "grasp_approach_height"
        )
        lift_height = self._positive_or_default(goal.lift_height, "lift_height")

        scene_object = self._resolve_scene_object(goal.object_id, "cube")
        if scene_object is None:
            scene_object = self._resolve_scene_object(goal.object_id, "")
        if scene_object is None:
            result.success = False
            result.message = f"Cannot grasp unknown object '{goal.object_id}'"
            goal_handle.abort()
            return result

        pose, message = self._pose_for_object(scene_object, self._controller.base_frame)
        if pose is None:
            result.success = False
            result.message = message
            goal_handle.abort()
            return result

        target = self._pose_target_from_pose(pose)
        result.grasp_pose = pose
        result.object_id = str(scene_object.get("name") or goal.object_id)
        result.object_frame = str(scene_object.get("frame_id") or result.object_id)

        steps = [
            ("moving to pregrasp", PoseTarget(target.x, target.y, target.z + hover_height, target.yaw)),
            ("approaching object", PoseTarget(target.x, target.y, target.z + approach_height, target.yaw)),
            ("lifting object", PoseTarget(target.x, target.y, target.z + lift_height, target.yaw)),
        ]
        success, status = self._run_pose_steps(
            goal_handle,
            GraspObject.Feedback,
            steps,
            timeout,
            close_pause=True,
            scene_object=scene_object,
        )
        if not success:
            result.success = False
            result.message = status
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        result.success = True
        result.message = f"Grasp completed for '{result.object_id}'"
        goal_handle.succeed()
        return result

    def _execute_place(self, goal_handle):
        goal = goal_handle.request
        result = PlaceObject.Result()
        timeout = goal.timeout if goal.timeout > 0.0 else self._default_timeout
        hover_height = self._positive_or_default(goal.hover_height, "place_hover_height")
        release_height = self._positive_or_default(
            goal.release_height, "place_release_height"
        )
        lift_height = self._positive_or_default(goal.lift_height, "lift_height")

        target_object = self._resolve_scene_object(goal.target_id, "place_target")
        if target_object is None:
            target_object = self._resolve_scene_object(goal.target_id, "")
        if target_object is None:
            result.success = False
            result.message = f"Cannot place on unknown target '{goal.target_id}'"
            goal_handle.abort()
            return result

        held_name = goal.object_id or (
            str(self._held_object.get("name") or self._held_object.get("frame_id"))
            if self._held_object
            else ""
        )
        pose, message = self._pose_for_object(target_object, self._controller.base_frame)
        if pose is None:
            result.success = False
            result.message = message
            goal_handle.abort()
            return result

        target = self._pose_target_from_pose(pose)
        result.place_pose = pose
        result.object_id = held_name
        result.target_id = str(target_object.get("name") or goal.target_id)

        steps = [
            ("moving over place target", PoseTarget(target.x, target.y, target.z + hover_height, target.yaw)),
            ("lowering to release", PoseTarget(target.x, target.y, target.z + release_height, target.yaw)),
            ("retreating after release", PoseTarget(target.x, target.y, target.z + lift_height, target.yaw)),
        ]
        success, status = self._run_pose_steps(
            goal_handle,
            PlaceObject.Feedback,
            steps,
            timeout,
            close_pause=False,
            scene_object=target_object,
        )
        if not success:
            result.success = False
            result.message = status
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result

        result.success = True
        result.message = f"Place completed on '{result.target_id}'"
        goal_handle.succeed()
        return result

    def _run_step(self, step: SequenceStep, cancel_requested) -> tuple[bool, str]:
        if step.kind == WAIT:
            deadline = time.monotonic() + step.params["seconds"]
            while time.monotonic() < deadline:
                if cancel_requested():
                    return False, "Sequence canceled"
                time.sleep(0.05)
            return True, "Wait completed"

        if step.kind == MOVE_ABSOLUTE:
            success, message, _ = self._controller.move_absolute(
                target=PoseTarget(
                    step.params["x"],
                    step.params["y"],
                    step.params["z"],
                    step.params["yaw"],
                ),
                position_tolerance=self._default_position_tolerance,
                yaw_tolerance=self._default_yaw_tolerance,
                timeout_sec=self._default_timeout,
                cancel_requested=cancel_requested,
            )
            return success, message

        if step.kind == MOVE_RELATIVE:
            directions = {
                name: amount
                for name, amount in step.params.items()
                if name in {"forward", "back", "left", "right", "up", "down"}
            }
            dx, dy, dz = direction_offsets(directions)
            success, message, _ = self._controller.move_relative(
                dx=dx,
                dy=dy,
                dz=dz,
                dyaw=step.params.get("yaw", 0.0),
                position_tolerance=self._default_position_tolerance,
                yaw_tolerance=self._default_yaw_tolerance,
                timeout_sec=self._default_timeout,
                cancel_requested=cancel_requested,
            )
            return success, message

        return False, f"Unsupported step kind '{step.kind}'"

    def _run_pose_steps(
        self,
        goal_handle,
        feedback_type,
        steps: List[tuple[str, PoseTarget]],
        timeout: float,
        close_pause: bool,
        scene_object: Dict,
    ) -> tuple[bool, str]:
        per_motion_timeout = max(timeout / max(len(steps), 1), 1.0)
        for index, (state, target) in enumerate(steps):
            if goal_handle.is_cancel_requested:
                self._controller.halt()
                return False, "Action canceled"

            feedback = feedback_type()
            feedback.state = state
            goal_handle.publish_feedback(feedback)

            if close_pause and index == 2:
                pause_feedback = feedback_type()
                pause_feedback.state = "closing simulated gripper"
                goal_handle.publish_feedback(pause_feedback)
                self._sleep_with_cancel(
                    float(self.get_parameter("gripper_pause").value),
                    goal_handle,
                )
                if goal_handle.is_cancel_requested:
                    return False, "Action canceled"
                attached, message = self._attach_scene_object(scene_object)
                if not attached:
                    return False, message
            elif not close_pause and index == 2:
                pause_feedback = feedback_type()
                pause_feedback.state = "opening simulated gripper"
                goal_handle.publish_feedback(pause_feedback)
                self._sleep_with_cancel(
                    float(self.get_parameter("gripper_pause").value),
                    goal_handle,
                )
                if goal_handle.is_cancel_requested:
                    return False, "Action canceled"
                released, message = self._release_scene_object(scene_object)
                if not released:
                    return False, message

            success, message, _ = self._controller.move_absolute(
                target=target,
                position_tolerance=self._default_position_tolerance,
                yaw_tolerance=self._default_yaw_tolerance,
                timeout_sec=per_motion_timeout,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
            )
            if not success:
                return False, f"{state} failed: {message}"

        return True, "completed"

    @property
    def _scene_fixed_frame(self) -> str:
        return str(self.get_parameter("scene_fixed_frame").value).strip().lstrip(
            "/"
        ) or "world"

    def _attach_scene_object(self, scene_object: Dict) -> tuple[bool, str]:
        object_frame = str(
            scene_object.get("frame_id") or scene_object.get("name") or ""
        )
        if not object_frame:
            return False, "Cannot attach a scene object without a frame"

        try:
            object_pose = self._world_pose_for_object(scene_object)
            ee_transform = self._controller.lookup_transform(
                self._scene_fixed_frame,
                self._controller.ee_frame,
            )
        except TransformException as exc:
            return False, f"Could not attach '{object_frame}': {exc}"

        object_position = self._position_tuple(object_pose)
        ee_position = self._translation_tuple(ee_transform)
        with self._held_object_lock:
            self._held_object = scene_object
            self._held_object_offset = attachment_offset(
                object_position,
                ee_position,
            )
            self._held_object_orientation = object_pose.orientation
            self._object_world_poses[self._object_key(scene_object)] = object_pose

        self._set_gazebo_entity_pose(scene_object, object_pose)
        return True, f"Attached '{scene_object.get('name', object_frame)}'"

    def _release_scene_object(self, target_object: Dict) -> tuple[bool, str]:
        with self._held_object_lock:
            held_object = self._held_object
            if held_object is None:
                return False, "Cannot place because no object is held"

            try:
                target_pose = self._world_pose_for_object(target_object)
            except TransformException as exc:
                return False, f"Could not resolve place target: {exc}"

            position = placement_position(
                self._position_tuple(target_pose),
                float(held_object.get("size", 0.0)),
                float(target_object.get("height", 0.0)),
            )
            released_pose = Pose()
            released_pose.position.x = position[0]
            released_pose.position.y = position[1]
            released_pose.position.z = position[2]
            if self._held_object_orientation is not None:
                released_pose.orientation = self._held_object_orientation
            else:
                released_pose.orientation = target_pose.orientation

            self._set_gazebo_entity_pose(held_object, released_pose)
            self._object_world_poses[
                self._object_key(held_object)
            ] = released_pose
            self._held_object = None
            self._held_object_offset = None
            self._held_object_orientation = None

        return True, f"Released object on '{target_object.get('name', 'target')}'"

    def _update_held_object_pose(self) -> None:
        with self._held_object_lock:
            if self._held_object is None or self._held_object_offset is None:
                return

            try:
                ee_transform = self._controller.lookup_transform(
                    self._scene_fixed_frame,
                    self._controller.ee_frame,
                )
            except TransformException:
                return

            position = attached_position(
                self._translation_tuple(ee_transform),
                self._held_object_offset,
            )
            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            if self._held_object_orientation is not None:
                pose.orientation = self._held_object_orientation
            else:
                pose.orientation.w = 1.0

            self._set_gazebo_entity_pose(self._held_object, pose)
            self._object_world_poses[
                self._object_key(self._held_object)
            ] = pose

    def _set_gazebo_entity_pose(
        self,
        scene_object: Dict,
        pose: Pose,
    ) -> bool:
        if not self._set_pose_client.service_is_ready():
            if not self._warned_about_set_pose:
                self.get_logger().warning(
                    "Gazebo set-pose service is unavailable; grasp actions "
                    "will run without camera-visible object motion"
                )
                self._warned_about_set_pose = True
            return False

        entity_name = str(scene_object.get("name") or "").strip()
        if not entity_name:
            return False

        request = SetEntityPose.Request()
        request.entity.name = entity_name
        request.entity.type = Entity.MODEL
        request.pose = pose
        self._set_pose_client.call_async(request)
        return True

    def _world_pose_for_object(self, scene_object: Dict) -> Pose:
        cached_pose = self._object_world_poses.get(self._object_key(scene_object))
        if cached_pose is not None:
            return cached_pose

        object_frame = str(
            scene_object.get("frame_id") or scene_object.get("name") or ""
        )
        transform = self._controller.lookup_transform(
            self._scene_fixed_frame,
            object_frame,
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _object_key(scene_object: Dict) -> str:
        return str(
            scene_object.get("name") or scene_object.get("frame_id") or ""
        )

    @staticmethod
    def _position_tuple(pose: Pose) -> tuple[float, float, float]:
        return pose.position.x, pose.position.y, pose.position.z

    @staticmethod
    def _translation_tuple(transform) -> tuple[float, float, float]:
        translation = transform.transform.translation
        return translation.x, translation.y, translation.z

    def _sleep_with_cancel(self, seconds: float, goal_handle) -> None:
        deadline = time.monotonic() + max(seconds, 0.0)
        while time.monotonic() < deadline and not goal_handle.is_cancel_requested:
            time.sleep(0.05)

    def _on_scene(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Ignoring malformed scene payload: {exc}")
            return

        objects: List[Dict] = []
        for key, kind in (
            ("cubes", "cube"),
            ("place_targets", "place_target"),
            ("charuco_boards", "charuco_board"),
        ):
            for item in payload.get(key, []) or []:
                if isinstance(item, dict):
                    scene_object = dict(item)
                    scene_object["kind"] = kind
                    objects.append(scene_object)
        self._scene_objects = objects

    def _resolve_scene_object(self, query: str, kind: str) -> Optional[Dict]:
        query = str(query or "").strip()
        kind = str(kind or "").strip()
        for scene_object in self._scene_objects:
            if kind and scene_object.get("kind") != kind:
                continue
            if not query:
                return scene_object
            names = {
                str(scene_object.get("name", "")),
                str(scene_object.get("frame_id", "")),
            }
            if query in names:
                return scene_object
        return None

    def _pose_for_object(
        self, scene_object: Dict, target_frame: str
    ) -> tuple[Optional[PoseStamped], str]:
        object_frame = str(scene_object.get("frame_id") or scene_object.get("name") or "")
        if not object_frame:
            return None, "Detected object has no frame_id"

        with self._held_object_lock:
            cached_pose = self._object_world_poses.get(
                self._object_key(scene_object)
            )
        if cached_pose is not None:
            try:
                pose = self._transform_world_pose(cached_pose, target_frame)
            except TransformException as exc:
                return (
                    None,
                    f"Could not transform '{object_frame}' to "
                    f"'{target_frame}': {exc}",
                )
            return pose, f"Detected '{scene_object.get('name', object_frame)}'"

        try:
            transform = self._controller.lookup_transform(target_frame, object_frame)
        except TransformException as exc:
            return None, f"Could not transform '{object_frame}' to '{target_frame}': {exc}"

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = target_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose, f"Detected '{scene_object.get('name', object_frame)}'"

    def _transform_world_pose(
        self,
        world_pose: Pose,
        target_frame: str,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = target_frame
        if target_frame == self._scene_fixed_frame:
            pose.pose = world_pose
            return pose

        transform = self._controller.lookup_transform(
            target_frame,
            self._scene_fixed_frame,
        ).transform
        transform_quaternion = (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        rotated_position = ServoMotionController._rotate_vector(
            transform_quaternion,
            self._position_tuple(world_pose),
        )
        pose.pose.position.x = rotated_position[0] + transform.translation.x
        pose.pose.position.y = rotated_position[1] + transform.translation.y
        pose.pose.position.z = rotated_position[2] + transform.translation.z

        object_quaternion = (
            world_pose.orientation.x,
            world_pose.orientation.y,
            world_pose.orientation.z,
            world_pose.orientation.w,
        )
        qx, qy, qz, qw = ServoMotionController._quaternion_multiply(
            transform_quaternion,
            object_quaternion,
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _pose_target_from_pose(self, pose: PoseStamped) -> PoseTarget:
        return self._controller.target_from_pose_stamped(pose)

    def _target_frame(self, requested: str) -> str:
        frame = str(requested or "").strip().lstrip("/")
        return frame or self._controller.base_frame

    def _positive_or_default(self, value: float, parameter_name: str) -> float:
        return float(value) if value > 0.0 else float(self.get_parameter(parameter_name).value)

    def _sequence_paths(self) -> List[Path]:
        package_share = get_package_share_directory("manipulator_actions")
        paths: List[Path] = sequence_search_paths(package_share)
        configured = str(self.get_parameter("sequence_directories").value)
        paths.extend(Path(path) for path in configured.split(":") if path)
        return paths

    @staticmethod
    def _move_result(success: bool, message: str, final_pose: PoseTarget):
        result = MoveEndEffector.Result()
        result.success = success
        result.message = message
        result.final_x = final_pose.x
        result.final_y = final_pose.y
        result.final_z = final_pose.z
        result.final_yaw = final_pose.yaw
        return result


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManipulatorActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
