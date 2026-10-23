"""Check MoveIt collision detection without commanding robot motion."""
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

PROBE_ID = 'collision_probe'


def main():
    rclpy.init()
    node = rclpy.create_node('collision_probe')
    clients = {}
    cleanup_needed = False

    def call(service_type, name, request):
        if name not in clients:
            clients[name] = node.create_client(service_type, name)
        client = clients[name]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'Service unavailable: {name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            future.cancel()
            raise RuntimeError(f'Service timed out: {name}')
        return future.result()

    def apply_object(obj):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [obj]
        response = call(ApplyPlanningScene, '/apply_planning_scene', request)
        if not response.success:
            raise RuntimeError('MoveIt rejected the scene update')

    try:
        # Take one state snapshot. Reuse it for every comparison.
        received = []
        subscription = node.create_subscription(
            JointState, '/joint_states', received.append,
            qos_profile_sensor_data)
        deadline = time.monotonic() + 10.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_subscription(subscription)
        if not received:
            raise RuntimeError('No /joint_states received')

        state = RobotState()
        state.joint_state = received[-1]
        state.is_diff = True

        def check():
            request = GetStateValidity.Request()
            request.robot_state = state
            # Empty group checks the whole robot.
            request.group_name = ''
            return call(GetStateValidity, '/check_state_validity', request)

        baseline = check()
        print(f'BASELINE valid={baseline.valid}', flush=True)
        if not baseline.valid:
            for contact in baseline.contacts:
                print(f'  {contact.contact_body_1} <-> {contact.contact_body_2}')
            raise RuntimeError('Initial state is invalid; resolve this first')

        # Forward kinematics: compute tool0 position from the joint snapshot.
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = 'world'
        fk_request.fk_link_names = ['tool0']
        fk_request.robot_state = state
        fk = call(GetPositionFK, '/compute_fk', fk_request)
        if fk.error_code.val != 1 or not fk.pose_stamped:
            raise RuntimeError(f'Forward kinematics failed: {fk.error_code.val}')

        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = PROBE_ID
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.30, 0.30, 0.30]
        pose = Pose()
        pose.position = fk.pose_stamped[0].pose.position
        pose.orientation.w = 1.0
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        cleanup_needed = True
        apply_object(obj)

        collision = check()
        print(f'WITH PROBE valid={collision.valid}', flush=True)
        probe_contact = False
        for contact in collision.contacts:
            print(f'  {contact.contact_body_1} <-> {contact.contact_body_2}')
            if PROBE_ID in (contact.contact_body_1, contact.contact_body_2):
                probe_contact = True
        if collision.valid or not probe_contact:
            raise RuntimeError('Expected contact with collision_probe was not confirmed')
        print('PASS: MoveIt detected the temporary obstacle.', flush=True)

    finally:
        try:
            if cleanup_needed:
                removal = CollisionObject()
                removal.header.frame_id = 'world'
                removal.id = PROBE_ID
                removal.operation = CollisionObject.REMOVE
                apply_object(removal)
                print('Temporary obstacle removed.', flush=True)
                restored = check()
                print(f'RESTORED valid={restored.valid}', flush=True)
                if not restored.valid:
                    raise RuntimeError('State is still invalid after cleanup')
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
