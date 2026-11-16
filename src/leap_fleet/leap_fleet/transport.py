"""ROS transport, imported lazily so offline configuration needs no ROS setup."""

import uuid


class RosTransport:
    def __init__(self, cars):
        self.cars = cars
        self._resources = []
        self._publishers = {}
        self._twist_type = None

    def open(self):
        from geometry_msgs.msg import Twist
        from rclpy.context import Context
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self._twist_type = Twist
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        suffix = uuid.uuid4().hex[:8]
        for car in self.cars:
            context = Context()
            resource = [context, None, None]
            self._resources.append(resource)
            context.init(args=[], domain_id=car.domain_id)
            node = Node(f'leap_drive_{car.name}_{suffix}', context=context,
                        use_global_arguments=False, start_parameter_services=False,
                        enable_rosout=False)
            resource[1] = node
            publisher = node.create_publisher(Twist, car.cmd_vel_topic, qos)
            resource[2] = publisher
            self._publishers[car.name] = publisher

    def publish(self, car_id, velocity):
        message = self._twist_type()
        message.linear.x = velocity.v
        message.angular.z = velocity.w
        self._publishers[car_id].publish(message)

    def subscriber_count(self, car_id):
        return self._publishers[car_id].get_subscription_count()

    def close(self):
        from rclpy.duration import Duration

        errors = []
        interruption = None
        # A bounded DDS flush before destroying publishers improves delivery of
        # final zero commands; it is not a physical-stop acknowledgement.
        for car, (context, node, publisher) in zip(self.cars, self._resources):
            if publisher is not None:
                try:
                    acknowledged = publisher.wait_for_all_acked(Duration(seconds=0.1))
                    if not acknowledged:
                        errors.append(f'{car.name}: DDS ACK timed out after 0.1 s '
                                      '(not a vehicle execution ACK)')
                except BaseException as error:
                    if isinstance(error, Exception):
                        errors.append(str(error))
                    else:
                        interruption = interruption or error
        for context, node, publisher in reversed(self._resources):
            if node is not None:
                try:
                    node.destroy_node()
                except BaseException as error:
                    if isinstance(error, Exception):
                        errors.append(str(error))
                    else:
                        interruption = interruption or error
            try:
                context.try_shutdown()
            except BaseException as error:
                if isinstance(error, Exception):
                    errors.append(str(error))
                else:
                    interruption = interruption or error
        self._resources.clear()
        self._publishers.clear()
        if interruption is not None:
            raise interruption
        if errors:
            raise RuntimeError('ROS cleanup failed: ' + '; '.join(errors))
