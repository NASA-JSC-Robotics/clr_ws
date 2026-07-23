#!/usr/bin/env python3
import os
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from imetro_behavior.executor import BehaviorTreeExecutor


def main(args=None):
    rclpy.init(args=args)

    pkg_share = get_package_share_directory("clr_connector_mating_demo")
    config_path = os.path.join(pkg_share, "config", "clr_connector_mating_config.yaml")

    node = Node("run_behavior", parameter_overrides=[rclpy.parameter.Parameter("behavior_config", value=config_path)])

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    bt_exec = BehaviorTreeExecutor(node)  # noqa

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
