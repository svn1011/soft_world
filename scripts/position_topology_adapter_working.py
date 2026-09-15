"""Translate known position publishers' interval topology to the active limb.

Only identified controller publishers with a complete contiguous joint topic
set are adapted. Individual/manual joint commands and torque are unchanged.
"""

import re
import time


class PositionTopologyAdapter:
    def __init__(self, node, interval_count):
        self.node = node
        self.interval_count = interval_count
        self._cache = {}
        self._warned = set()

    def scale(self, topic, message_info=None):
        endpoints = self.node.get_publishers_info_by_topic(topic)
        # This project's source-built Humble rclpy delivers only the message,
        # not publisher metadata. Adapt only an unambiguous topic publisher.
        if message_info is None:
            if len(endpoints) != 1:
                if len(endpoints) > 1 and "multiple_publishers" not in self._warned:
                    self.node.get_logger().error(
                        "Multiple position publishers are commanding the same "
                        "joint. Stop duplicate controllers; conversion is disabled.")
                    self._warned.add("multiple_publishers")
                return 1.0
            gid = bytes(endpoints[0].endpoint_gid)
        else:
            gid = bytes(message_info.publisher_gid)
        now = time.monotonic()
        cached = self._cache.get(gid)
        if cached and now < cached[0]:
            return cached[1]
        source = next((endpoint for endpoint in
                       endpoints
                       if bytes(endpoint.endpoint_gid) == gid), None)
        if source is None or source.node_name != "soft_limb_position_controller":
            return 1.0
        identity = (source.node_name, source.node_namespace)

        def belongs(endpoint):
            return (endpoint.node_name, endpoint.node_namespace) == identity

        base = "/soft_limb/pair_00/base/roll/position"
        base_sources = [endpoint for endpoint in
                        self.node.get_publishers_info_by_topic(base)
                        if belongs(endpoint)]
        if len(base_sources) != 1:
            if "ambiguous" not in self._warned:
                self.node.get_logger().error(
                    "Multiple position controllers share a node name. Stop old "
                    "controllers; topology conversion is ambiguous and disabled.")
                self._warned.add("ambiguous")
            return 1.0
        pairs = {}
        for name, _ in self.node.get_topic_names_and_types():
            match = re.fullmatch(
                r"/soft_limb/pair_(\d+)/d6_([ab])/roll/position", name)
            if match and any(belongs(endpoint) for endpoint in
                             self.node.get_publishers_info_by_topic(name)):
                pairs.setdefault(int(match[1]), set()).add(match[2])
        # Both halves of every serial pair must belong to this source.
        complete = {index for index, sides in pairs.items() if sides == {"a", "b"}}
        count = max(complete, default=0) + 1
        factor = 1.0
        if count in (self.interval_count, 11) and complete == set(range(1, count)):
            factor = count / self.interval_count
        if factor != 1.0 and identity not in self._warned:
            self.node.get_logger().warning(
                f"Detected legacy {count + 1}-disc position controller: "
                f"rescaling bend targets by {count}/{self.interval_count}. "
                "180 degrees now refers to the sixth/final disc Disk_05. "
                "Prefer control_soft_limb_working.py.")
            self._warned.add(identity)
        # Retry quickly while discovery may still be completing.
        self._cache[gid] = (now + (1.0 if factor != 1.0 else 0.2), factor)
        return factor
