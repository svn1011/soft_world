"""Offline regressions for six-disc position command compatibility."""

import unittest
from types import SimpleNamespace

from position_topology_adapter_working import PositionTopologyAdapter

BASE = "/soft_limb/pair_00/base/roll/position"


class Graph:
    def __init__(self, intervals, node_name="soft_limb_position_controller"):
        self.messages = []
        self.topics = {BASE: [self.endpoint(0, node_name)]}
        for index in range(1, intervals):
            for side_index, side in enumerate(("a", "b")):
                topic = f"/soft_limb/pair_{index:02d}/d6_{side}/roll/position"
                self.topics[topic] = [self.endpoint(index * 2 + side_index, node_name)]

    @staticmethod
    def endpoint(gid, name):
        return SimpleNamespace(endpoint_gid=bytes([gid]), node_name=name,
                               node_namespace="/")

    def get_publishers_info_by_topic(self, topic):
        return self.topics.get(topic, [])

    def get_topic_names_and_types(self):
        return [(topic, ["std_msgs/msg/Float64"]) for topic in self.topics]

    def get_logger(self):
        return SimpleNamespace(warning=self.messages.append, error=self.messages.append)


class TopologyTests(unittest.TestCase):
    def scale(self, graph):
        return PositionTopologyAdapter(graph, 5).scale(
            BASE, SimpleNamespace(publisher_gid=b"\x00"))

    def test_legacy_180_reaches_final_disc(self):
        graph = Graph(11)
        factor = self.scale(graph)
        self.assertEqual(factor, 11 / 5)
        self.assertAlmostEqual((180 / 11) * factor * 5, 180)
        self.assertEqual(len(graph.messages), 1)

    def test_working_not_scaled_again(self):
        self.assertEqual(self.scale(Graph(5)), 1)

    def test_humble_message_only_callback(self):
        self.assertEqual(PositionTopologyAdapter(Graph(11), 5).scale(BASE), 11 / 5)

    def test_manual_joint_not_scaled(self):
        self.assertEqual(self.scale(Graph(11, "manual_publisher")), 1)

    def test_incomplete_discovery_not_guessed(self):
        graph = Graph(11)
        del graph.topics["/soft_limb/pair_06/d6_a/roll/position"]
        self.assertEqual(self.scale(graph), 1)

    def test_duplicate_controllers_not_guessed(self):
        graph = Graph(11)
        graph.topics[BASE].append(Graph.endpoint(99, "soft_limb_position_controller"))
        self.assertEqual(self.scale(graph), 1)
        self.assertIn("Multiple position controllers", graph.messages[0])


if __name__ == "__main__":
    unittest.main()
