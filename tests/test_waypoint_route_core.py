import pathlib
import sys
import tempfile
import unittest

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from waypoint_route_core import Route, Waypoint, route_heading, save_route, load_route, from_dict, filter_unsafe_route


class RouteTests(unittest.TestCase):
    def make_route(self):
        route = Route(map_yaml='/tmp/map.yaml')
        route.add(0, 0)
        route.add(1, 0)
        route.add(1, 1)
        route.update(route.waypoints[-1].id, skippable=False)
        return route

    def test_add_insert_delete_reorder(self):
        route = self.make_route()
        id2 = route.waypoints[1].id
        self.assertTrue(route.shift(id2, 1))
        self.assertEqual(route.waypoints[2].id, id2)
        self.assertFalse(route.shift(id2, 1))
        route.remove(id2)
        self.assertEqual(len(route.waypoints), 2)
        route.add(2, 2)
        route.validate()

    def test_yaw_follows_order(self):
        route = self.make_route()
        self.assertAlmostEqual(route_heading(route, 1), 1.5707963267948966)
        route.shift(route.waypoints[2].id, -1)
        self.assertNotAlmostEqual(route_heading(route, 1), 1.5707963267948966)

    def test_persistence_and_map_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = pathlib.Path(tmp) / 'route.yaml'
            route = self.make_route()
            save_route(file, route)
            result = load_route(file, route.map_yaml)
            self.assertEqual([w.id for w in result.waypoints], [w.id for w in route.waypoints])
            with self.assertRaises(ValueError):
                load_route(file, '/tmp/another_map.yaml')

    def test_protect_map_overwrite(self):
        route = self.make_route()
        with self.assertRaises(ValueError):
            save_route('/tmp/map.yaml', route)

    def test_draft_no_goal_rejected_for_execution(self):
        route = self.make_route()
        route.waypoints[-1].skippable = True
        route.validate()
        with self.assertRaises(ValueError):
            route.validate(require_goals=True)

    def test_invalid_coordinates_and_yaw(self):
        route = self.make_route()
        with self.assertRaises(ValueError):
            route.update(route.waypoints[0].id, x=float('nan'))
        with self.assertRaises(ValueError):
            route.update(route.waypoints[0].id, orientation_required=True)

    def test_duplicate_rejected(self):
        route = self.make_route()
        route.waypoints.append(Waypoint(route.waypoints[0].id, 0, 0))
        with self.assertRaises(ValueError):
            route.validate()

    def test_static_filter_skips_optional_but_not_required(self):
        class FakeGrid:
            def clearance_check(self, x, y, yaw, footprint, clearance, unknown):
                return (False, 'blocked') if x == 1 and y == 0 else (True, 'free')
        route = self.make_route()
        result, skipped = filter_unsafe_route(route, FakeGrid(), ((0, 0),), 0.05)
        self.assertEqual([w.id for w in result.waypoints], [route.waypoints[0].id, route.waypoints[-1].id])
        self.assertEqual(len(skipped), 1)
        route.update(route.waypoints[1].id, skippable=False)
        with self.assertRaises(RuntimeError):
            filter_unsafe_route(route, FakeGrid(), ((0, 0),), 0.05)

    def test_unknown_schema_rejected(self):
        with self.assertRaises(ValueError):
            from_dict({'version': 1, 'map_yaml': '/tmp/map.yaml', 'frame_id': 'map',
                       'waypoints': [{'id': 'wp1', 'x': 0, 'y': 1, 'unexpected': 1}]}, '/tmp/map.yaml')

    def test_integer_coordinates_are_normalized(self):
        data = {
            'version': 1,
            'frame_id': 'map',
            'map_yaml': '/tmp/map.yaml',
            'waypoints': [{
                'id': 'wp001',
                'x': 1,
                'y': 2,
                'yaw': 0,
                'skippable': False,
                'orientation_required': True,
            }],
        }
        route = from_dict(data, '/tmp/map.yaml')
        waypoint = route.waypoints[0]
        self.assertIs(type(waypoint.x), float)
        self.assertIs(type(waypoint.y), float)
        self.assertIs(type(waypoint.yaw), float)
        self.assertEqual((waypoint.x, waypoint.y), (1.0, 2.0))

if __name__ == '__main__':
    unittest.main()
