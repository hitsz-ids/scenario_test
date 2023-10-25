#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Sudden right lane change scenario:

The scenario realizes a driving behavior on the highway.
The user-controlled ego vehicle is driving straight.
Another car is driving on right lane, and it suddenly changes to the lane ego vehicle is driving on.
The ego vehicle may need to brake to avoid a collision.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      LaneChange,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenarios.basic_scenario import BasicScenario


class SuddenRightLaneChange(BasicScenario):
    """
    The ego vehicle is driving on a highway and another car is cutting in just in front.
    This is a single ego vehicle scenario
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._velocity = 80
        self._trigger_distance = 20

        # get direction from config name
        self._config = config

        super(SuddenRightLaneChange, self).__init__("SuddenRightLaneChange",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - lane_change: change the lane
        - continue_driving: continue driving on the highway
        - end_condition: drive for a defined distance or stop for 1 second
        """

        # car_visible
        behaviour = py_trees.composites.Sequence("CarOnRightLane")
        car_visible = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        behaviour.add_child(car_visible)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[0], self._velocity)
        just_drive.add_child(car_driving)

        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        just_drive.add_child(trigger_distance)
        behaviour.add_child(just_drive)

        # lane_change
        lane_change = LaneChange(self.other_actors[0], direction='left')
        behaviour.add_child(lane_change)

        # continue driving
        continue_driving = WaypointFollower(self.other_actors[0], self._velocity)
        behaviour.add_child(continue_driving)

        # end_condition
        end_condition = py_trees.composites.Parallel("Waiting for end",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition1 = DriveDistance(self.other_actors[0], 200)
        end_condition2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        end_condition.add_child(end_condition1)
        end_condition.add_child(end_condition2)

        # build tree
        root = py_trees.composites.Parallel("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(behaviour)
        root.add_child(end_condition)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria is created, which is later used in the parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors after deletion.
        """
        self.remove_all_actors()
