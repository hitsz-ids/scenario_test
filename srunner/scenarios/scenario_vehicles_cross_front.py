#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Vehicles cross front scenario:

The scenario realizes a common driving behavior, in which the user-controlled ego vehicle drives on the street.
When the ego vehicle reaches the intersection, a tesla suddenly drives from right side at a high speed,
and a bicycle will also ride from left side farther.
The ego vehicle has to react accordingly to avoid a collision.
The scenario ends either if the ego vehicle stops, or if it keeps driving for a defined distance.
"""


import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      KeepVelocity)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import StandStill
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario


class VehiclesCrossFront(BasicScenario):
    """
    This is a single ego vehicle scenario
    """

    timeout = 120  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._ego_distance = 80  # defined distance that the ego vehicle is excepted to drive for
        self._tesla_speed = 20
        self._bicycle_speed = 6

        super(VehiclesCrossFront, self).__init__("VehiclesCrossFront",
                                                 ego_vehicles,
                                                 config,
                                                 world,
                                                 debug_mode,
                                                 criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        self._first_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        self._second_transform = config.other_actors[1].transform
        second_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[1].transform.location.x,
                           config.other_actors[1].transform.location.y,
                           config.other_actors[1].transform.location.z - 500),
            config.other_actors[1].transform.rotation)
        second_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[1].model, second_vehicle_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)

    def _create_behavior(self):
        """
        Order of Sequence:
        - tesla_keep_driving: tesla keeps driving and ignore ego vehicle
        - bicycle_keep_riding: bicycle keeps riding and ignore ego vehicle
        - end_condition: drive for a defined distance or stop for 1 second
        """

        # tesla driving
        tesla_keep_driving = KeepVelocity(self.other_actors[0], self._tesla_speed)

        # bicycle riding
        bicycle_keep_riding = KeepVelocity(self.other_actors[1], self._bicycle_speed)

        # end condition
        end_condition = py_trees.composites.Parallel("Waiting for end",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition_part1 = DriveDistance(self.ego_vehicles[0], self._ego_distance)
        end_condition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        end_condition.add_child(end_condition_part1)
        end_condition.add_child(end_condition_part2)

        # Build behavior tree
        root = py_trees.composites.Sequence()
        root.add_child(ActorTransformSetter(self.other_actors[0], self._first_transform))
        root.add_child(ActorTransformSetter(self.other_actors[1], self._second_transform))

        sequence = py_trees.composites.Parallel("Parallel Behavior",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sequence.add_child(tesla_keep_driving)
        sequence.add_child(bicycle_keep_riding)
        sequence.add_child(end_condition)

        root.add_child(sequence)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
