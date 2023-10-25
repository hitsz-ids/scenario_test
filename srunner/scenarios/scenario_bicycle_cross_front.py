#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Bicycle cross front scenario:

The scenario realizes a common driving behavior, in which the user-controlled ego vehicle drives on the street.
When the ego vehicle reaches the intersection, a bicycle suddenly rides from right side at a high speed.
The ego vehicle has to react accordingly to avoid a collision.
The scenario ends either if the ego vehicle stops, or if it keeps driving for a defined distance.
"""

from __future__ import print_function

import random

import py_trees

import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      StopVehicle,
KeepVelocity,
                                                                      SyncArrival,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               StandStill)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (get_geometric_linear_intersection,
                                           get_crossing_point,
                                           generate_target_waypoint)


class BicycleCrossFront(BasicScenario):
    """
    This is a single ego vehicle scenario
    """

    timeout = 120  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._ego_distance = 80  # defined distance that the ego vehicle is excepted to drive for
        self._target_vel = 20  # velocity of bicycle
        self.timeout = timeout

        super(BicycleCrossFront, self).__init__("BicycleCrossFront",
                                                ego_vehicles,
                                                config,
                                                world,
                                                debug_mode,
                                                criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        Behavior Sequence:
        - Show Bicycle: use ActorTransformSetter() to spawn the bicycle
        - keep_velocity: keep riding and ignore the ego vehicle
        - end_condition: drive for a defined distance or stop for 1 second
        """

        # bicycle keep going
        keep_velocity = KeepVelocity(self.other_actors[0], self._target_vel)

        # end condition
        end_condition = py_trees.composites.Parallel("Waiting for end",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        end_condition_part1 = DriveDistance(self.ego_vehicles[0], self._ego_distance)
        end_condition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        end_condition.add_child(end_condition_part1)
        end_condition.add_child(end_condition_part2)

        # Behavior tree
        root = py_trees.composites.Sequence()
        root.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))

        sequence = py_trees.composites.Parallel("Behavior Parallel",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sequence.add_child(keep_velocity)
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
