#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Leading vehicle sudden brake scenario:

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the leading car suddenly brake.
The ego vehicle has to react accordingly to avoid a collision.
The scenario ends if the ego vehicle stops.
"""

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class LeadingVehicleSuddenBrake(BasicScenario):
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
        self._first_vehicle_speed = 20
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._max_brake = 1.0
        self._stop_in_front_intersection = 40
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(LeadingVehicleSuddenBrake, self).__init__("LeadingVehicleSuddenBrake",
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
        Order of Sequence:
        - Show Vehicle: use ActorTransformSetter() to spawn the leading vehicle
        - driving_to_next_intersection: drive until next intersection
        - stop: stop the leading vehicle
        - end_condition: end if the ego vehicle stops for 1 second
        """

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._stop_in_front_intersection))

        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._max_brake)

        # end condition
        end_condition = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        # Build behavior tree
        sequence = py_trees.composites.Sequence()
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(end_condition)

        return sequence

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
