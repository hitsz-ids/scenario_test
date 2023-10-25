#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Vehicle from opposite scenario:

The scenario realizes a common driving behavior, in which the user-controlled ego vehicle drives on the street.
When ego vehicle drives for a while, another vehicle suddenly drives from opposite legally.
The ego vehicle has to react accordingly to avoid a collision.
The scenario ends either if the ego vehicle stops.
"""

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      WaypointFollower,
KeepVelocity,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class VehicleFromOpposite(BasicScenario):
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
        self._trigger_distance = 30  # decide when the opposite vehicle start to brake
        self._opposite_vehicle_speed = 20
        self._max_brake = 1
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(VehicleFromOpposite, self).__init__("VehicleFromOpposite",
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
        Sequence Tesla:
        - just_drive: keep driving until in trigger distance with ego vehicle
        - stop: stop driving

        Order of Sequence:
        - sequence tesla
        - end_condition: stop for 1 second
        """

        # sequence tesla
        sequence_tesla = py_trees.composites.Sequence("Tesla")
        sequence_tesla.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))

        # keep driving until in trigger distance with ego vehicle
        just_drive = py_trees.composites.Parallel("DriveTowardsVehicle",
                                                  policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        tesla_driving = KeepVelocity(self.other_actors[0], self._opposite_vehicle_speed)
        just_drive.add_child(tesla_driving)
        distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._trigger_distance)
        just_drive.add_child(distance_to_vehicle)
        sequence_tesla.add_child(just_drive)

        # brake
        stop = StopVehicle(self.other_actors[0], self._max_brake)
        sequence_tesla.add_child(stop)
        sequence_tesla.add_child(Idle())

        # end condition
        end_condition = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        # Build behavior tree
        root = py_trees.composites.Parallel("Parallel Behavior",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(sequence_tesla)
        root.add_child(end_condition)
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
