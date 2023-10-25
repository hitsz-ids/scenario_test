#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Pedestrian cross front scenario:

The scenario realizes a common driving behavior,
in which the user-controlled ego vehicle drives on the street.
When the ego vehicle reaches the intersection,
one pedestrian suddenly run from right side.
The pedestrian will be scared and stop when the ego vehicle come close enough.
The ego vehicle has to react accordingly to avoid a collision.
The scenario ends if the ego vehicle stops.
"""

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      StopVehicle,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import (get_waypoint_in_distance,
                                           generate_target_waypoint)


class PedestrianCrossFront(BasicScenario):
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
        self._start_trigger_distance = 45
        self._stop_trigger_distance = 10
        self._pedestrian_speed = 6
        self._max_brake = 1
        self.timeout = timeout

        super(PedestrianCrossFront, self).__init__("PedestrianCrossFront",
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
        Sequence of Pedestrian:
        - start_distance_to_vehicle: wait until ego vehicle come
        - run_until_vehicle_come: run until in trigger distance to ego_vehicle
        - stop: stop running

        Order of Sequence:
        - sequence pedestrian
        - end_condition: end if ego vehicle stop for 1 second
        """

        # sequence pedestrian
        sequence_pedestrian = py_trees.composites.Sequence("Pedestrian Behavior")
        sequence_pedestrian.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))

        # wait until ego vehicle come
        start_distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._start_trigger_distance)
        sequence_pedestrian.add_child(start_distance_to_vehicle)

        # start running
        just_run = py_trees.composites.Parallel("WalkTowardsVehicle",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        pedestrian_run = WaypointFollower(self.other_actors[0], self._pedestrian_speed)
        just_run.add_child(pedestrian_run)
        distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._stop_trigger_distance)
        just_run.add_child(distance_to_vehicle)
        sequence_pedestrian.add_child(just_run)

        # stop running
        stop = StopVehicle(self.other_actors[0], self._max_brake)
        sequence_pedestrian.add_child(stop)
        sequence_pedestrian.add_child(Idle())

        # end condition
        end_condition = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        # Build behavior tree
        root = py_trees.composites.Parallel("Parallel Behavior",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(sequence_pedestrian)
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
