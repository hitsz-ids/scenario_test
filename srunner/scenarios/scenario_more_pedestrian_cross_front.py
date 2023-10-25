#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
More pedestrian cross front scenario:

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle drives on the street. When the ego
vehicle reaches the intersection, two pedestrians suddenly run from
right side. One of them will keep running and ignore the ego vehicle,
while the other will be scared and stop when the ego vehicle come close enough.
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


class MorePedestrianCrossFront(BasicScenario):
    """
    This is a single ego vehicle scenario
    """

    timeout = 120  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)

        self._start_trigger_distance = 45  # decide when the pedestrians start running
        self._stop_trigger_distance = 10  # decide when the pedestrian stop

        self._first_pedestrian_speed = 6
        self._second_pedestrian_speed = 5

        self._max_brake = 1
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(MorePedestrianCrossFront, self).__init__("MorePedestrianCrossFront",
                                                       ego_vehicles,
                                                       config,
                                                       world,
                                                       debug_mode,
                                                       criteria_enable=criteria_enable)

        if randomize:
            self._first_vehicle_speed = random.randint(1, 6)

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
        Sequence of Pedestrian 1:
        - start_distance_to_vehicle: wait until ego vehicle come
        - run_until_vehicle_come: run until in trigger distance to ego_vehicle
        - stop: stop running

        Sequence of Pedestrian 2:
        - start_distance_to_vehicle: wait until ego vehicle come
        - just_run: keep running and ignore ego vehicle
        - stop: stop running

        Order of Sequence:
        - sequence pedestrian1 & sequence pedestrian2
        - end_condition: end if ego vehicle stop for 1 second
        """

        # sequence pedestrian1
        sequence_pedestrian1 = py_trees.composites.Sequence("Pedestrian1 Behavior")
        sequence_pedestrian1.add_child(ActorTransformSetter(self.other_actors[0], self._first_transform))

        # wait until ego vehicle come
        start_distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._start_trigger_distance)
        sequence_pedestrian1.add_child(start_distance_to_vehicle)

        # start running
        run_until_vehicle_come = py_trees.composites.Parallel("WalkTowardsVehicle",
                                                              policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        pedestrian_run = WaypointFollower(self.other_actors[0], self._first_pedestrian_speed)
        run_until_vehicle_come.add_child(pedestrian_run)
        distance_to_vehicle = InTriggerDistanceToVehicle(
            self.other_actors[0], self.ego_vehicles[0], self._stop_trigger_distance)
        run_until_vehicle_come.add_child(distance_to_vehicle)
        sequence_pedestrian1.add_child(run_until_vehicle_come)

        # stop running
        stop = StopVehicle(self.other_actors[0], self._max_brake)
        sequence_pedestrian1.add_child(stop)
        sequence_pedestrian1.add_child(Idle())

        # sequence pedestrian2
        sequence_pedestrian2 = py_trees.composites.Sequence("Pedestrian2 Behavior")
        sequence_pedestrian2.add_child(ActorTransformSetter(self.other_actors[1], self._second_transform))
        start_distance_to_vehicle2 = InTriggerDistanceToVehicle(
            self.other_actors[1], self.ego_vehicles[0], self._start_trigger_distance)
        sequence_pedestrian2.add_child(start_distance_to_vehicle2)
        just_run = WaypointFollower(self.other_actors[1], self._second_pedestrian_speed)
        sequence_pedestrian2.add_child(just_run)
        stop = StopVehicle(self.other_actors[1], self._max_brake)
        sequence_pedestrian2.add_child(stop)
        sequence_pedestrian2.add_child(Idle())

        # end condition
        end_condition = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)

        # Build behavior tree
        root = py_trees.composites.Parallel("Parallel Behavior",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(sequence_pedestrian1)
        root.add_child(sequence_pedestrian2)
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
