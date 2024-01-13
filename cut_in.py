#!/usr/bin/env python

# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Cut in scenario:

The scenario realizes a driving behavior on the highway.
The user-controlled ego vehicle is driving straight and keeping its velocity at a constant level.
Another car is cutting just in front, coming from left or right lane.

The ego vehicle may need to brake to avoid a collision.
"""

import random
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      LaneChange,
                                                                      WaypointFollower,
                                                                      AccelerateToCatchUp, ActorDestroy)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle, DriveDistance, \
    InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import generate_target_waypoint
from agents.navigation.local_planner import RoadOption


class CutIn(BasicScenario):
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

        self._velocity = 60  # 40
        self._delta_velocity = 20  # 10
        self._trigger_distance = 30  # 30

        # get direction from config name
        self._config = config
        self._direction = None
        self._transform_visible = None

        super(CutIn, self).__init__("CutIn",
                                    ego_vehicles,
                                    config,
                                    world,
                                    debug_mode,
                                    criteria_enable=criteria_enable)

        if randomize:
            self._velocity = random.randint(20, 60)
            self._trigger_distance = random.randint(10, 40)

    def _initialize_actors(self, config):

        # direction of lane, on which other_actor is driving before lane change
        if 'LEFT' in self._config.name.upper():
            self._direction = 'left'

        if 'RIGHT' in self._config.name.upper():
            self._direction = 'right'

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            vehicle.set_simulate_physics(enabled=False)

        # transform visible

        other_actor_transform = self.other_actors[1].get_transform()
        # self._transform_visible = carla.Transform(
        #     carla.Location(other_actor_transform.location.x,
        #                    other_actor_transform.location.y,
        #                    other_actor_transform.location.z + 105),
        #     other_actor_transform.rotation)
        self._transform_visible = carla.Transform(
            carla.Location(other_actor_transform.location.x,
                           other_actor_transform.location.y,
                           other_actor_transform.location.z),
            other_actor_transform.rotation)

        other_actor_transform_front = self.other_actors[0].get_transform()
        self._transform_visible_front = carla.Transform(
            carla.Location(other_actor_transform_front.location.x,
                           other_actor_transform_front.location.y,
                           other_actor_transform_front.location.z),
            other_actor_transform.rotation)

    def _create_behavior(self):
        """
        Order of sequence:
        - car_visible: spawn car at a visible transform
        - just_drive: drive until in trigger distance to ego_vehicle
        - accelerate: accelerate to catch up distance to ego_vehicle
        - lane_change: change the lane
        - endcondition: drive for a defined distance
        """

        # car_visible
        behaviour = py_trees.composites.Sequence("CarOn_{}_Lane".format(self._direction))

        ###
        behaviour_front = py_trees.composites.Sequence("Follow_front")

        car_visible = ActorTransformSetter(self.other_actors[1], self._transform_visible)

        ###
        car_visible_front = ActorTransformSetter(self.other_actors[0], self._transform_visible_front)

        behaviour.add_child(car_visible)

        ###
        behaviour_front.add_child(car_visible_front)

        # just_drive
        just_drive = py_trees.composites.Parallel(
            "DrivingStraight", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        car_driving = WaypointFollower(self.other_actors[1], self._velocity)
        just_drive.add_child(car_driving)

        trigger_distance = InTriggerDistanceToVehicle(
            self.other_actors[1], self.ego_vehicles[0], self._trigger_distance)

        # accelerate_front = AccelerateToCatchUp(self.other_actors[0], self.ego_vehicles[0], throttle_value=1,
        #                                  delta_velocity=self._delta_velocity, trigger_distance=5,
        #                                  max_distance=500)  # 5  500

        ###
        target_waypoint = CarlaDataProvider.get_map().get_waypoint(self.other_actors[0].get_location(), 0)
        plan = []
        wp_choice = target_waypoint.next(1.0)
        while len(plan) < 200:
            target_waypoint = wp_choice[0]
            plan.append((target_waypoint, RoadOption.LANEFOLLOW))
            wp_choice = target_waypoint.next(1.0)

        # move_actor = WaypointFollower(self.other_actors[0], self._velocity, plan=plan)
        # waypoint_follower_end = InTriggerDistanceToLocation(
        #     self.other_actors[0], plan[-1][0].transform.location, 10)
        ###
        accelerate_front = WaypointFollower(self.other_actors[0], self._velocity, plan=plan)

        just_drive.add_child(trigger_distance)
        # just_drive.add_child(accelerate_front)
        behaviour_front.add_child(accelerate_front)
        # behaviour_front.add_child(ActorDestroy(self.other_actors[0]))

        behaviour.add_child(just_drive)

        # accelerate
        accelerate = AccelerateToCatchUp(self.other_actors[1], self.ego_vehicles[0], throttle_value=1,
                                         delta_velocity=self._delta_velocity, trigger_distance=5,
                                         max_distance=500)  # 5  500

        behaviour.add_child(accelerate)
        # behaviour.add_child(accelerate_front)

        # lane_change
        if self._direction == 'left':
            lane_change = LaneChange(
                self.other_actors[1], speed=None, direction='right', distance_same_lane=5, distance_other_lane=300)
            behaviour.add_child(lane_change)
        else:
            lane_change = LaneChange(
                self.other_actors[1], speed=None, direction='left', distance_same_lane=5, distance_other_lane=300)
            behaviour.add_child(lane_change)


        # endcondition
        endcondition = DriveDistance(self.other_actors[1], 5)  # 200
        behaviour.add_child(endcondition)
        behaviour.add_child(ActorDestroy(self.other_actors[1]))
        # build tree
        # root = py_trees.composites.Sequence("Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root = py_trees.composites.Parallel('Two Behavior', policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        # root = py_trees.composites.Sequence("Behavior")
        root.add_child(behaviour)
        root.add_child(behaviour_front)
        print(py_trees.display.print_ascii_tree(root, show_status=True))

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
