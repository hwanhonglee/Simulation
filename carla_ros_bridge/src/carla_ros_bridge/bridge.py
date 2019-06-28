#!/usr/bin/env python
#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosbridge class:

Class that handle communication between CARLA and ROS
"""
import threading
import time
import rospy

import carla

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.communication import Communication
from carla_ros_bridge.sensor import Sensor

from carla_ros_bridge.map import Map
from carla_ros_bridge.spectator import Spectator
from carla_ros_bridge.traffic import Traffic, TrafficLight
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.lidar import Lidar
from carla_ros_bridge.gnss import Gnss
from carla_ros_bridge.ego_vehicle import EgoVehicle
from carla_ros_bridge.collision_sensor import CollisionSensor
from carla_ros_bridge.lane_invasion_sensor import LaneInvasionSensor
from carla_ros_bridge.camera import Camera, RgbCamera, DepthCamera, SemanticSegmentationCamera
from carla_ros_bridge.object_sensor import ObjectSensor


class CarlaRosBridge(object):

    """
    Carla Ros bridge
    """

    def __init__(self, carla_world, params):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        rospy.init_node("carla_bridge", anonymous=True)
        self.parameters = params
        self.actors = {}
        self.carla_world = carla_world

        self.timestamp_last_run = 0.0
        self.comm = Communication()

        # register callback to create/delete actors
        self.update_child_actors_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_update_child_actors)

        # register callback to update actors
        self.update_lock = threading.Lock()
        self.carla_world.on_tick(self._carla_time_tick)

        self.pseudo_actors = []

        # add map
        self.pseudo_actors.append(Map(carla_world=self.carla_world,
                                      communication=self.comm))

        # add global object sensor
        self.pseudo_actors.append(ObjectSensor(parent=None,
                                               communication=self.comm,
                                               actor_list=self.actors,
                                               filtered_id=None))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        self.update_child_actors_lock.acquire()
        self.update_lock.acquire()
        rospy.signal_shutdown("")
        rospy.loginfo("Exiting Bridge")

    def _carla_time_tick(self, carla_timestamp):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates.

        After successful locking the update mutex
        (only perform trylock to respect bridge processing time)
        the clock and the children are updated.
        Finally the ROS messages collected to be published are sent out.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not rospy.is_shutdown():
            if self.update_lock.acquire(False):
                if self.timestamp_last_run < carla_timestamp.elapsed_seconds:
                    self.timestamp_last_run = carla_timestamp.elapsed_seconds
                    self.comm.update_clock(carla_timestamp)
                    self._update()
                    self.comm.send_msgs()
                self.update_lock.release()

    def _carla_update_child_actors(self, _):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates of the actors

        After successful locking the mutex
        (only perform trylock to respect bridge processing time)
        the existing actors are updated.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not rospy.is_shutdown():
            if self.update_child_actors_lock.acquire(False):
                self._update_actors()
                # actors are only created/deleted around once per second
                time.sleep(1)
                self.update_child_actors_lock.release()

    def _update_actors(self):
        """
        update the available actors
        """
        carla_actors = self.carla_world.get_actors()
        # Add new actors
        for actor in carla_actors:
            if actor.id not in self.actors:
                self._create_actor(actor)

        # create list of carla actors ids
        carla_actor_ids = []
        for actor in carla_actors:
            carla_actor_ids.append(actor.id)

        # remove non-existing actors
        ids_to_delete = []
        for actor_id in self.actors:
            if actor_id not in carla_actor_ids:
                ids_to_delete.append(actor_id)

        if ids_to_delete:
            with self.update_lock:
                for id_to_delete in ids_to_delete:
                    # remove actor
                    actor = self.actors[id_to_delete]
                    rospy.loginfo("Remove {}(id={}, parent_id={}, prefix={})".format(
                        actor.__class__.__name__, actor.get_id(),
                        actor.get_parent_id(),
                        actor.get_prefix()))
                    actor.destroy()
                    del self.actors[id_to_delete]

                    # remove pseudo-actors that have actor as parent
                    updated_pseudo_actors = []
                    for pseudo_actor in self.pseudo_actors:
                        if pseudo_actor.get_parent_id() == id_to_delete:
                            rospy.loginfo("Remove {}(parent_id={}, prefix={})".format(
                                pseudo_actor.__class__.__name__,
                                pseudo_actor.get_parent_id(),
                                pseudo_actor.get_prefix()))
                            pseudo_actor.destroy()
                            del pseudo_actor
                        else:
                            updated_pseudo_actors.append(pseudo_actor)
                    self.pseudo_actors = updated_pseudo_actors

    def _create_actor(self, carla_actor):  # pylint: disable=too-many-branches,too-many-statements
        """
        create an actor
        """
        parent = None
        if carla_actor.parent:
            if carla_actor.parent.id in self.actors:
                parent = self.actors[carla_actor.parent.id]
            else:
                parent = self._create_actor(carla_actor.parent)

        actor = None
        pseudo_actors = []
        if carla_actor.type_id.startswith('traffic'):
            if carla_actor.type_id == "traffic.traffic_light":
                actor = TrafficLight(carla_actor, parent, self.comm)
            else:
                actor = Traffic(carla_actor, parent, self.comm)
        elif carla_actor.type_id.startswith("vehicle"):
            if carla_actor.attributes.get('role_name')\
                    in self.parameters['ego_vehicle']['role_name']:
                ego_vehicle = EgoVehicle(carla_actor, parent, self.comm)
                actor = ego_vehicle
                pseudo_actors.append(ObjectSensor(parent=ego_vehicle,
                                                  communication=self.comm,
                                                  actor_list=self.actors,
                                                  filtered_id=carla_actor.id))
            else:
                actor = Vehicle(carla_actor, parent, self.comm)
        elif carla_actor.type_id.startswith("sensor"):
            if carla_actor.type_id.startswith("sensor.camera"):
                if carla_actor.type_id.startswith("sensor.camera.rgb"):
                    actor = RgbCamera(carla_actor, parent, self.comm)
                elif carla_actor.type_id.startswith("sensor.camera.depth"):
                    actor = DepthCamera(carla_actor, parent, self.comm)
                elif carla_actor.type_id.startswith("sensor.camera.semantic_segmentation"):
                    actor = SemanticSegmentationCamera(carla_actor, parent, self.comm)
                else:
                    actor = Camera(carla_actor, parent, self.comm)
            elif carla_actor.type_id.startswith("sensor.lidar"):
                actor = Lidar(carla_actor, parent, self.comm)
            elif carla_actor.type_id.startswith("sensor.other.gnss"):
                actor = Gnss(carla_actor, parent, self.comm)
            elif carla_actor.type_id.startswith("sensor.other.collision"):
                actor = CollisionSensor(carla_actor, parent, self.comm)
            elif carla_actor.type_id.startswith("sensor.other.lane_invasion"):
                actor = LaneInvasionSensor(carla_actor, parent, self.comm)
            else:
                actor = Sensor(carla_actor, parent, self.comm)
        elif carla_actor.type_id.startswith("spectator"):
            actor = Spectator(carla_actor, parent, self.comm)
        else:
            actor = Actor(carla_actor, parent, self.comm)

        rospy.loginfo("Created {}(id={}, parent_id={},"
                      " type={}, prefix={}, attributes={})".format(
                          actor.__class__.__name__, actor.get_id(),
                          actor.get_parent_id(), carla_actor.type_id,
                          actor.get_prefix(), carla_actor.attributes))
        with self.update_lock:
            self.actors[carla_actor.id] = actor

        for pseudo_actor in pseudo_actors:
            rospy.loginfo("Created {}(parent_id={}, prefix={})".format(
                pseudo_actor.__class__.__name__,
                pseudo_actor.get_parent_id(),
                pseudo_actor.get_prefix()))
            with self.update_lock:
                self.pseudo_actors.append(pseudo_actor)

        return actor

    def run(self):
        """
        Run the bridge functionality.

        Registers on shutdown callback at rospy and spins ROS.

        :return:
        """
        rospy.on_shutdown(self.on_shutdown)
        rospy.spin()

    def on_shutdown(self):
        """
        Function to be called on shutdown.

        This function is registered at rospy as shutdown handler.

        """
        rospy.loginfo("Shutdown requested")
        self.destroy()

    def _update(self):
        """
        update all actors
        :return:
        """
        # update all pseudo actors
        for actor in self.pseudo_actors:
            actor.update()

        # update all carla actors
        for actor_id in self.actors:
            try:
                self.actors[actor_id].update()
            except RuntimeError as e:
                rospy.logwarn("Update actor {}({}) failed: {}".format(
                    self.actors[actor_id].__class__.__name__, actor_id, e))
                continue


def main():
    """
    main function for carla simulator ROS bridge
    maintaining the communication client and the CarlaBridge object
    """
    parameters = rospy.get_param('carla')
    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=parameters['host'], port=parameters['port']))

    carla_bridge = None
    carla_world = None
    carla_client = None
    try:
        carla_client = carla.Client(
            host=parameters['host'],
            port=parameters['port'])
        carla_client.set_timeout(2000)

        carla_world = carla_client.get_world()

        carla_bridge = CarlaRosBridge(carla_client.get_world(), parameters)
        carla_bridge.run()
    finally:
        del carla_world
        del carla_client


if __name__ == "__main__":
    main()
