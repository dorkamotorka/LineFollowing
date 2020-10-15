#! /usr/bin/env python
from __future__ import division

import rospy
import re
import math
import json
import datetime
import os
import requests
import subprocess

from geometry_msgs.msg import Point
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties

from std_msgs.msg import String
from line_follower_evaluator.msg import EvaluatorMetric

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from angles import normalize_angle, shortest_angular_distance

from line_follower_evaluator.tile import Tile


class AdaptiveEvaluatorNode:

    def __init__(self):

        self.nodeName = 'adaptive_evaluator'
        self.robotName = 'robot1'
        self.git_dir = '/workspace/src/.git'

        user_name = ''
        remote_name = ''
        branch_name = ''
        commit_id = ''
        riders_project_id = os.environ.get('RIDERS_PROJECT_ID', '')

        # RIDERS_COMMIT_ID
        git_dir_exists = os.path.isdir(self.git_dir)
        if git_dir_exists:

            popen = subprocess.Popen(['git', 'remote', 'get-url', 'origin'], cwd=self.git_dir, stdout=subprocess.PIPE)
            remote_url, error_remote_url = popen.communicate()

            popen = subprocess.Popen(['git', 'name-rev', '--name-only', 'HEAD'], cwd=self.git_dir, stdout=subprocess.PIPE)
            branch_name, error_branch_name = popen.communicate()

            popen = subprocess.Popen(['git', 'rev-parse', 'HEAD'], cwd=self.git_dir, stdout=subprocess.PIPE)
            commit_id, error_commit_id = popen.communicate()

            if error_remote_url is None and remote_url is not None:
                remote_url = remote_url.rstrip('\n').rsplit('@')
                user_name = remote_url[0].rsplit(':')[1][2:]
                remote_name = remote_url[1]

        # published at a high frequency
        self.display_metrics = {
            'sum_area': '0',
            'sum_distance': '0',
            'sum_theta': '0',
            'disqualification_reason': None,
            'race_started': False,
            'race_finished': False,
            'time': '0',
            'score': '0'
        }

        # published once, at the end of the evaluation
        self.result_metrics = {
            'sum_area': '0',
            'sum_distance': '0',
            'sum_theta': '0',
            'disqualification_reason': None,
            'race_started': False,
            'race_finished': False,
            'time': '0',
            'user_name': user_name.rstrip('\n'),
            'remote_name': remote_name.rstrip('\n'),
            'branch_name': branch_name.rstrip('\n'),
            'commit_id': commit_id.rstrip('\n'),
            'track_distance': '0',
            'track_sum_theta': '0',
            'rec_time': '',
            'riders_project_id': riders_project_id
        }

        self.tile_metrics = {
            'track_name': '',
            'track_type': '',
            'sum_area': '0',
            'time': '0'
        }

        self.diagnostics = {
            'event': '',
            'source': '',
            'rec_time': '0'
        }

        # initialize rospy
        rospy.init_node(self.nodeName)

        # get parameters
        self.debug = bool(rospy.get_param('~debug', False))
        self.send_to_api = bool(rospy.get_param('~send_api', True))
        self.send_disq = bool(rospy.get_param('~send_disq', True))
        self.rate = float(rospy.get_param('~rate', 50.0))
        self.race_timeout = float(rospy.get_param('~race_timeout', 200.0))

        if self.debug:
            self.print_diagnostics('started', 'system')

        def shutdown():
            self.print_diagnostics('shutdown', 'system')

        rospy.on_shutdown(shutdown)

        # global position
        self.position = Point()
        self.prev_position = Point()

        # map of evaluators
        self.tiles = {}
        self.get_tiles()

        # index of current and previous tile
        self.current_tile = 0
        self.prev_tile = 0

        # evaluator flags and timing
        self.disqualified = False
        self.disqualification_reason = ''
        self.correct_position = False
        self.eval_finished = False
        self.start_time = None
        self.finished_time = None

        # theta and prev
        self.theta = 0
        self.prev_theta = 0

        # published metrics
        self.track_distance = 0
        self.sum_area = 0
        self.sum_theta = 0
        self.sum_distance = 0
        self.race_started = False
        self.race_finished = False

        # calculated metrics
        self.corrected_elapsed_time = None
        self.elapsed_time = 0
        self.score = 0

        self.evaluatorMetric = EvaluatorMetric()

        # track characteristics are fetched automatically
        self.track_distance, self.track_sum_theta = self.get_track_character()

        # publisher for recording evaluator metrics, with header, and timestamp
        self.metrics_pub = rospy.Publisher('evaluator_metrics', EvaluatorMetric, queue_size=32)

        # publisher for sending evaluator metrics to display
        self.display_metrics_pub = rospy.Publisher('simulation_metrics', String, queue_size=1)

        # do not remove, starts robots
        self.publish_metrics(None)

    def get_tiles(self):

        def get_models():

            rospy.wait_for_service('/gazebo/get_world_properties')
            try:
                get_models = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
                resp = get_models()
                return resp.model_names

            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: %s', e)

        def get_model_properties(model_name):

            rospy.wait_for_service('/gazebo/get_model_properties')
            try:
                model_props = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
                resp = model_props(model_name)
                return resp.body_names[0]

            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: %s', e)

        def get_model_state(model_name):

            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                resp = get_state(model_name, None)

                quad = resp.pose.orientation
                # yaw calculation(z-axis rotation)
                siny_cosp = 2 * (quad.w * quad.z + quad.x * quad.y)
                cosy_cosp = 1 - 2 * (quad.y * quad.y + quad.z * quad.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                data = []
                data.append(resp.pose.position.x)
                data.append(resp.pose.position.y)
                data.append(yaw)

                return data

            except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: %s', e)

        def atoi(text):

            return int(text) if text.isdigit() else text

        def natural_keys(text):

            '''
            alist.sort(key=natural_keys) sorts in human order
            http://nedbatchelder.com/blog/200712/human_sorting.html
            (See Toothy's implementation in the comments)
            '''
            return [atoi(c) for c in re.split(r'(\d+)', text)]

        track_types = ['lefttrack', 'righttrack', 'linetrack', 'sharpleft', 'sharpright']

        model_names = [model for model in get_models() if ('track' in model)]
        model_names.sort(key=natural_keys)

        order = 0
        for model in model_names:
            link_name = get_model_properties(model)
            pos_data = get_model_state(model)
            self.tiles.update(
                {model: Tile(track_types.index(link_name), model, order, pos_data[0], pos_data[1], pos_data[2])})
            order += 1

    def update_current_tile(self, position, prev_position):
        for tile in self.tiles.values():
            if abs(tile.evaluator.tile_x - position.x) < 0.5 and abs(tile.evaluator.tile_y - position.y) < 0.5:
                tile.update(position, prev_position)
                return tile.order
        return -1

    def get_sum_area(self):
        sum_area = 0
        for tile in self.tiles.values():
            sum_area += tile.get_sum_area()
        self.sum_area = sum_area

    def get_track_character(self):
        track_distance = 0
        track_sum_theta = 0
        for tile in self.tiles.values():
            _d, _t = tile.get_character()
            track_distance += _d
            track_sum_theta += _t
        return track_distance, track_sum_theta

    def send_results(self, score=0.0, disqualified=False,  **kwargs):

        # if disqualified, and send disqualifications is not set, then return
        if disqualified:
            if not self.send_disq:
                return

        # only send if send to api is true
        if self.send_to_api:
            simulation_id = os.environ.get('RIDERS_SIMULATION_ID', None)
            args = score, disqualified
            try:
                if simulation_id:
                    self.send_simulation_results(simulation_id, *args, **kwargs)
                else:
                    self.send_branch_results(*args, **kwargs)
            except Exception as e:
                rospy.loginfo('Exception occurred while sending metric: %s', e)

    def send_simulation_results(self, simulation_id, score, disqualified, **kwargs):
        host = os.environ.get('RIDERS_HOST', None)
        token = os.environ.get('RIDERS_SIMULATION_AUTH_TOKEN', None)
        create_round_url = '%s/api/v1/simulation/%s/rounds/' % (host, simulation_id)
        kwargs['score'] = kwargs.get('score', score)
        kwargs['disqualified'] = kwargs.get('disqualified', disqualified)
        data = {
            'metrics': json.dumps(kwargs)
        }
        # send actual data
        requests.post(create_round_url, {}, data, headers={
            'Authorization': 'Token %s' % token,
            'Content-Type': 'application/json',
        })

    def send_branch_results(self, score, disqualified, **kwargs):

        rsl_host = 'http://localhost:8059'

        # metrics endpoint stores data differently compared to simulation endpoint,
        # hence we change how we send it
        info = {
            'score': score,
            'disqualified': disqualified,
        }
        info.update(kwargs)
        url = '%s/events' % rsl_host
        data = {
            'event': 'metrics',
            'info': json.dumps(info)
        }

        requests.post(url, {}, data, headers={
            'Content-Type': 'application/json',
        })

    def publish_metrics(self, current_time):

        if self.start_time and not self.eval_finished:
            self.elapsed_time = (current_time - self.start_time).to_sec()

        # publish metrics for gzweb
        self.display_metrics['sum_area'] = str(self.sum_area)
        self.display_metrics['sum_distance'] = str(self.sum_distance)
        self.display_metrics['sum_theta'] = str(self.sum_theta)
        self.display_metrics['disqualification_reason'] = str(self.disqualification_reason)
        self.display_metrics['race_started'] = str(self.race_started)
        self.display_metrics['race_finished'] = str(self.race_finished)
        self.display_metrics['time'] = str(self.elapsed_time)

        self.display_metrics_pub.publish(json.dumps(self.display_metrics, sort_keys=True))

        if current_time:
            self.evaluatorMetric.header.stamp = current_time

        self.evaluatorMetric.sum_area = self.sum_area
        self.evaluatorMetric.sum_distance = self.sum_distance
        self.evaluatorMetric.sum_theta = self.sum_theta
        self.evaluatorMetric.disqualification_reason = self.disqualification_reason
        self.evaluatorMetric.race_started = self.race_started
        self.evaluatorMetric.race_finished = self.race_finished
        self.evaluatorMetric.elapsed_time = self.elapsed_time
        self.evaluatorMetric.score = self.score

        self.metrics_pub.publish(self.evaluatorMetric)

    def populate_result_metrics(self):

        self.result_metrics['sum_area'] = self.display_metrics['sum_area']
        self.result_metrics['sum_distance'] = self.display_metrics['sum_distance']
        self.result_metrics['sum_theta'] = self.display_metrics['sum_theta']
        self.result_metrics['disqualification_reason'] = self.display_metrics['disqualification_reason']
        self.result_metrics['race_started'] = self.display_metrics['race_started']
        self.result_metrics['race_finished'] = self.display_metrics['race_finished']
        self.result_metrics['time'] = self.display_metrics['time']
        self.result_metrics['track_distance'] = str(self.track_distance)
        self.result_metrics['track_sum_theta'] = str(self.track_sum_theta)
        self.result_metrics['rec_time'] = str(datetime.datetime.now())

    def calculate_score(self, elapsed_time, sum_distance, sum_area):

        d_negative = self.track_distance - sum_distance
        time_penalty = 0
        avg_speed = sum_distance / elapsed_time
        if d_negative > 0:
            time_penalty = d_negative / avg_speed

        self.corrected_elapsed_time = elapsed_time + time_penalty
        self.score = self.track_distance / (self.corrected_elapsed_time + (20 * sum_area))

    def print_tile_stats(self, pt, rec_time):

        self.tile_metrics['track_name'] = pt.trackName
        self.tile_metrics['track_type'] = pt.evaluator.evaluator_type
        self.tile_metrics['sum_area'] = pt.get_sum_area()

        if self.start_time:
            self.tile_metrics['time'] = str((rec_time - self.start_time).to_sec())

        rospy.loginfo(self.tile_metrics)

    def print_diagnostics(self, event, source):

        self.diagnostics['event'] = event
        self.diagnostics['source'] = source
        self.diagnostics['rec_time'] = str(datetime.datetime.now())

        rospy.loginfo(self.diagnostics)

    def main(self):

        r = rospy.Rate(self.rate)

        # wait for gazebo/get_model_state service to be available
        rospy.wait_for_service('/gazebo/get_model_state')

        evaluation_timeout = rospy.Time.now() + rospy.Duration.from_sec(self.race_timeout)

        while not rospy.is_shutdown():

            current_time = rospy.Time.now()

            try:
                get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                robot_state = get_model_state(self.robotName, None)
            except Exception as e:
                print(e)

            # get position
            self.position = robot_state.pose.position

            # calculate orientation
            orientation_q = robot_state.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, self.theta) = euler_from_quaternion(orientation_list)

            if not self.race_started and not self.eval_finished:

                # check initial point
                if self.position.x < 0.001 and self.position.y < 0.001:
                    self.correct_position = True
                elif self.correct_position:
                    self.start_time = current_time
                    self.race_started = True
                    if self.debug:
                        self.print_diagnostics('race_started', 'evaluator')
                elif not self.correct_position:
                    self.disqualified = True
                    self.disqualification_reason = 'incorrect_start'
                    self.print_diagnostics('incorrect_start', 'evaluator')

            elif not self.eval_finished:

                # calculate sum_theta and sum_distance
                self.sum_theta += math.fabs(normalize_angle(self.theta - self.prev_theta))
                self.sum_distance += math.sqrt(
                    ((self.position.x - self.prev_position.x) ** 2) + ((self.position.y - self.prev_position.y) ** 2))

                # update position
                self.current_tile = self.update_current_tile(self.position, self.prev_position)

                # state checking
                if self.current_tile == -1:

                    self.disqualified = True
                    self.disqualification_reason = 'out_of_track'
                    self.print_diagnostics('out_of_track', 'evaluator')

                elif self.current_tile != self.prev_tile:

                    # we check state transition by checking if prev_tile is -1
                    # robot could start with tile 1, but this would be disqualified before by previous checks
                    pt = self.tiles['track_%s' % self.prev_tile]

                    if self.prev_tile + 1 == self.current_tile:

                        # state transition between tiles occurs here
                        pt.lock_tile()
                        if self.debug:
                            self.print_tile_stats(pt, current_time)

                    elif self.prev_tile == (len(self.tiles) - 1) and self.current_tile == 0:

                        # race finished
                        pt.lock_tile()
                        self.finished_time = current_time
                        self.race_finished = True

                        if self.debug:
                            self.print_tile_stats(pt, current_time)
                            self.print_diagnostics('race_finished', 'evaluator')

                    else:

                        # wrong track, disqualified
                        self.disqualified = True
                        self.disqualification_reason = 'out_of_sequence'
                        self.print_diagnostics('out_of_sequence', 'evaluator')

                # sum total area from tracks
                self.get_sum_area()

                # notice prev_position, and prev_tile must be recorded only when race has started
                self.prev_position = self.position
                self.prev_tile = self.current_tile

            # notice prev theta should be recorded even if race has not started
            self.prev_theta = self.theta

            # publish evaluator data
            self.publish_metrics(current_time)

            # disqualification by timeout
            if (evaluation_timeout < current_time) and not self.eval_finished:

                self.disqualified = True
                self.disqualification_reason = 'out_of_time'
                self.print_diagnostics('out_of_time', 'evaluator')

                # safety
                self.display_metrics['disqualification_reason'] = self.disqualification_reason

                # send result metrics
                self.populate_result_metrics()
                self.send_results(0.0, self.disqualified, **self.result_metrics)

                # print self metrics here, because will skip next statement because self.eval_finished will be true
                rospy.loginfo(self.display_metrics)
                self.eval_finished = True

            # notice race could be finished, or disqualified here
            if (self.race_finished or self.disqualified) and not self.eval_finished:

                # safety
                self.display_metrics['disqualification_reason'] = self.disqualification_reason

                if self.race_finished:
                    self.calculate_score(self.elapsed_time, self.sum_distance, self.sum_area)
                    self.display_metrics['score'] = str(self.score)

                # send result metrics
                self.populate_result_metrics()
                self.send_results(self.score, self.disqualified, **self.result_metrics)

                rospy.loginfo(self.display_metrics)
                self.eval_finished = True

                # uncomment this when operating in local
                # rospy.signal_shutdown('evaluator_finished')


if __name__ == '__main__':

    try:
        node = AdaptiveEvaluatorNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
