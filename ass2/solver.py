import math
import random
import sys
from tester import test_obstacle_collision, __get_lenient_obstacles, test_config_equality, test_self_collision, \
    test_environment_bounds
from support.problem_spec import ProblemSpec
from support.robot_config import RobotConfig, make_robot_config_from_ee1, make_robot_config_from_ee2, \
    write_robot_config_list_to_file
from support.angle import Angle
from support.obstacle import Obstacle


class GraphNode:
    def __init__(self, spec, config):
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors


def check_distance(spec, config1, config2):
    for i in range(spec.num_segments):
        if config2.lengths[i]-config1.lengths[i] > 0.5:
            return False
        if abs(config2.ee1_angles[i].radians - config1.ee1_angles[i].radians) > 0.5:
            return False

    return True


def uniformly_sampling(spec, obstacles, stage):
    while True:
        sampling_angle = []
        sampling_length = []
        for i in range(spec.num_segments):
            angle = random.uniform(-165, 165)
            sampling_angle.append(Angle(degrees=float(angle)))
            length = random.uniform(spec.min_lengths[i], spec.max_lengths[i])
            sampling_length.append(length)

        if stage % 2 == 0:
            new_config = make_robot_config_from_ee1(spec.grapple_points[stage][0], spec.grapple_points[stage][1],
                                                    sampling_angle, sampling_length, ee1_grappled=True)
        else:
            new_config = make_robot_config_from_ee2(spec.grapple_points[stage][0], spec.grapple_points[stage][1],
                                                    sampling_angle, sampling_length, ee2_grappled=True)

        if test_obstacle_collision(new_config, spec, obstacles) and test_self_collision(new_config, spec) and \
                test_environment_bounds(new_config):
            return new_config


def check_path(config1, config2, spec, obstacles):
    angles1 = config1.ee1_angles
    angles2 = config2.ee1_angles
    angle_distance = []
    if config1.ee2_grappled is True:
        angles1 = config1.ee2_angles
        angles2 = config2.ee2_angles

    lengths1 = config1.lengths
    lengths2 = config2.lengths
    length_distance = []

    for j in range(len(angles1)):
        angle_distance.append(angles2[j].__sub__(angles1[j]))
        length_distance.append(lengths2[j] - lengths1[j])

    for j in range(20):
        new_angle_list = []
        new_length_list = []
        for k in range(len(angles1)):
            if config1.ee1_grappled is True:
                new_angle_list.append(config1.ee1_angles[k].__add__(angle_distance[k].__mul__(0.05 * j)))
            elif config1.ee2_grappled is True:
                new_angle_list.append(config1.ee2_angles[k].__add__(angle_distance[k].__mul__(0.05 * j)))

            new_length_list.append(config1.lengths[k] + length_distance[k] * 0.05 * j)

        x, y = config1.points[0]
        new_config = make_robot_config_from_ee1(x, y, new_angle_list, new_length_list, ee1_grappled=True, ee2_grappled=False)
        if config1.ee2_grappled is True:
            x, y = config1.points[-1]
            new_config = make_robot_config_from_ee2(x, y, new_angle_list, list(reversed(new_length_list)),
                                                    ee1_grappled=False, ee2_grappled=True)

        if not (test_obstacle_collision(new_config, spec, obstacles) and test_self_collision(new_config, spec) and
                test_environment_bounds(new_config)):
            return False

    return True


def create_state_graph(spec, obstacles):
    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    nodes = [init_node, goal_node]
    for stage in range(spec.num_grapple_points):
        for i in range(500):
            sample_point = uniformly_sampling(spec, obstacles, stage)
            new_node = GraphNode(spec, sample_point)
            nodes.append(new_node)

            for node in nodes:
                if check_distance(spec, sample_point, node.config) and check_path(sample_point, node.config, spec,
                                                                                  obstacles):
                    node.neighbors.append(new_node)
                    new_node.neighbors.append(node)

    init_container = [init_node]
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, spec.goal, spec):
            result = init_visited[current]
            # return result
            new_list = []
            for i in range(len(result) - 1):
                angles1 = result[i].ee1_angles
                angles2 = result[i + 1].ee1_angles
                if result[i].ee2_grappled is True:
                    angles1 = result[i].ee2_angles
                    angles2 = result[i + 1].ee2_angles

                angle_distance = []
                lengths1 = result[i].lengths
                lengths2 = result[i + 1].lengths
                length_distance = []

                for j in range(len(angles1)):
                    angle_distance.append(angles2[j].__sub__(angles1[j]))
                    length_distance.append(lengths2[j] - lengths1[j])

                for j in range(501):
                    new_angle_list = []
                    new_length_list = []
                    for k in range(len(angles1)):
                        if result[i].ee1_grappled is True:
                            new_angle_list.append(result[i].ee1_angles[k].__add__(angle_distance[k].__mul__(0.002 * j)))
                        elif result[i].ee2_grappled is True:
                            new_angle_list.append(result[i].ee2_angles[k].__add__(angle_distance[k].__mul__(0.002 * j)))

                        new_length_list.append(result[i].lengths[k] + length_distance[k] * 0.002 * j)

                    if result[i].ee1_grappled is True:
                        x, y = result[i].points[0]
                        new_config = make_robot_config_from_ee1(x, y, new_angle_list, new_length_list,
                                                                ee1_grappled=True, ee2_grappled=False)
                        if test_obstacle_collision(new_config, spec, obstacles) and test_self_collision(new_config, spec) and test_environment_bounds(new_config):
                            new_list.append(new_config)
                    elif result[i].ee2_grappled is True:
                        x, y = result[i].points[-1]
                        new_config = make_robot_config_from_ee2(x, y, new_angle_list, list(reversed(new_length_list)),
                                                                ee1_grappled=False, ee2_grappled=True)
                        if test_obstacle_collision(new_config, spec, obstacles) and test_self_collision(new_config, spec) and test_environment_bounds(new_config):
                            new_list.append(new_config)

            return new_list

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]


def main(arglist):
    input_file = arglist[0]
    spec = ProblemSpec(input_file)
    obstacles = __get_lenient_obstacles(spec)
    s = create_state_graph(spec, obstacles)
    write_robot_config_list_to_file("filename", s)


if __name__ == '__main__':
    main(sys.argv[1:])
