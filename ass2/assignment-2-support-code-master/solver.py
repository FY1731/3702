import math
import random
import sys
import time
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


def check_distance(spec, config1, config2, stage):
    for i in range(spec.num_segments):
        if config2.lengths[i]-config1.lengths[i] > 0.5:
            return False
        if stage % 2 == 0:
            if abs(config2.ee1_angles[i].radians - config1.ee1_angles[i].radians) > 0.5:
                return False
        else:
            if abs(config2.ee2_angles[i].radians - config1.ee2_angles[i].radians) > 0.5:
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


def check_path(config1, config2, spec, obstacles, stage):
    angles1 = config1.ee1_angles
    angles2 = config2.ee1_angles
    if stage % 2 == 1:
        angles1 = config1.ee2_angles
        angles2 = config2.ee2_angles

    angle_distance = []
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
            if stage % 2 == 0:
                new_angle_list.append(config1.ee1_angles[k].__add__(angle_distance[k].__mul__(0.05 * j)))
            elif stage % 2 == 1:
                new_angle_list.append(config1.ee2_angles[k].__add__(angle_distance[k].__mul__(0.05 * j)))

            new_length_list.append(config1.lengths[k] + length_distance[k] * 0.05 * j)

        if stage % 2 == 0:
            x, y = config1.points[0]
            new_config = make_robot_config_from_ee1(x, y, new_angle_list, new_length_list,
                                                    ee1_grappled=True)
        else:
            x, y = config1.points[-1]
            new_config = make_robot_config_from_ee2(x, y, new_angle_list, new_length_list,
                                                    ee2_grappled=True)

        if not (test_obstacle_collision(new_config, spec, obstacles) and test_self_collision(new_config, spec) and
                test_environment_bounds(new_config)):
            return False

    return True


def create_state_graph(spec, obstacles, stage, sub_init=None, sub_goals=None):
    if sub_init is None:
        init_node = GraphNode(spec, spec.initial)
    else:
        init_node = GraphNode(spec, sub_init)

    nodes = [init_node]

    if sub_goals is None:
        goal_node = GraphNode(spec, spec.goal)
        nodes.append(goal_node)
    else:
        for sub_goal in sub_goals:
            nodes.append(GraphNode(spec, sub_goal))

    for i in range(2000):
        sample_point = uniformly_sampling(spec, obstacles, stage)
        new_node = GraphNode(spec, sample_point)
        nodes.append(new_node)

        for node in nodes:
            if check_distance(spec, sample_point, node.config, stage):
                if check_path(sample_point, node.config, spec, obstacles, stage):
                    node.neighbors.append(new_node)
                    new_node.neighbors.append(node)

    init_container = [init_node]
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        find_goal = False
        if sub_goals is None:
            if test_config_equality(current.config, spec.goal, spec):
                find_goal = True
        else:
            for sub_goal in sub_goals:
                if test_config_equality(current.config, sub_goal, spec):
                    find_goal = True

        if find_goal is True:
            result = init_visited[current]
            # return result
            new_list = []
            for i in range(len(result) - 1):
                angles1 = result[i].ee1_angles
                angles2 = result[i + 1].ee1_angles
                if stage % 2 == 1:
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
                        if stage % 2 == 0:
                            new_angle_list.append(result[i].ee1_angles[k].__add__(angle_distance[k].__mul__(0.002 * j)))
                        elif stage % 2 == 1:
                            new_angle_list.append(result[i].ee2_angles[k].__add__(angle_distance[k].__mul__(0.002 * j)))

                        new_length_list.append(result[i].lengths[k] + length_distance[k] * 0.002 * j)

                    if stage % 2 == 0:
                        x, y = result[i].points[0]
                        new_config = make_robot_config_from_ee1(x, y, new_angle_list, new_length_list,
                                                                ee1_grappled=True)
                        new_list.append(new_config)
                    elif stage % 2 == 1:
                        x, y = result[i].points[-1]
                        new_config = make_robot_config_from_ee2(x, y, new_angle_list, new_length_list,
                                                                ee2_grappled=True)
                        new_list.append(new_config)

            return new_list

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]


def generate_sub_goals(spec, stage, obstacles):
    start_point = spec.grapple_points[stage]
    goal_point = spec.grapple_points[stage+1]

    sub_goals = []
    for i in range(10):
        find_goal = False

        while find_goal is False:
            sampling_angle = []
            sampling_length = []
            for j in range(spec.num_segments - 1):
                angle = random.uniform(-165, 165)
                sampling_angle.append(Angle(degrees=float(angle)))
                length = random.uniform(spec.min_lengths[j], spec.max_lengths[j])
                sampling_length.append(length)

            if stage % 2 == 0:
                x1, y1 = start_point
                x2, y2 = goal_point
                partial_config = make_robot_config_from_ee1(x1, y1, sampling_angle, sampling_length, ee1_grappled=True)
                if (partial_config.points[-1][0] - x2) ** 2 + (partial_config.points[-1][1] - y2) ** 2 < spec.max_lengths[spec.num_segments-1] ** 2:
                    sampling_length.append(math.sqrt((partial_config.points[-1][0] - x2) ** 2 + (partial_config.points[-1][1] - y2) ** 2))

                    required_net_angle = math.atan((partial_config.points[-1][1] - y2) / (partial_config.points[-1][0] - x2))
                    if y2 > partial_config.points[-1][1] and x2 < partial_config.points[-1][0]:
                        required_net_angle += math.pi
                    elif y2 < partial_config.points[-1][1] and x2 < partial_config.points[-1][0]:
                        required_net_angle -= math.pi
                    current_net_angle = 0

                    for angle in sampling_angle:
                        current_net_angle += angle.in_radians()
                    sampling_angle.append(Angle(radians=float(required_net_angle-current_net_angle)))

                    sub_goal_config = make_robot_config_from_ee1(x1, y1, sampling_angle, sampling_length, ee1_grappled=True, ee2_grappled=True)
                    if test_obstacle_collision(sub_goal_config, spec, obstacles) and test_self_collision(sub_goal_config, spec)\
                            and test_environment_bounds(sub_goal_config):

                        sub_goals.append(sub_goal_config)
                        find_goal = True

            else:
                x1, y1 = start_point
                x2, y2 = goal_point
                partial_config = make_robot_config_from_ee2(x1, y1, sampling_angle, sampling_length, ee2_grappled=True)

                if (partial_config.points[0][0] - x2) ** 2 + (partial_config.points[0][1] - y2) ** 2 < spec.max_lengths[0] ** 2:
                    sampling_length.insert(0, math.sqrt((partial_config.points[0][0] - x2) ** 2 + (partial_config.points[0][1] - y2) ** 2))
                    required_net_angle = math.atan((partial_config.points[0][1] - y2) / (partial_config.points[0][0] - x2))
                    if required_net_angle < 0:
                        required_net_angle += math.pi
                    current_net_angle = 0
                    for angle in sampling_angle:
                        current_net_angle += angle.in_radians()
                    sampling_angle.append(Angle(radians=float(required_net_angle-current_net_angle)))
                    sub_goal_config = make_robot_config_from_ee2(x1, y1, sampling_angle, sampling_length, ee1_grappled=True, ee2_grappled=True)

                    if test_obstacle_collision(sub_goal_config, spec, obstacles) and test_self_collision(sub_goal_config, spec)\
                            and test_environment_bounds(sub_goal_config):
                        sub_goals.append(sub_goal_config)
                        find_goal = True

    return sub_goals


def main(arglist):
    start_time = time.time()
    input_file = arglist[0]
    spec = ProblemSpec(input_file)
    obstacles = __get_lenient_obstacles(spec)

    solution = []
    if spec.num_grapple_points == 1:
        solution += create_state_graph(spec, obstacles, 0)
    else:
        for stage in range(spec.num_grapple_points):
            if stage == 0:
                sub_goals = generate_sub_goals(spec, stage, obstacles)
                list1 = create_state_graph(spec, obstacles, stage, sub_goals=sub_goals)
                if list1 is None:
                    print("step1 no solution")
                else:
                    solution += list1
            elif 0 < stage < spec.num_grapple_points - 1:
                sub_goals = generate_sub_goals(spec, stage, obstacles)
                list2 = create_state_graph(spec, obstacles, stage, sub_init=solution[-1], sub_goals=sub_goals)
                if list2 is None:
                    print("step2 no solution")
                else:
                    solution += list2
            else:
                list3 = create_state_graph(spec, obstacles, stage, sub_init=solution[-1])
                if list3 is None:
                    print("step3 no solution")
                else:
                    solution += list3

    print("time:" + str(time.time()-start_time))
    write_robot_config_list_to_file("filename", solution)


if __name__ == '__main__':
    main(sys.argv[1:])
