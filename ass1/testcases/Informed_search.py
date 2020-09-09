import sys
import copy
import time
import numpy as np
from sokoban_map import SokobanMap
from itertools import permutations


def goal_test(state):
    return state.is_finished()


def deadlock_checker(state):  # return True if the input state has no solution
    target_x_pos = []
    target_y_pos = []
    for target in state.tgt_positions:
        target_x_pos.append(target[0])
        target_y_pos.append(target[1])

    for box in state.box_positions:
        x_axis, y_axis = box[0], box[1]

        # If the upper row is a wall then detect if there is a target at the current row,
        # if not, then there is no solution
        is_top_wall = True
        for element in state.obstacle_map[x_axis - 1]:
            if element != '#':
                is_top_wall = False
                break
        if is_top_wall:
            if x_axis not in target_x_pos:
                return True

        # Check the lower row
        is_bot_wall = True
        for element in state.obstacle_map[x_axis + 1]:
            if element != '#':
                is_bot_wall = False
                break
        if is_bot_wall:
            if x_axis not in target_x_pos:
                return True

        # Check the left row
        is_left_wall = True
        i = 0
        while i < state.y_size:
            if state.obstacle_map[i][y_axis - 1] != '#':
                is_left_wall = False
                break
            i += 1
        if is_left_wall:
            if y_axis not in target_y_pos:
                return True

        # Check the right row
        is_right_wall = True
        i = 0
        while i < state.y_size:
            if state.obstacle_map[i][y_axis + 1] != '#':
                is_right_wall = False
                break
            i += 1
        if is_right_wall:
            if y_axis not in target_y_pos:
                return True

        # If the box is at corner but not on target, then there is no solution
        if state.obstacle_map[x_axis - 1][y_axis] == '#' and state.obstacle_map[x_axis][y_axis - 1] == '#':
            if box not in state.tgt_positions:
                return True
        if state.obstacle_map[x_axis - 1][y_axis] == '#' and state.obstacle_map[x_axis][y_axis + 1] == '#':
            if box not in state.tgt_positions:
                return True
        if state.obstacle_map[x_axis + 1][y_axis] == '#' and state.obstacle_map[x_axis][y_axis - 1] == '#':
            if box not in state.tgt_positions:
                return True
        if state.obstacle_map[x_axis + 1][y_axis] == '#' and state.obstacle_map[x_axis][y_axis + 1] == '#':
            if box not in state.tgt_positions:
                return True

    return False


class SokobanGame(SokobanMap):
    # return an array of successor nodes expanded from current node
    def __init__(self, filename):
        super(SokobanGame, self).__init__(filename)
        self.moved_steps = 0
        self.estimated_distance = 0

    def next_states(self):
        successor_states = []

        new_state1 = copy.deepcopy(self)
        if new_state1.apply_move('u'):  # if move is successful then append node
            successor_states.append(new_state1)
        else:  # or append none to indicate there is no path this way
            successor_states.append(None)

        new_state2 = copy.deepcopy(self)
        if new_state2.apply_move('d'):
            successor_states.append(new_state2)
        else:
            successor_states.append(None)

        new_state3 = copy.deepcopy(self)
        if new_state3.apply_move('l'):
            successor_states.append(new_state3)
        else:
            successor_states.append(None)

        new_state4 = copy.deepcopy(self)
        if new_state4.apply_move('r'):
            successor_states.append(new_state4)
        else:
            successor_states.append(None)

        return successor_states

    def pn(self):
        # Return a person distance heuristic
        player_pos = [(self.player_y, self.player_x)]
        pn_distance = []
        for dis_index in range(len(self.box_positions)):
            distance = 0
            for dx in np.array(player_pos) - np.array(self.box_positions[dis_index]):
                distance = sum(abs(dx))
            pn_distance.append(distance)
        player_min_distance_to_box = min(pn_distance) - 1  # 改成所有人到所有箱子的distance的总和
        return player_min_distance_to_box

    def hn(self):
        # Return a box distance heuristic
        permute_result = list(permutations(self.tgt_positions, len(self.box_positions)))
        distance_list = []
        for k in permute_result:
            individual_distance_list = []
            for k_index in range(len(k)):
                distance = sum([abs(dx) for dx in np.array(k[k_index]) - np.array(self.box_positions[k_index])])
                individual_distance_list.append(distance)
            distance_list.append(sum(individual_distance_list))
        heur_distance = min(distance_list)
        return heur_distance

    def get_heuristic(self):
        return self.hn() + self.pn()

    def get_estimated_distance(self):
        self.estimated_distance = self.moved_steps + self.get_heuristic()
        #print(self.estimated_distance)
        return self.estimated_distance


class NodeContainer:
    def __init__(self, state, actions):
        self.state = state
        self.actions = actions

    def get_successors(self):
        successor_nodes = []
        successor_states = self.state.next_states()

        if successor_states[0] is not None:
            successor_states[0].moved_steps += 1
            successor_states[0].get_estimated_distance()
            successor_nodes.append(NodeContainer(successor_states[0], self.actions + 'u'))
        if successor_states[1] is not None:
            successor_states[1].moved_steps += 1
            successor_states[1].get_estimated_distance()
            successor_nodes.append(NodeContainer(successor_states[1], self.actions + 'd'))
        if successor_states[2] is not None:
            successor_states[2].moved_steps += 1
            successor_states[2].get_estimated_distance()
            successor_nodes.append(NodeContainer(successor_states[2], self.actions + 'l'))
        if successor_states[3] is not None:
            successor_states[3].moved_steps += 1
            successor_states[3].get_estimated_distance()
            successor_nodes.append(NodeContainer(successor_states[3], self.actions + 'r'))

        return successor_nodes

def a_star_search(filename):
    start_time = time.time()
    initial = SokobanGame(filename)
    counter = 0
    i=0
    if goal_test(initial):
        print('Initial state is the goal state')
        return ''
    container = []
    container.append([NodeContainer(initial, '')])
    visited = set([])
    while len(container) > 0:
        if len(container[counter]) != 0:
            node = container[counter].pop(0)
##            index = np.argmin([node.state.hn() for node in container[counter]])
##            node = container[counter][index]
##            del container[counter][index]
            successor = node.get_successors()
            counter = counter+1
            if len(successor) !=0:
                container.append([])
                for next_node in successor:
                    map_state = ''
                    for box in next_node.state.box_positions:
                        map_state += str(box[0])
                        map_state += str(box[1])
                    map_state += str(next_node.state.player_x)
                    map_state += str(next_node.state.player_y)
                    if map_state not in visited:
                        if goal_test(next_node.state):
                            print('expended nodes ' + str(i))
                            print('fringe width ' + str(len(container)))
                            print('visited nodes ' + str(len(visited)))
                            print('run time ' + str(time.time() - start_time))
                            print(next_node.actions)

                            return next_node.actions
                        next_node.actions += ','
                        visited.add(map_state)
                        if not deadlock_checker(next_node.state):
                            container[counter].append(next_node)
##                    if goal_test(next_node.state):
##                        print('expended nodes ' + str(i))
##                        print('fringe width ' + str(len(container)))
##                        print('visited nodes ' + str(len(visited)))
##                        print('run time ' + str(time.time() - start_time))
##                        print(next_node.actions)
##
##                        return next_node.actions
##                    if map_state in visited:
##                        continue
##                    if deadlock_checker(next_node.state):
##                        continue
##                    else:
##                        next_node.actions += ','
##                        visited.add(map_state)
##                        container[counter].append(next_node)
                    
            else:
                counter = counter-1

        else:
            del container[counter]
            if counter>0:
                counter = counter-1
            else:
                return None
        i += 1
    return None

# def a_star_search(filename):
#     start_time = time.time()
#     initial = SokobanGame(filename)
#     if goal_test(initial):
#         print('Initial state is the goal state')
#         return ''
#     container = [NodeContainer(initial, '')]
#     visited = set([])
#     i = 0  # i implements whenever a node is expanded
#     while len(container) > 0:
#         index = np.argmin([node.state.estimated_distance for node in container])
#         node = container[index]
#         del container[index]
#         if goal_test(node.state):
#             print('expended nodes ' + str(i))
#             print('fringe width ' + str(len(container)))
#             print('visited nodes ' + str(len(visited)))
#             print('run time ' + str(time.time() - start_time))
#             return next_node.actions
#         successor = node.get_successors()
#         f ={}
#
#         for next_node in successor:
#             fx = next_node.state.estimated_distance
#             map_state = ''
#             for box in next_node.state.box_positions:
#                 map_state += str(box[0])
#                 map_state += str(box[1])
#             map_state += str(next_node.state.player_x)
#             map_state += str(next_node.state.player_y)
#
#             if map_state in visited:
#                 continue
#             if goal_test(next_node.state):
#                 print('expended nodes ' + str(i))
#                 print('fringe width ' + str(len(container)))
#                 print('visited nodes ' + str(len(visited)))
#                 print('run time ' + str(time.time() - start_time))
#                 print(next_node.actions)
#
#                 return next_node.actions
#             if deadlock_checker(next_node.state):
#                 continue
#             else:
#                 visited.add(map_state)
#                 f.update({next_node:fx})
#                 container.append(next_node)
##        if len(f)>0:
##            c=sorted(f.items(),key=lambda x:x[1])
##            min_node = c[0][0]
##            container.append(min_node)

       # i += 1
        #         if goal_test(next_node.state):
        #             print('expended nodes ' + str(i))
        #             print('fringe width ' + str(len(container)))
        #             print('visited nodes ' + str(len(visited)))
        #             print('run time ' + str(time.time() - start_time))
        #             return next_node.actions
        #         next_node.actions += ','
        #         visited.add(map_state)
        #         if not deadlock_checker(next_node.state):
        #             container.append(next_node)
        # i += 1
        # if i > 100000:
        #     return None
    # return None


def uniform_search(filename):
    start_time = time.time()
    initial = SokobanGame(filename)
    if goal_test(initial):
        print('Initial state is the goal state')
        return ''

    container = [NodeContainer(initial, '')]
    visited = set([])
    i = 0  # i implements whenever a node is expanded
    while len(container) > 0:
        node = container.pop(0)

        successor = node.get_successors()
        for next_node in successor:

            map_state = ''
            for box in next_node.state.box_positions:
                map_state += str(box[0])
                map_state += str(box[1])
            map_state += str(next_node.state.player_x)
            map_state += str(next_node.state.player_y)

            if map_state not in visited:
                if goal_test(next_node.state):
                    print('expended nodes ' + str(i))
                    print('fringe width ' + str(len(container)))
                    print('visited nodes ' + str(len(visited)))
                    print('run time ' + str(time.time() - start_time))
                    return next_node.actions
                next_node.actions += ','
                visited.add(map_state)
                if not deadlock_checker(next_node.state):
                    container.append(next_node)
        i += 1

    return None


def main(arglist):
    if len(arglist) != 1:
        print("Running this file find a shortest solution to sokoban using uniform search.")
        print("Usage: Uniform_search [map_file_name]")
        return
    #solution = uniform_search(arglist[0])
    solution = a_star_search(arglist[0])
    print(solution)  # todo: print it to txt file
    #print(len(solution))


if __name__ == '__main__':
    main(sys.argv[1:])
