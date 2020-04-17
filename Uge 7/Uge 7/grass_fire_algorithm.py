import numpy as np
from matplotlib import colors
import matplotlib.pyplot as plt
import os

class Cell:
    def __init__(self):
        self._dist = np.inf
        self._checked = False
        self._val = 0
        self._start = False
        self._goal = False

def close_event():
    plt.close()

def point_connectivity(grid, cell, point_connectivity=8):
    """
    This function will check neighbours of investigating cell
    :param grid: Whole map matrix
    :param cell: cell to investigate in map
    :param point_connectivity: Which point connectivity to use
    :return: neighbours of cell
    """
    neighbours = []
    shape = grid.shape
    x, y = cell
    if point_connectivity == 8:
        for i in range(-1, 2):
            if i == -1 or i == 1:
                x_diag = True
            else:
                x_diag = False
            for j in range(-1, 2):
                if j == -1 or j == 1:
                    y_diag = True
                else:
                    y_diag = False

                if not (i == 0 and j == 0):
                    if (x + i) >= 0 and (x + i) < shape[0] and (y + j) >= 0 and (y + j) < shape[1]:
                        if x_diag and y_diag:
                            neighbours.append([(x+i, y+j), True])
                        else:
                            neighbours.append([(x+i, y+j), False])

    elif point_connectivity == 4:
        # up
        if y + 1 < shape[1]:
            neighbours.append([(x, y + 1), False])
        # Down
        if y - 1 >= 0:
            neighbours.append([(x, y - 1), False])
        # Right
        if x + 1 < shape[0]:
            neighbours.append([(x + 1, y), False])

        # Left
        if x - 1 >= 0:
            neighbours.append([(x - 1, y), False])

    return neighbours

def print_map(map, dist=False):
    rows, columns = map.shape
    for i in range(rows):
        list_to_print = []
        for j in range(columns):
            if not dist:
                list_to_print.append(map[i, j]._val)
            else:
                list_to_print.append(map[i, j]._dist)
        print(list_to_print)

def initialize_map(shape):
    map = np.zeros(shape, dtype=object)
    for i in range(shape[0]):
        for j in range(shape[1]):
            map[i, j] = Cell()

    return map

def make_distance_map(node_que, map):
    """
    This function will calculate the distance map
    :param node_que: The node que should contain the goal cell coordinates
    :param map: This is the whole map
    :return: returns the map with distance calculations and start_point coordinate
    """
    start_point_reached = False
    start_point_idx = None
    checked_points = 0
    while len(node_que) > 0 and not start_point_reached:
        point_inve = node_que.pop(0)

        # Check if the element of the node que is a list of points or a single point
        if isinstance(point_inve, list):
            points_to_check = len(point_inve)
            iteration_idx = True
        else:
            iteration_idx = False
            points_to_check = 1

            # If there is only a single point check if it is goal
            if map[point_inve]._goal:
                cur_d = 0
                map[point_inve]._checked = 2
                map[point_inve]._dist = cur_d
                checked_points += 1
        # plot_grid_map(map)

        points_to_search_list = []
        for i in range(points_to_check):
            if iteration_idx:
                point_search = point_inve[i]
            else:
                point_search = point_inve


            if map[point_search]._start:
                start_point_idx = point_search
                start_point_reached = True
                map[point_search]._checked = 3
                # plot_grid_map(map)
                break

            if checked_points == (map.shape[0]+1) * (map.shape[1]+1):
                print("Points are exhausted")

            add_node_list = False
            neighbour_points = point_connectivity(map, point_search, point_connectivity=4)
            cur_d = map[point_search]._dist

            # print(f'Checking points: {point_search}')
            # Check the distance of neighbours and give the distance
            for point, diag in neighbour_points:
                if map[point]._val == 0:
                    add_node_list = True
                    if not diag:
                        dist = round(cur_d + 1, 2)
                        if dist < map[point]._dist:
                            map[point]._dist = dist
                    else:
                        dist = round(cur_d + 1.4142, 2)
                        if dist < map[point]._dist:
                            map[point]._dist = dist

                    checked_points += 1
                    if not map[point]._checked:
                        points_to_search_list.append(point)
                        map[point]._checked = True

                elif map[point]._val == 1:
                    map[point]._dist = np.inf
                    map[point]._checked = 5

                else:
                    add_node_list = False

        if add_node_list:
            node_que.append(points_to_search_list)


    return map, start_point_idx

def plot_grid_map(map):
    shape = map.shape
    # Data
    data = return_matrix_vals(map, val='checked')

    # create discrete colormap
    cmap = colors.ListedColormap(['red', 'blue', 'yellow', 'green', 'black'])
    bounds = [0, 1, 2, 3, 4, 6]
    norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    timer = fig.canvas.new_timer(interval=4000)
    timer.add_callback(close_event)
    ax.imshow(data, cmap=cmap, norm=norm)

    # draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    ax.set_xticks(np.arange(-.5, shape[0], 1))
    ax.set_yticks(np.arange(-.5, shape[1], 1))

    # timer.start()
    plt.show()

def plot_vec_direction(vec):
    fig, ax = plt.subplots()
    timer = fig.canvas.new_timer(interval=4000)
    timer.add_callback(close_event)
    plt.quiver(0, 0, vec[0], vec[1], color=['r'], scale=21)
    ax.grid()
    timer.start()
    plt.show()

def return_matrix_vals(map, val='checked'):
    data = np.zeros(map.shape)

    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if val == 'checked':
                data[i, j] = map[i, j]._checked
            elif val == 'value':
                data[i, j] = map[i, j]._val

    return data

def change_map_vals(map, data, val='value'):
    assert(map.shape == data.shape), "They must have the same shape matrix form"
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            if val == 'value':
                map[i, j]._val = data[i, j]

    return map

def find_route(map, start_position):
    """
    Will find the shortest route of a distance map
    :param map: Map of cell objects that consists of Cell()
    :param start_position: The start position in the map
    :return: path finding array
    """
    dist = map[start_position]._dist
    goal_position_reached = False
    exhausted_cells = False
    position = start_position
    waypoint_array = []
    while (not goal_position_reached) and (not exhausted_cells):
        neighbours = point_connectivity(map, position, point_connectivity=8)

        for point, diag in neighbours:

            if map[point]._dist < dist:
                position = point
                dist = map[point]._dist


        if map[position]._goal:
            print("Goal Reached")
            goal_position_reached = True

        map[position]._checked = 3
        waypoint_array.append(position)
        plot_grid_map(map)
    # plot_grid_map(map)
    return waypoint_array

def write_smrcl_waypoint_drive_script(waypoint_array, start_position, map_shape, grid_size = 5):
    """

    :param waypoint_array: an array with the points in the grid the robot needs to move in
    :param start_position: The posiiton the robot starts from within the grid
    :param grid_size: The size of the grid.
    :return:
    """
    if not os.path.exists('waypoints_scripts'):
        os.mkdir('waypoints_scripts')

    file = open("./waypoints_scripts/smrcl_pathfinding.smr", "w+")

    print(f'start position x: {start_position[0] * grid_size/100}')
    print(f'start position y: {(map_shape[1] - start_position[1]) * grid_size/100}')
    file.write(f'$odox = {start_position[0] * grid_size/100}\n')
    file.write(f'$odoy = {(map_shape[1] - start_position[1]) * grid_size/100}\n')
    last_pos = start_position
    for waypoint in waypoint_array:
        vec = np.asarray(waypoint) - np.asarray(last_pos)
        vec[0] = -1*vec[0]
        theta = np.arctan2(vec[0], vec[1])
        theta = round(np.arctan2(np.sin(theta), np.cos(theta))*180/np.pi, 2)
        x = round(waypoint[0] * grid_size, 2)/100
        y = round((map_shape[1] - waypoint[1]) * grid_size, 2)/100

        line = "drive %.1f %.1f %.1f :($targetdist<0.02)\n" % (round(x, 1), round(y, 1), round(theta, 1))
        file.write('ignoreobstacles\n')
        file.write(line)
        last_pos = waypoint
        # plot_vec_direction(vec)


# Que list for nodes to be investigated
node_que = []
shape = (10, 10)
map = initialize_map(shape)
start_point = 9, 9
goal_point = 2, 2

points = return_matrix_vals(map, val='value')
points[0:7, 3] = 1
points[3:10, 0] = 1

map = change_map_vals(map, points, val='value')

map[goal_point]._val = 2
map[start_point]._start = True
map[goal_point]._goal = True
print_map(map)
node_que.append(goal_point)
map, start_point = make_distance_map(node_que, map)
print_map(map, dist=True)
waypoints = find_route(map, start_point)
write_smrcl_waypoint_drive_script(waypoints, start_point, map.shape)



