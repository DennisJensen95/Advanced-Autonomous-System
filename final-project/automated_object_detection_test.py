from subprocess import Popen, PIPE
import random
import time
import os
import numpy as np

def open_ulmsserver():
    """
    Open ulmserver
    :return: Subprocess running ulmsserver
    """
    ulmsserver = Popen(['ulmsserver'],
                       stdout=PIPE, stderr=PIPE, universal_newlines=True)
    return ulmsserver

def open_simserver():
    """
    Open simserver
    :return: Subprocess running simulation server
    """
    simserver = Popen(['simserver1', 'automateEnvironment.xml'],
                      stdout=PIPE, stderr=PIPE, universal_newlines=True)
    return simserver

def run_mrc_script(mrc_script_path):
    """
    Run mrc script
    :return: Subprocess running mission script
    """
    mrc_script = Popen(['mrc', '-s8000', f'{mrc_script_path}'], universal_newlines=True)
    return mrc_script

def rotate(origin_angle, point):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy, angle = origin_angle
    px, py = point

    qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
    qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)
    return qx, qy

def generate_object(obj_num, point_start=None, _random=False):
    """
    Takes a starting point or will random choose one and generate the object
    corresponding with the object number given.
    :param obj_num: Object to genereate
    :param point_start: A point to generate the object from if not to generate random position
    :param _random: Do a random generation of the object
    :return:
    """
    if not _random:
        point_start = (2, 1.5, 0)
        x, y, theta = point_start
    elif point_start == None and _random:
        x = random.uniform(1.4, 2.6)
        y = random.uniform(1.4, 1.6)
        theta = random.uniform(0, 3.14)
        point_start = (x, y, theta)

    if obj_num == 1:
        lower_right_corner = rotate(point_start, (x + 0.40, y))
        upper_left_corner = rotate(point_start, (x, y + 0.15))
        diag_corner = rotate(point_start, (point_start[0] + 0.40, point_start[1] + 0.15))

        point_o = (np.asarray(point_start[:2]) + np.asarray(lower_right_corner) +
                   np.asarray(upper_left_corner) + np.asarray(diag_corner)) / 4


        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n' \
                       f'{upper_left_corner[0]}\t{upper_left_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n'
    elif obj_num == 2:
        lower_right_corner = rotate(point_start, (x + 0.30, y))
        upper_left_corner = rotate(point_start, (x, y + 0.20))
        diag_corner = rotate(point_start, (point_start[0] + 0.30, point_start[1] + 0.20))

        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n' \
                       f'{upper_left_corner[0]}\t{upper_left_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n'

        point_o = (np.asarray(point_start[:2]) + np.asarray(lower_right_corner) +
                   np.asarray(upper_left_corner) + np.asarray(diag_corner)) / 4

    elif obj_num == 3:
        lower_right_corner = rotate(point_start, (x + 0.40, y))
        upper_left_corner = rotate(point_start, (x, y + 0.10))
        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{upper_left_corner[0]}\t{upper_left_corner}\n'

        point_o = np.asarray(point_start[:2])

    elif obj_num == 4:
        lower_right_corner = rotate(point_start, (x + 0.30, y))
        upper_left_corner = rotate(point_start, (x, y + 0.15))
        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n'

        point_o = np.asarray(point_start[:2])



    return str_to_write, point_o, theta

map = f'0.0     0.0     1.8     0.0     bottom left\n' \
      f'2.2     0.0     4.0     0.0     bottom right\n' \
      f'0.0     5.0     1.8     5.0     top left\n' \
      f'2.2     5.0     4.0     5.0     top right\n' \
      f'0.0     0.0     0.0     1.8     left down\n' \
      f'0.0     3.2     0.0     5.0     left up\n' \
      f'4.0     0.0     4.0     1.8     right down\n' \
      f'4.0     3.2     4.0     5.0     right up\n' \
      f'0.9     3.1     1.7     3.1     maze bottom left\n' \
      f'2.3     3.1     3.1     3.1     maze bottom right\n' \
      f'1.7     2.5     1.7     3.1     maze left down\n' \
      f'0.9     3.1     0.9     4.3     maze left up\n' \
      f'2.3     2.5     2.3     3.1     maze right down\n' \
      f'3.1     3.1     3.1     4.3     maze right up\n' \
      f'1.5     3.7     2.5     3.7     maze middle horizontal\n' \
      f'2.0     3.7     2.0     4.3     maze middle vertical\n' \
      f'0.9     4.3     3.1     4.3     maze top\n'

os.chdir('./../../test/')

if os.path.exists('results_python.txt'):
    os.remove('results_python.txt')

if os.path.exists('results_cpp.txt'):
    os.remove('results_cpp.txt')

with open('results_python.txt', 'w+') as file:
    file.write('Object | Point o | Object pose\n')

with open('results_cpp.txt', 'w+') as file:
    file.write('Object | Point o x | point o y | Object pose | SSD\n')


iterations = 50
for j in range(iterations):
    for i in range(1, 5):
        object_string, point_o, theta = generate_object(i, _random=True)


        map_environ = map + object_string
        if os.path.exists('388auto'):
            os.remove('388auto')

        with open('./results_python.txt', 'a+') as file:
            file.write(f'{i}, {point_o}, {theta}\n')

        with open('./388auto', 'w+') as file:
            file.write(map_environ)

        with open('results_cpp.txt', 'a+') as file:
            file.write(f'-1 -1 -1 -1 -1\n')

        ulmsserver = open_ulmsserver()
        simserver = open_simserver()
        time.sleep(5)
        mrc_path = '../Advanced-Autonomous-System/final-project/final_project_v4'
        mrc_process = run_mrc_script(mrc_path)
        time.sleep(3)
        mrc_process.wait()
        mrc_process.terminate()
        ulmsserver.terminate()
        simserver.terminate()

        # with open('results_cpp.txt', 'r+') as file:
        #     lines = file.read().splitlines()
        #     last_line = lines[-1]
        #     if last_line == '-1 -1 -1 -1 -1':
        #         next_line = True
        #     else:
        #         next_line = False

        # if next_line:
        #     with open('results_cpp.txt', 'a+') as file:
        #         file.write('\n')

os.rename('results_cpp.txt', 'results_cpp_done.txt')
os.rename('results_python.txt', 'results_python_done.txt')

print("Done")
