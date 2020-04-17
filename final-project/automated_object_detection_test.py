from subprocess import Popen, PIPE
import random
import time
import os
import numpy as np

def open_ulmsserver():
    """
    Open ulmserver
    :return:
    """
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    return ulmsserver

def open_simserver():
    """
    Open simserver
    :return:
    """
    simserver = Popen(['simserver1', 'automateEnvironment.xml'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    return simserver

def run_mrc_script(mrc_script_path):
    """
    Run mrc script
    :return:
    """
    mrc_script = Popen(['mrc', '-s8000', f'{mrc_script_path}'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
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
                   np.asrray(upper_left_corner) + np.asarray(diag_corner)) / 4


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
                   np.asrray(upper_left_corner) + np.asarray(diag_corner)) / 4

    elif obj_num == 3:
        lower_right_corner = rotate(point_start, (x + 0.40, y))
        upper_left_corner = rotate(point_start, (x, y + 0.10))
        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{upper_left_corner[0]}\t{upper_left_corner}\n'

        point_o = point_start

    elif obj_num == 4:
        lower_right_corner = rotate(point_start, (x + 0.30, y))
        upper_left_corner = rotate(point_start, (x, y + 0.15))
        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n'

        point_o = point_start



    return str_to_write, point_o, theta

os.chdir('./../../test/')

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

iterations = 1
for j in range(iterations):
    for i in range(1, 5):
        object_string, point_o, theta = generate_object(i, _random=True)


        map_environ = map + object_string
        if os.path.exists('388auto'):
            os.remove('388auto')

        if os.path.exists('results_python.txt'):
            os.remove('results_python.txt')

        if os.path.exists('result.txt'):
            os.remove('result.txt')

        with open('./388auto', 'a+') as file:
            file.write(f'{i}, {point_o}, {theta}')

        with open('./388auto', 'w+') as file:
            file.write(map_environ)

        ulmsserver = open_ulmsserver()
        simserver = open_simserver()

        time.sleep(5)

        ulmsserver.terminate()
        simserver.terminate()

print("Done")