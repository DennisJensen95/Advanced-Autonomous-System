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
        lower_right_corner = (x + np.cos(theta) * 0.40, y + np.sin(theta) * 0.40)
        upper_left_corner = (x + np.sin(theta) * 0.15, y + np.cos(theta) * 0.15)
        diag_corner = (lower_right_corner[0] + np.sin(theta) * 0.15, lower_right_corner[1] + np.cos(theta) * 0.15)

        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n' \
                       f'{upper_left_corner[0]}\t{upper_left_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n'
    elif obj_num == 2:
        lower_right_corner = (x + np.cos(theta) * 0.30, y + np.sin(theta) * 0.30)
        upper_left_corner = (x + np.sin(theta) * 0.20, y + np.cos(theta) * 0.20)
        diag_corner = (lower_right_corner[0] + np.sin(theta) * 0.20, lower_right_corner[1] + np.cos(theta) * 0.20)

        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n' \
                       f'{upper_left_corner[0]}\t{upper_left_corner[1]}\t{diag_corner[0]}\t{diag_corner[1]}\n'

    elif obj_num == 3:
        lower_right_corner = (x + np.cos(theta) * 0.40, y + np.sin(theta) * 0.40)
        upper_left_corner = (x + np.sin(theta) * 0.10, y + np.cos(theta) * 0.10)
        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{upper_left_corner[0]}\t{upper_left_corner}\n'

    elif obj_num == 4:
        lower_right_corner = (x + np.cos(theta) * 0.30, y + np.sin(theta) * 0.30)
        upper_left_corner = (x + np.sin(theta) * 0.15, y + np.cos(theta) * 0.15)
        str_to_write = f'{point_start[0]}\t{point_start[1]}\t{lower_right_corner[0]}\t{lower_right_corner[1]}\n' \
                       f'{point_start[0]}\t{point_start[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n' \
                       f'{lower_right_corner[0]}\t{lower_right_corner[1]}\t{upper_left_corner[0]}\t{upper_left_corner[1]}\n'

    return str_to_write

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

for i in range(1, 5):
    object_string = generate_object(i, _random=True)
    map_environ = map + object_string
    if os.path.exists('388auto'):
        os.remove('388auto')

    with open('./388auto', 'w+') as file:
        file.write(map_environ)

    ulmsserver = open_ulmsserver()
    simserver = open_simserver()

    time.sleep(5)

    ulmsserver.terminate()
    simserver.terminate()

print("Done")