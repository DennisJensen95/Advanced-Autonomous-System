from subprocess import Popen, PIPE
import time
import os

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
    simserver = Popen(['simserver1', 'simconfig388proj.xml'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    return simserver

def run_mrc_script(mrc_script_path):
    """
    Run mrc script
    :return:
    """
    mrc_script = Popen(['mrc', '-s8000', f'{mrc_script_path}'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    return mrc_script

os.chdir('./../../test/')
ulmsserver = open_ulmsserver()
simserver = open_simserver()


time.sleep(10)
print("Done")