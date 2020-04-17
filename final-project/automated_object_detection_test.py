from subprocess import Popen, PIPE
import select
import time


def open_ulmsserver():
    """
    Open ulmserver
    :return:
    """
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    return ulmsserver

ulmsserver= open_ulmsserver()
start = time.time()
timeout = 15
while True:
    output = ulmsserver.stdout.readline()
    if output == '' and ulmsserver.poll() is not None:
        break
    elif output == '':
        ulmsserver.stdin.write('\n')

    if output:
        print(output.strip())

    if time.time() - start > timeout:
        time.sleep(1)
        ulmsserver.terminate()
        break

print("Done")