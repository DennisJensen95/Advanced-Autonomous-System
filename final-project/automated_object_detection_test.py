from subprocess import Popen, PIPE
from fcntl import fcntl, F_GETFL, F_SETFL
import time
from os import O_NONBLOCK, read

def open_ulmsserver():
    """
    Open ulmserver
    :return:
    """
    ulmsserver = Popen(['ulmsserver'], stdout=PIPE, stdin=PIPE, universal_newlines=True)
    flags = fcntl(ulmsserver.stdout, F_GETFL)
    fcntl(ulmsserver.stdout, F_SETFL, flags | O_NONBLOCK)
    return ulmsserver

ulmsserver= open_ulmsserver()
start = time.time()
timeout = 15
while True:
    try:
        output = read(ulmsserver.stdout.fileno(), 1024)
    except OSError:
        print("No more data")
        break

    # if output == '' and ulmsserver.poll() is not None:
    #     break
    # if output:
    #     print(output.strip())

    if time.time() - start > timeout:
        time.sleep(1)
        ulmsserver.terminate()
        break

print("Done")