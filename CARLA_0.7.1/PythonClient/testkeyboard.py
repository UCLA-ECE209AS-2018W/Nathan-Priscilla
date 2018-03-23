import sys, time, tty, termios, os

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def get_keyboard_control(char):
    control = None
    if char == 'a':
        print 'got a'
        #control.steer = -1.0
    if char == 'd':
        print 'got d'
        #control.steer = 1.0
    if char == 'w':
        print 'got w'
        #control.throttle = 1.0
    if char == 's':
        print 'got s'
        #control.brake = 1.0
    if char == ' ':
        print 'got space'
        #control.hand_brake = True
    if char == 'q':
        print 'got q'
        #self._is_on_reverse = not self._is_on_reverse
    #control.reverse = self._is_on_reverse
    return control

control = None
while True:
    char = getch()
    control = get_keyboard_control(char)
    time.sleep(0.5)

