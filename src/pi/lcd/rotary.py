import pigpio

pi = pigpio.pi()

# -1: left transition, +1: right transition, 0: no transition and 14: impossible transition
TRANS = [0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0]
LEFT = 18
RIGHT = 19
PUSH = 13

def rotary():
    global lrmem
    global lrsum

    l = pi.read(LEFT)
    r = pi.read(RIGHT)
    lrmem = (lrmem % 4)*4 + 2*l + r
    lrsum = lrsum + TRANS[lrmem]
    # encoder not in the neutral state
    if(lrsum % 4 != 0): return(0)
    # encoder in the neutral state
    if (lrsum == 4):
        lrsum=0
        return(1)
    if (lrsum == -4):
        lrsum=0
        return(-1)
    # lrsum > 0 if the impossible transition
    lrsum=0
    return(0)

pi.set_mode(LEFT, pigpio.INPUT)
pi.set_mode(RIGHT, pigpio.INPUT)
pi.set_mode(PUSH, pigpio.INPUT)
pi.set_pull_up_down(LEFT, pigpio.PUD_UP)
pi.set_pull_up_down(RIGHT, pigpio.PUD_UP)
pi.set_pull_up_down(PUSH, pigpio.PUD_UP)

lrmem = 3
lrsum = 0
num = 0
print(num)

while(True):
    res = rotary()
    if (res!=0):
        num=num + res
        print(num)
    if(pi.read(PUSH)==0):
        break
