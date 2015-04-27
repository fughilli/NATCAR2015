import serial, pygame, sys

try:
    ser = serial.Serial("COM13", 9600)
except Exception:
    print "Failed to open serial port."
    sys.exit()

pygame.init()

screen_size = (1280, 800)
black = 0,0,0
white = 255,255,255

b64arr = [chr(ord('A') + x) for x in range(26)]
b64arr.extend([chr(ord('a') + x) for x in range(26)])
b64arr.extend([chr(ord('0') + x) for x in range(10)])
b64arr.extend(['/','+'])

b64str = ''.join(b64arr)

screen = pygame.display.set_mode(screen_size)

def drawdatbuf(databuffer, color):
    for i in range(len(databuffer) - 1):
        pygame.draw.line(screen, color, (mapval(i, 0, 127, 0, screen_size[0]),
                                         screen_size[1] - mapval(databuffer[i], 0, 2**13, 0, screen_size[1])),
                         (mapval(i + 1, 0, 127, 0, screen_size[0]),
                                         screen_size[1] - mapval(databuffer[i + 1], 0, 2**13, 0, screen_size[1])), 3)

def mapval(val, inmin, inmax, outmin, outmax):
    if(not inmax == inmin):
        return (val * (outmax - outmin)) / (inmax - inmin)
    return 0

def bufavg(buf):
    average = 0
    for x in buf:
        average += x
    return (average / len(buf))

datbufstate = [0 for i in range(256)]

mappedxstate = [0 for x in range(10)]

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    databuffer = []
    dataline = ser.readline()[:-1]
    #print dataline
    for i in range(len(dataline)/2):
        try:
            databuffer.append(((b64arr.index(dataline[2*i]) & 0x3F) << 6) | (b64arr.index(dataline[2*i + 1]) & 0x3F))
        except Exception:
            databuffer.append(0)

    #print databuffer

    screen.fill(white)

    drawdatbuf(databuffer, (255,0,0))
    databufferdeltas = []
    
    for i in range(len(databuffer) - 1):
        databufferdeltas.append(abs(databuffer[i] - databuffer[i+1]))
        datbufstate[i] *= 0.1
        datbufstate[i] += databufferdeltas[i]*0.9

    drawdatbuf([x*5 for x in datbufstate], (0,0,255))

    maxval = max(datbufstate)
    minval = min(datbufstate)

    threshxs = []

    for (i, val) in enumerate(datbufstate):
        if((val - minval) > (0.65*(maxval - minval))):
            threshxs.append(i)

    average = bufavg(threshxs)

    mappedx = mapval(average, 0, 127, 0, screen_size[0])

    mappedxstate = mappedxstate[1:]
    mappedxstate.append(mappedx)

    linexpos = bufavg(mappedxstate)

    pygame.draw.line(screen, black, (linexpos, screen_size[1]),
                         (linexpos, 0), 3)

    drawdatbuf(databufferdeltas, (0,255,0))

    pygame.display.flip()
