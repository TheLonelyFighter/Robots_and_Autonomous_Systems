from djitellopy import Tello
import time


point = [100,100,100]

current_x = 0
current_y = 0
current_z = 0

direction_x = ""
direction_y = ""
direction_z = ""

break_X = False
break_Y = False
break_Z = False

dt = 0.1

while True:
    Vx = 0
    Vy = 0
    Vz = 0
    Yaw = 0

    if (point[0] > current_x):
        direction_x = "pos"
    elif (point[0] < current_x):
        direction_x = "neg"

    if (point[1] > current_y):
        direction_y = "pos"
    elif (point[1] < current_y):
        direction_y = "neg"

    if (point[2] > current_z):
        direction_z = "pos"
    elif (point[2] < current_z):
        direction_z = "neg"

    if(point[0] -10 < current_x <= point[0]+10):
            Vx = 0
            break_X = True
            print("Vx Done")
    elif ((point[0] -20) < current_x <= (point[0]+20)):
        if direction_x == "pos":
            Vx = 5
        elif direction_x == "neg":
            Vx = -5
    elif ((point[0] -50) < current_x <= (point[0]+50)):
        if direction_x == "pos":
            Vx = 10
        elif direction_x == "neg":
            Vx = -10
    else:
        if direction_x == "pos":
            Vx = 30
        elif direction_x == "neg":
            Vx = -30

    if( (point[1] - 10) < current_y <= (point[1]+10) ):
        Vy = 0
        break_Y = True
        print("Vy Done")
    elif ((point[1] - 20) < current_y <= (point[1]+20)):
        if direction_y == "pos":
            Vy = 5
        elif direction_y == "neg":
            Vy = -5
    elif ((point[1] - 50) < current_y <= (point[1]+50)):
        if direction_y == "pos":
            Vy = 10
        elif direction_y == "neg":
            Vy = -10
    else:
        if direction_y == "pos":
            Vy = 30
        elif direction_y == "neg":
            Vy = -30

    if( (point[2] -10) < current_z <= (point[2]+10) ):
        Vz = 0
        break_Z = True
        print("Vz  Done")
    elif ((point[2] -50) < current_z <= (point[2]+50)):
        if direction_z == "pos":
            Vz = 10
        elif direction_z == "neg":
            Vz = -10
    else:
        if direction_z == "pos":
            Vz = 30
        elif direction_z == "neg":
            Vz = -30

    if(break_X and break_Y and break_Z):
        print("Done")
        break

    current_x += Vx*dt
    current_y += Vy*dt
    current_z += Vz*dt

    print(Vx,Vy,Vz, current_x, current_y, current_z)

    time.sleep(dt)