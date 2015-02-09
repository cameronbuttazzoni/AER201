import serial
import numpy as np
from matplotlib import pyplot as plt
import time
NAMES = ['ping_time: ', 'left_ir_time: ', 'prev_ir_left: ', 'right_ir_time: ',
         'prev_ir_right: ', 'initial_orient_robot: ', 'x_robot: ', 'y_robot: ',
         'orient_robot: ', 'next_ball_column: ', 'millis(): ', 'hoppers[0].x: ',
         'hoppers[0].y: ', 'hoppers[0].balls: ', 'hoppers[0].orient: ',
         'hoppers[1].x: ', 'hoppers[1].y: ', 'hoppers[1].balls: ',
         'hoppers[1].orient: ', 'hoppers[2].x: ', 'hoppers[2].y: ',
         'hoppers[2].balls: ', 'hoppers[2].orient: ', 'hoppers[3].x: ',
         'hoppers[3].y: ', 'hoppers[3].balls: ', 'hoppers[3].orient: ']

COLORS = ['b', 'r', 'g', 'k', 'm', 'c', 'y', 'w']

# CONSTS
number_of_graphs = 3
set_x_range = True
x_min = 0
x_max = 160 #gameboard width is 160cm
set_y_range = True
y_min = 0
y_max = 18000 #gameboard height is 180cm
#x_max_length = 2 #max number of x values stored
#y_max_length = 2 #max number of y values stored

show_all_vals = False
disp_vals = [] #If show_all_vals is false, show vals in this list
#Choose which serial values to plot
#Basic format (assumes 4 hoppers):

#0  ping_time, left_ir_time, prev_ir_left, right_ir_time, prev_ir_right,
#5  initial_orient_robot, x_robot, y_robot, orient_robot, next_ball_column,
#10 millis(), hoppers[0].x, hoppers[0].y, hoppers[0].balls, hoppers[0].orient,
#15 hoppers[1].x, hoppers[1].y, hoppers[1].balls, hoppers[1].orient,
#19 hoppers[2].x, hoppers[2].y, hoppers[2].balls, hoppers[2].orient,
#23 hoppers[3].x, hoppers[3].y, hoppers[3].balls, hoppers[3].orient

#detect_hoppers() format:
#27 cur_time, prev_dist, prev_dist2, cur_hopper, flag, ping_dist, count

check_x = [1, 1, 1] #list of x values to care about
check_y = [2, 4, 5] #list of y values to care about

# CREATE PLOT
ser = serial.Serial("COM3", 9600) #open serial communication with arduino
plt.ion() #set plotting to interactive mode
fig = plt.figure()
x_vals = [[] for x in range(number_of_graphs)]
y_vals = [[] for x in range(number_of_graphs)]
plt.scatter([], [])
if set_x_range:
    plt.xlim([x_min, x_max])
if set_y_range:
    plt.ylim([y_min, y_max])
ax1 = fig.add_subplot(111)

#PLOT DATA RECEIVED BY SERIAL
line = ser.readline().rstrip()
temp = time.clock()
while True:
    print "init: ", time.clock() - temp
    line = ser.readline().rstrip()
    print "serial: ", time.clock() - temp
    temp = time.clock()
    vals = line.split(',')
    print "strip: ",  time.clock() - temp
    temp = time.clock()
    for x in range(number_of_graphs):
        x_vals[x].append(float(vals[check_x[x]]))
        y_vals[x].append(float(vals[check_y[x]]))
    print "new vals: ", time.clock() - temp
    temp = time.clock()
    if show_all_vals:
        print line
    else:
        temp_str = ''
        for x in xrange(len(disp_vals)):
            temp_str += NAMES[disp_vals[x]]
            temp_str += str(vals[disp_vals[x]])
            temp_str += ' '
            if len(disp_vals) == x + 1:
                print temp_str
    print "disp: ", time.clock() - temp
    temp = time.clock()
    for x in range(number_of_graphs):
        plt.scatter(x_vals[x], y_vals[x], color = COLORS[x])
    print "scatter: ", time.clock() - temp
    temp = time.clock()
    plt.draw()
    print "draw: ", time.clock() - temp
    temp = time.clock()
    plt.pause(0.000001)

