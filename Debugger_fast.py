## AER201 Code Debugging Software
## Written by: Cameron Buttazzoni
## Team members: Yew Meng Khaw, Abhimanyu Joshi, James Tu
## Team Name: 1 Plus 1 Plus 1 Plus 1 Plus Rob the Robot
##
## Version 1:
##  - 1.1 Added real-time plotting of serial data
##  - 1.2 Improved speed (still only about once every ~0.12 seconds)
##  - 1.3 Automatically saves generated graph and serial input on exit
##  - 1.4 Considerable speed improvement (once every 0.05 seconds)
##
##
## Things to add:
##  - Output the selected variables on graphs
##
##

import serial
##import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import datetime
print "Finished import"
##import time
NAMES = ['ping_time: ', 'left_ir_time: ', 'prev_ir_left: ', 'right_ir_time: ',
         'prev_ir_right: ', 'initial_orient_robot: ', 'x_robot: ', 'y_robot: ',
         'orient_robot: ', 'next_ball_column: ', 'millis(): ', 'hoppers[0].x: ',
         'hoppers[0].y: ', 'hoppers[0].balls: ', 'hoppers[0].orient: ',
         'hoppers[1].x: ', 'hoppers[1].y: ', 'hoppers[1].balls: ',
         'hoppers[1].orient: ', 'hoppers[2].x: ', 'hoppers[2].y: ',
         'hoppers[2].balls: ', 'hoppers[2].orient: ', 'hoppers[3].x: ',
         'hoppers[3].y: ', 'hoppers[3].balls: ', 'hoppers[3].orient: ']

MONTHS = ["January", "February", "March", "April", "May", "June", "July",
          "August", "September", "October", "November", "December"]

COLORS = ['b', 'r', 'g', 'k', 'm', 'c', 'y', 'w']
serial_input = []

# CONSTS
save_folder = "debugger_output"
plot_delay = 0.00001 #set really really small
x_min = 0
x_max = 10000 #gameboard width is 160cm
y_min = 0
y_max = 100 #gameboard height is 180cm
limit_length = False #limit number of points shown on graph
max_length = 2 #max number of values stored

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

number_of_graphs = 1 #must be same as length of next 2 lists
check_x = [27] #list of x values to care about
check_y = [32] #list of y values to care about

# CREATE PLOT
plt.ion() #set plotting to interactive mode
fig = plt.figure()
ax = plt.axes(xlim = (x_min, x_max), ylim = (y_min, y_max))
lines = [ax.plot([], [], COLORS[x])[0] for x in range(number_of_graphs)]

x_vals = [[] for x in range(number_of_graphs)]
y_vals = [[] for x in range(number_of_graphs)]

#PLOT DATA RECEIVED BY SERIAL
ser = serial.Serial("COM4", 9600) #open serial communication with arduino
line = ser.readline().rstrip()
serial_input.append(line)

def init(): #initiate the lists to plot
    for x in xrange(number_of_graphs):
        lines[x].set_data(x_vals[x], y_vals[x])
    return lines

def animate(i):
##    print time.time()
    line = ser.readline().rstrip() #get serial data line and strip newline char
    serial_input.append(line) #save line to later write into file
    vals = line.split(',') #line is csv so split into list
    #print vals[27], vals[32]
    for x in xrange(number_of_graphs): #save appropriate values in their list
        x_vals[x].append(float(vals[check_x[x]]))
        y_vals[x].append(float(vals[check_y[x]]))
        if limit_length == True: #limit the number of values on graph
            if len(y_vals[x]) > max_length:
                y_vals[x] = y_vals[x][1:]
                x_vals[x] = x_vals[x][1:]
    for x in xrange(number_of_graphs): #plot the new values
        lines[x].set_data(x_vals[x], y_vals[x])
    if show_all_vals: #show all the values in the shell
        print line
    else: #show certain values in the shell
        temp_str = ''
        for x in xrange(len(disp_vals)):
            temp_str += NAMES[disp_vals[x]]
            temp_str += str(vals[disp_vals[x]])
            temp_str += ' '
            if len(disp_vals) == x + 1:
                print temp_str
    return lines

try:        
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   interval=plot_delay, blit=True)
    plt.show()
except KeyboardInterrupt: #save graph and serial data to files
    cur_time = datetime.datetime.now().strftime('%m-%d-%Y_%H-%M-%S') #get time
    image_file_name = save_folder + '/' + 'Debugger_graph_' + cur_time + '.png'
    plt.savefig(image_file_name) #save graph
    text_file_name = save_folder + '/' + 'Debugger_data_' + cur_time + '.csv'
    out_file = open(text_file_name, 'w') #create new text file to write to
    for x in range(len(NAMES)): #write header (only includes defaults)
        out_file.write(NAMES[x][:-2])
        if x < len(NAMES) - 1:
            out_file.write(',')
    for line in serial_input: #wite every line received from serial comm
        out_file.write(line + '\n')
    out_file.close()
    plt.close(fig) #close the now crashing figure
