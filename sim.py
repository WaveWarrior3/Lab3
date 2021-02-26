from math import sin, cos, pi, atan
import numpy as np
import matplotlib.pyplot as plt
import csv



# Hyperparameters (all lengths in meters)
#L = 10  # Length of arena
#H = 10  # Width of arena

B_x = 0     # Earth's magnetic field, replace later
B_y = 1


# Paperbot
d_paper = 0.05    # wheel diameter
w_paper = 0.09    # width of robot


# Segway
d_seg = 0.502
w_seg = 0.53




# run a full simulation
def simulate(input_list, init_state, delta_t, d, w, H, L):
    state_list = []
    sensor_list = []
    
    state = init_state
    for inputs in input_list:
        state_list.append(state)
        state, sensor = simulate_step(state, inputs, delta_t, d, w, H, L)
        sensor_list.append(sensor)
    
    return state_list, sensor_list


# simulate one time step
def simulate_step(state_k, input_k, delta_t, d, w, H, L):
    '''
    Inputs:
    state_t: tuple of x, y, and theta at time k, in that order
    input_t: tuple of left wheel velocity and right wheel velocity, in that order

    Outputs:
    next_state: x, y, and theta at time k+1
    readings: sensor readings for l1, l2, big_omega, b1, and b2
    '''


    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    omegaL_k = input_k[0]
    omegaR_k = input_k[1]



    # state update
    x_next = x_k + (-((d*delta_t/4)*sin(theta_k))*omegaR_k) + (((d*delta_t/4)*sin(theta_k))*omegaL_k)    #no noise component yet
    y_next = y_k + (-((d*delta_t/4)*cos(theta_k))*omegaR_k) + (((d*delta_t/4)*cos(theta_k))*omegaL_k)    #no noise component yet
    theta_next = theta_k + ((d*delta_t/(2*w))*omegaR_k) + ((d*delta_t/(2*w))*omegaL_k)                  #no noise component yet

    # bound x between 0 and L (avoid divide by 0)
    if x_next < 0.001:
        x_next = 0.001 
    elif x_next > L-0.001:
        x_next = L-0.001

    # bound y between 0 and H
    if y_next < 0.001:
        y_next = 0.001
    elif y_next > H-0.001:
        y_next = H-0.001

    # normalize theta between 0 and 2*pi
    if theta_next < 0.0:
        theta_next += 2*pi
    elif theta_next > 2*pi:
        theta_next -= 2*pi

    
    # find regions for LIDAR readings
    regionF = None
    regionR = None


    if 0 <= theta_k <= atan((L-x_k)/(H-y_k)) or (3*pi/2 + atan((H-y_k)/x_k)) <= theta_k <= 2*pi:
        regionF = 1
    elif atan((L-x_k)/(H-y_k)) <= theta_k <= (pi/2 + atan(y_k/(L-x_k))):
        regionF = 2
    elif (pi/2 + atan(y_k/(L-x_k))) <= theta_k <= (pi + atan(x_k/y_k)):
        regionF = 3
    elif (pi + atan(x_k/y_k)) <= theta_k <= (3*pi/2 + atan((H-y_k)/x_k)):
        regionF = 4

    if regionF == None:
        print("Something has gone horribly wrong with Front LIDAR")
        return


    if 0 <= theta_k <= atan(y_k/(L-x_k)) or (3*pi/2 + atan((L-x_k)/(H-y_k))) <= theta_k <= 2*pi:
        regionR = 1
    elif atan(y_k/(L-x_k)) <= theta_k <= (pi/2 + atan(x_k/y_k)):
        regionR = 2
    elif (pi/2 + atan(x_k/y_k)) <= theta_k <= (pi + atan((H-y_k)/x_k)):
        regionR = 3
    elif (pi + atan((H-y_k)/x_k)) <= theta_k <= (3*pi/2 + atan((L-x_k)/(H-y_k))):
        regionR = 4

    if regionR == None:
        print("Something has gone horribly wrong with Right LIDAR")
        return



    # Take Readings
    l1_k = f1(state_k, regionF, H, L)
    l2_k = f2(state_k, regionR, H, L)
    

    '''
    # Take Readings
    l1_k = H - y_k
    l2_k = L - x_k
    '''
    

    big_omega_k = (d/(2*w))*(omegaR_k+omegaL_k)
    if big_omega_k < 0.0:
        big_omega_k += 2*pi
    elif big_omega_k > 2*pi:
        big_omega_k -= 2*pi

    b1_k = B_x*cos(theta_k) - B_y*sin(theta_k)
    b2_k = B_x*sin(theta_k) + B_y*cos(theta_k)

    next_state = (x_next, y_next, theta_next)
    sensor_readings = (l1_k, l2_k, big_omega_k, b1_k, b2_k)

    return next_state, sensor_readings


def f1(state_k, regionF, H, L):
    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    if regionF == 1:
        return ((H-y_k)/cos(theta_k))
    elif regionF == 2:
        return ((L-x_k)/sin(theta_k))
    elif regionF == 3:
        return(-y_k/cos(theta_k))
    elif regionF == 4:
        return(-x_k/sin(theta_k))
    else:
        print("Bad Front Region")
        return



def f2(state_k, regionR, H, L):
    x_k = state_k[0]
    y_k = state_k[1]
    theta_k = state_k[2]

    if regionR == 1:
        return ((L-x_k)/cos(theta_k))
    elif regionR == 2:
        return (y_k/sin(theta_k))
    elif regionR == 3:
        return(-x_k/cos(theta_k))
    elif regionR == 4:
        return(-(H-y_k)/sin(theta_k))
    else:
        print("Bad Right Region")
        return



    
def paperbot():
    #PAPERBOTS

    # Matthew
    init_state = (0.0, 0.0, 45.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/matthew_right.txt')
    left_wheel = read_data('wheel_data/matthew_left.txt')



    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_paper, w_paper)
    plot_path(state_list, "Matthew Paperbot")

    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)




    # Ryan
    init_state = (0.0, 0.0, 45.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/ryan_right.txt')
    left_wheel = read_data('wheel_data/ryan_left.txt')


    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_paper, w_paper)
    plot_path(state_list, "Ryan Paperbot")
    
    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)


    
 
    # Remy
    init_state = (0.0, 0.0, 45.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/remy_right.txt')
    left_wheel = read_data('wheel_data/remy_left.txt')


    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_paper, w_paper)
    plot_path(state_list, "Remy Paperbot")
    
    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)

    





    # Gwen
    init_state = (0.2, 0.3, 0.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/gwen_right.txt')
    left_wheel = read_data('wheel_data/gwen_left.txt')


    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_paper, w_paper)
    plot_path(state_list, "Gwen Paperbot")
    
    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)





def segway():
    # SEGWAY

    # Matthew
    init_state = (0.0, 0.0, 45.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/matthew_right.txt')
    left_wheel = read_data('wheel_data/matthew_left.txt')



    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_seg, w_seg)
    plot_path(state_list, "Matthew Segway")

    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)




    # Ryan
    init_state = (0.0, 0.0, 45.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/ryan_seg_right.txt')
    left_wheel = read_data('wheel_data/ryan_seg_left.txt')


    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_seg, w_seg)
    plot_path(state_list, "Ryan Segway")
    
    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)



    

    # Remy
    init_state = (0.0, 0.0, 45.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/remy_right.txt')
    left_wheel = read_data('wheel_data/remy_left.txt')


    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_seg, w_seg)
    plot_path(state_list, "Remy Segway")
    
    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)





    # Gwen
    init_state = (2.0, 2.0, 0.0 * np.pi / 180)
    right_wheel = read_data('wheel_data/gwen_right.txt')
    left_wheel = read_data('wheel_data/gwen_left.txt')


    input_list = []
    for i in np.arange(len(left_wheel)):
        left_rad = left_wheel[i] * np.pi / 180
        right_rad = right_wheel[i] * np.pi / -180
        input_list.append((left_rad, right_rad))

    delta_t = 0.01   #length of time step

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_seg, w_seg)
    plot_path(state_list, "Gwen Segway")
    
    x_coord = []
    y_coord = []

    for i in np.arange(len(state_list)):
        x_coord.append(state_list[i][0] - init_state[0])
        y_coord.append(state_list[i][1] - init_state[1])

    print(x_coord)
    print()
    print(y_coord)
    



def read_data(datafile):
    f = open(datafile, 'r')
    data_str = f.readlines()
    data = []
    for i in np.arange(len(data_str)):
        data_str[i] = data_str[i].replace('\n', '')
        data.append(float(data_str[i]))

    
    return data


def write_data(filename, data):
    f = open(filename, 'w')
    data_str = []
    for i in np.arange(len(data)):
        data_str.append(str(data[i]) + '\n')
    f.writelines(data_str)


def plot_path(state_list, H, L, title=""):
    x = []
    y = []

    for states in state_list:
        x.append(states[0])
        y.append(states[1])

    f = plt.figure(1)
    plt.xlim((0,L))
    plt.ylim((0,H))
    f.set_figheight(5)
    f.set_figwidth(5)
    plt.plot(x,y)
    plt.title(title)
    plt.show()


def plot_sensors(sensor_data, title=""):
    front_sensor = []
    right_sensor = []
    for i in range(len(sensor_data)):
        front_sensor.append(sensor_data[i][0])
        right_sensor.append(sensor_data[i][1])

    x = range(len(sensor_data))
    f = plt.figure(1)
    f.set_figheight(5)
    f.set_figwidth(5)
    plt.plot(x, front_sensor)
    plt.plot(x, right_sensor)
    plt.title(title)
    plt.show()


def plot_sensors2(sensor_data, title=""):
    gyro = []
    angle1 = []
    angle2 = []
    for i in range(len(sensor_data)):
        gyro.append(sensor_data[i][2])
        angle1.append(sensor_data[i][3])
        angle2.append(sensor_data[i][4])

    x = range(len(sensor_data))
    f = plt.figure(1)
    f.set_figheight(5)
    f.set_figwidth(5)
    plt.plot(x, gyro)
    plt.plot(x, angle1)
    plt.plot(x, angle2)
    plt.title(title)
    plt.show()



def run_sim(init_state, input_file, output_file, delta_t=0.001, paper=False):
    g = open(input_file, 'r')
    reader = csv.reader(g, delimiter=',')
    data = []
    for row in reader:
        data.append(row)

    for i in range(20000):
        data[i][0] = float(data[i][0])
        data[i][1] = float(data[i][1])

    input_list = []
    for i in range(20000):

        input_list.append((-1*data[i][0], data[i][1]))




    if paper:
        H = 1
        L = 1
        state_list, sensor_list = simulate(input_list, init_state, delta_t, d_paper, w_paper, H, L)
        plot_path(state_list, H, L)
        plot_sensors(sensor_list)

        f = open(output_file, 'w')
        writer = csv.writer(f)
        for i in range(20000):
            writer.writerow(sensor_list[i])
        f.close()

    else:
        H = 10
        L = 10
        state_list, sensor_list = simulate(input_list, init_state, delta_t, d_seg, w_seg, H, L)
        plot_path(state_list, H, L)
        plot_sensors(sensor_list)

        f = open(output_file, 'w')
        writer = csv.writer(f)
        for i in range(20000):
            writer.writerow(sensor_list[i])
        f.close()
    
    


if __name__ == "__main__":


    

    #run_sim((0.1, 0.1, 45.0 * np.pi / 180), 'Input_Data/complex_inputs_2_paperbot.csv', 'sim_data/EE_complex_paper_2.csv', paper=True)
    #run_sim((0.1, 0.1, 45.0 * np.pi / 180), 'Input_Data/complex_inputs_3_paperbot.csv', 'sim_data/EE_complex_paper_3.csv', paper=True)
    run_sim((1.0, 1.0, 45.0 * np.pi / 180), 'Input_Data/complex_inputs_2_segway.csv', 'sim_data/EE_complex_segway_2.csv', paper=False)
    run_sim((1.0, 1.0, 45.0 * np.pi / 180), 'Input_Data/complex_inputs_3_segway.csv', 'sim_data/EE_complex_segway_3.csv', paper=False)



    '''
    # first sim
    
    init_state = (1.0, 1.0, 0.0 * np.pi / 180)
    delta_t = 0.001
    input_list = []
    for i in range(20000):
        input_list.append((2.0, -1.75))

    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_seg, w_seg, H, L)
    plot_path(state_list, H, L)
    plot_sensors(sensor_list)

    f = open('sim_data/EE_first_sim_segway_Lab3.csv', 'w')
    writer = csv.writer(f)
    for i in range(20000):
        writer.writerow(sensor_list[i])


    # second sim
    init_state = (9.0, 5.0, 0.0 * np.pi / 180)
    delta_t = 0.001

    g = open('spin_straight_data.csv', 'r')
    reader = csv.reader(g, delimiter=',')
    data = []
    for row in reader:
        data.append(row)
    
    input_list = []
    for i in range(20000):
        input_list.append((float(data[0][i]), -1 * float(data[1][i])))


    state_list, sensor_list = simulate(input_list, init_state, delta_t, d_paper, w_paper)
    plot_path(state_list)
    plot_sensors2(sensor_list)

    f = open('sim_data/EE_second_sim_paper_Lab3.csv', 'w')
    writer = csv.writer(f)
    for i in range(20000):
        writer.writerow(sensor_list[i])
    f.close()
    '''