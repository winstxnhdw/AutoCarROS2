#!/usr/bin/env python

import numpy as np

class QuinticPolynomial:

    def __init__(self, init_pos, init_vel, init_accel, final_pos, final_vel, final_accel, T):

        # Derived coefficients
        self.a_0 = init_pos
        self.a_1 = init_vel
        self.a_2 = init_accel / 2.0

        # Solve the linear equation (Ax = B)
        A = np.array([[T ** 3,       T ** 4,         T ** 5], 
                     [3 * T ** 2,    4 * T ** 3,     5 * T ** 4],
                     [6 * T,         12 * T ** 2,    20 * T ** 3]])

        B = np.array([final_pos - self.a_0 - self.a_1 * T - self.a_2 * T ** 2, final_vel - self.a_1 - init_accel * T, final_accel - init_accel])

        x = np.linalg.solve(A, B)

        self.a_3 = x[0]
        self.a_4 = x[1]
        self.a_5 = x[2]

    def calc_point(self, t):
        '''
        Calculate the point
        '''
        dt = self.a_0 + self.a_1 * t + self.a_2 * t ** 2 + self.a_3 * t ** 3 + self.a_4 * t ** 4 + self.a_5 * t ** 5

        return dt

    def calc_first_derivative(self, t):
        '''
        Calculate the velocity (m/s)
        '''
        dt = self.a_1 + 2 * self.a_2 * t + 3 * self.a_3 * t ** 2 + 4 * self.a_4 * t ** 3 + 5 * self.a_5 * t ** 4

        return dt

    def calc_second_derivative(self, t):
        '''
        Calculate the acceleration (m/s^2)
        '''
        dt = 2 * self.a_2 + 6 * self.a_3 * t + 12 * self.a_4 * t ** 2 + 20 * self.a_5 * t ** 3

        return dt

    def calc_third_derivative(self, t):
        '''
        Calculate the jerk (m/s^3)
        '''
        dt = 6 * self.a_3 + 24 * self.a_4 * t + 60 * self.a_5 * t ** 2

        return dt

class QuarticPolynomial:

    def __init__(self, init_pos, init_vel, init_accel, final_vel, final_accel, T):

        # Derived coefficients
        self.a_0 = init_pos
        self.a_1 = init_vel
        self.a_2 = init_accel / 2.0

        # Solve the linear equation (Ax = B)
        A = np.array([[3 * T ** 2,    4 * T ** 3],
                      [6 * T,         12 * T ** 2]])

        B = np.array([final_vel - self.a_1 - init_accel * T,
                      final_accel - init_accel])

        x = np.linalg.solve(A, B)

        self.a_3 = x[0]
        self.a_4 = x[1]

    def calc_point(self, t):
        '''
        Calculate the point
        '''
        st = self.a_0 + self.a_1 * t + self.a_2 * t ** 2 + self.a_3 * t ** 3 + self.a_4 * t ** 4

        return st

    def calc_first_derivative(self, t):
        '''
        Calculate the velocity (m/s)
        '''
        st = self.a_1 + 2 * self.a_2 * t + 3 * self.a_3 * t ** 2 + 4 * self.a_4 * t ** 3

        return st

    def calc_second_derivative(self, t):
        '''
        Calculate the acceleration (m/s^2)
        '''
        st = 2 * self.a_2 + 6 * self.a_3 * t + 12 * self.a_4 * t ** 2

        return st

    def calc_third_derivative(self, t):
        '''
        Calculate the jerk (m/s^3)
        '''
        st = 6 * self.a_3 + 24 * self.a_4 * t

        return st

class CubicPolynomial:

    def __init__(self, init_pos, init_vel, init_accel):
        
        # Derived coefficients
        self.d = init_pos
        self.c = init_vel
        self.b = init_accel / 2.0

def quintic_polynomial_planner(x_i, y_i, yaw_i, v_i, a_i, x_f, y_f, yaw_f, v_f, a_f, max_accel, max_jerk, dt):

    # Setting the time horizon
    max_time = 100.0
    min_time = 5.0
    
    # Calculate the velocity boundary conditions based on vehicle's orientation
    v_xi = v_i * np.cos(yaw_i)
    v_xf = v_f * np.cos(yaw_f)
    v_yi = v_i * np.sin(yaw_i)
    v_yf = v_f * np.sin(yaw_f)

    # Calculate the acceleration boundary conditions based on vehicle's orientation
    a_xi = a_i * np.cos(yaw_i)
    a_xf = a_f * np.cos(yaw_f)
    a_yi = a_i * np.sin(yaw_i)
    a_yf = a_f * np.sin(yaw_f)

    for T in np.arange(min_time, max_time, min_time):
        # Initialise the class
        xqp = QuinticPolynomial(x_i, v_xi, a_xi, x_f, v_xf, a_xf, T)
        yqp = QuinticPolynomial(y_i, v_yi, a_yi, y_f, v_yf, a_yf, T)

        # Instantiate/clear the arrays
        x = []
        y = []
        v = []
        a = []
        j = []
        yaw = []

        for t in np.arange(0.0, T + dt, dt):
            # Solve for position (m)
            x.append(xqp.calc_point(t))
            y.append(yqp.calc_point(t))

            # Solve for velocity (m/s)
            v_x = xqp.calc_first_derivative(t)
            v_y = yqp.calc_first_derivative(t)
            v.append(np.hypot(v_x, v_y))

            # Solve for orientation (rad)
            yaw.append(np.arctan2(v_y, v_x))

            # Solve for acceleration (m/s^2)
            a_x = xqp.calc_second_derivative(t)
            a_y = yqp.calc_second_derivative(t)

            if len(v) >= 2 and v[-1] - v[-2] < 0.0:
                a.append(-1.0 * np.hypot(a_x, a_y))

            else:
                a.append(np.hypot(a_x, a_y))
        
            # Solve for jerk (m/s^3)
            j_x = xqp.calc_third_derivative(t)
            j_y = yqp.calc_third_derivative(t)
            
            if len(a) >= 2 and a[-1] - a[-2] < 0.0:
                j.append(-1 * np.hypot(j_x, j_y))

            else:
                j.append(np.hypot(j_x, j_y))

        if max([abs(i) for i in a]) <= max_accel and max([abs(i) for i in j]) <= max_jerk:
            break

    return x, y, v, a, j, yaw

def quartic_polynomial_planner():
    pass

def cubic_polynomial_planner():
    pass

def plot():
    '''
    This function is for debugging and understanding purposes. Comment it out in main() if used as a library instead.
    '''
    # Initial values
    x_i = -10.0
    y_i = 10.0
    yaw_i = np.deg2rad(10.0)
    v_i = 4.0
    a_i = 0.1
    
    # Final values
    x_f = 30.0
    y_f = -10.0
    yaw_f = np.deg2rad(20.0)
    v_f = 0.0
    a_f = 0.1

    # Limits (Source: http://www.diva-portal.org/smash/get/diva2:839140/FULLTEXT01.pdf [Pg. 6])
    max_accel = 2.0
    max_jerk = 0.9
    dt = 0.1
    limits = max(abs(x_i), abs(x_f), abs(y_i), abs(y_f)) + 10.0

    x, y, v, a, j, yaw = quintic_polynomial_planner(x_i, y_i, yaw_i, v_i, a_i, x_f, y_f, yaw_f, v_f, a_f, max_accel, max_jerk, dt)

    print('no. of cells in x: {}'.format(len(x)))
    print('no. of cells in y: {}'.format(len(y)))

    import matplotlib.pyplot as plt

    plt.style.use('fivethirtyeight')
    plt.plot(x, y, label='Path', marker='.')
    plt.xlim(-limits, limits)
    plt.ylim(-limits, limits)
    plt.title('2D Path Trajectory')
    plt.xlabel('Horizontal Position (m)')
    plt.ylabel('Vertical Position (m)')
    plt.legend()
    plt.tight_layout()
    plt.show()

def main():

    plot()

if __name__ == '__main__':
    main()