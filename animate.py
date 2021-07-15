import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from stanley_controller import StanleyController
from matplotlib.animation import FuncAnimation
from libs.kinematic_model import KinematicBicycleModel
from libs.car_description import Description
from libs.cubic_spline_pp import generate_cubic_path

class Simulation:

    def __init__(self):

        fps = 50.0

        self.dt = 1/fps
        self.map_size = 100
        self.frames = 1250
        self.loop = False

class Path:

    def __init__(self):

        # Get path to waypoints.csv
        dir_path = 'data/waypoints.csv'
        df = pd.read_csv(dir_path)

        x = df['X-axis'].values.tolist()
        y = df['Y-axis'].values.tolist()
        ds = 0.05

        self.px, self.py, self.pyaw, _ = generate_cubic_path(x, y, ds)

class Car:

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.v = 25.0
        self.delta = 0.0
        self.L = 2.5
        self.max_steer = np.deg2rad(33)
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 8.0
        self.ksoft = 1.0
        self.kyaw = 0.01
        self.ksteer = 0.0
        self.crosstrack_error = None
        self.target_id = None

        # Description parameters
        self.length = 4.5
        self.width = 2.0
        self.rear2wheel = 1.0
        self.wheel_dia = 0.15 * 2
        self.wheel_width = 0.2
        self.tread = 0.7

        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, self.max_steer, self.L, self.px, self.py, self.pyaw)
        self.kbm = KinematicBicycleModel(self.L, self.dt)

    def drive(self):
        
        self.delta, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.v, self.delta)
        self.x, self.y, self.yaw = self.kbm.kinematic_model(self.x, self.y, self.yaw, self.v, self.delta)

def main():
    
    sim = Simulation()
    path = Path()
    car = Car(path.px[0], path.py[0], path.pyaw[0], sim, path)
    desc = Description(car.length, car.width, car.rear2wheel, car.wheel_dia, car.wheel_width, car.tread, car.L)

    interval = sim.dt * 10**3

    fig, ax = plt.subplots(3, 1)

    ax[0].set_aspect('equal')
    ax[0].plot(path.px, path.py, '--', color='gold')

    annotation = ax[0].annotate('Crosstrack error: {}'.format('None'), xy=(car.x - 10, car.y + 5), color='black', annotation_clip=False)
    target, = ax[0].plot([], [], '+r')

    outline, = ax[0].plot([], [], color='black')
    fr, = ax[0].plot([], [], color='black')
    rr, = ax[0].plot([], [], color='black')
    fl, = ax[0].plot([], [], color='black')
    rl, = ax[0].plot([], [], color='black')
    rear_axle, = ax[0].plot(car.x, car.y, '+', color='black', markersize=2)

    yaw_arr = []
    yaw_data, = ax[1].plot([], [])
    ax[1].set_xlim(0, sim.frames)
    ax[1].set_ylabel("Yaw")
    ax[1].grid()

    crosstrack_arr = []
    crosstrack_data, = ax[2].plot([], [])
    ax[2].set_xlim(0, sim.frames)
    ax[2].set_ylabel("Crosstrack error")
    ax[2].grid()

    frames = []
    
    def animate(frame):

        # Camera tracks car
        ax[0].set_xlim(car.x - sim.map_size, car.x + sim.map_size)
        ax[0].set_ylim(car.y - 13, car.y + 17)

        # Drive and draw car
        car.drive()
        outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = desc.plot_car(car.x, car.y, car.yaw, car.delta)
        outline.set_data(outline_plot[0], outline_plot[1])
        fr.set_data(fr_plot[0], fr_plot[1])
        rr.set_data(rr_plot[0], rr_plot[1])
        fl.set_data(fl_plot[0], fl_plot[1])
        rl.set_data(rl_plot[0], rl_plot[1])
        rear_axle.set_data(car.x, car.y)

        # Show car's target
        target.set_data(path.px[car.target_id], path.py[car.target_id])

        # Annotate car's coordinate above car
        annotation.set_text('Crosstrack error: {}'.format(np.around(car.crosstrack_error, 5)))
        annotation.set_position((car.x - 10, car.y + 5))

        frames.append(frame)

        # Animate yaw
        yaw_arr.append(car.yaw)
        yaw_data.set_data(frames, yaw_arr)
        ax[1].set_ylim(yaw_arr[-1] - 5, yaw_arr[-1] + 5)

        # Animate crosstrack error
        crosstrack_arr.append(car.crosstrack_error)
        crosstrack_data.set_data(frames, crosstrack_arr)
        ax[2].set_ylim(crosstrack_arr[-1] - 1, crosstrack_arr[-1] + 1)

        ax[0].set_title('{}s'.format(np.around(sim.dt * frame, 2)), loc='right')

        return outline, fr, rr, fl, rl, rear_axle, target, yaw_data, crosstrack_data,

    _ = FuncAnimation(fig, animate, frames=sim.frames, interval=interval, repeat=sim.loop)
    # anim.save('animation.gif', writer='imagemagick', fps=50)
    plt.show()

    print("Mean yaw: {}".format(np.mean(yaw_arr)))
    print("Mean crosstrack error: {}".format(np.mean(crosstrack_arr)))

if __name__ == '__main__':
    main()
