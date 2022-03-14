import math
import functions
import matplotlib.pyplot as plt
import warnings

warnings.filterwarnings("ignore")


class ProjectileSim:

    def __init__(self):
        # setting states for instance variables
        self.positionx = 0
        self.positiony = 0
        self.positionz = 0
        self.posx = []
        self.posy = []
        self.posz = []

        self.speed = 0
        self.bearing = 0
        self.trajectory = 0
        self.velocity = []

        self.windspeed = 0
        self.windbearing = 0

        self.t = 0.
        self.interval = 0
        self.timesteps = []

        self.targetx = 0
        self.targety = 0

    def initialize(self, startingspeed, startingbearing, startingtrajectory, windspeed, windbearing, targetx, targety,
                   timeinterval):
        # dimension positions
        self.positionx = 0
        self.positiony = 0
        self.positionz = 2.2

        # velocity vector measures
        self.speed = startingspeed
        self.bearing = startingbearing
        self.trajectory = startingtrajectory
        self.velocity = [self.speed]

        # measures for wind (headwing_vector variables)
        self.windspeed = windspeed
        self.windbearing = windbearing

        # 2 dimension target coordinates of ball (3rd dimension z not necessary since the ball will be at ground level)
        self.targetx = targetx
        self.targety = targety

        # Initialize time, and all data tracking lists to initial state
        self.interval = timeinterval
        self.t = 0.
        self.timesteps = [self.t]

        self.posx = [self.positionx]
        self.posy = [self.positiony]
        self.posz = [self.positionz]

    def observe(self):
        # appending all data to its appropriate list for plotting
        self.posx.append(self.positionx)
        self.posy.append(self.positiony)
        self.posz.append(self.positionz)
        self.velocity.append(self.speed)
        self.timesteps.append(self.t)

    def update(self):
        oldpos1 = self.positionx
        oldpos2 = self.positiony
        oldpos3 = self.positionz

        headwind_vector = functions.add_spherical_vectors((-self.speed), self.bearing, self.trajectory, self.windspeed,
                                                          self.windbearing, 0)

        drag_force = 0.003 * (headwind_vector[0] ** 2)

        drag_acceleration_magnitude = drag_force / 0.2

        drag_gravity_vector = functions.add_spherical_vectors((drag_acceleration_magnitude * self.interval),
                                                              headwind_vector[1], headwind_vector[2],
                                                              (9.8 * self.interval), 0, -90)

        final_ball_velocity = (self.speed, self.bearing, self.trajectory)
        final_ball_velocity = functions.add_spherical_vectors(final_ball_velocity[0], final_ball_velocity[1],
                                                              final_ball_velocity[2], drag_gravity_vector[0],
                                                              drag_gravity_vector[1], drag_gravity_vector[2]
                                                              )

        self.speed = final_ball_velocity[0]
        self.bearing = final_ball_velocity[1]
        self.trajectory = final_ball_velocity[2]

        positions = functions.spherical_to_components(final_ball_velocity[0], final_ball_velocity[1],
                                                      final_ball_velocity[2])

        newpos1 = oldpos1 + positions[0] * self.interval
        newpos2 = oldpos2 + positions[1] * self.interval
        newpos3 = oldpos3 + positions[2] * self.interval

        self.positionx = newpos1
        self.positiony = newpos2
        self.positionz = newpos3

        self.t = self.t + self.interval

    def runSim(self, startingspeed, startingbearing, startingtrajectory, windspeed, windbearing, targetx, targety,
               timeinterval):
        self.initialize(startingspeed, startingbearing, startingtrajectory, windspeed, windbearing, targetx, targety,
                        timeinterval)
        while self.positionz > 0:
            self.update()
            self.observe()

        distance = math.sqrt((self.positionx - self.targetx) ** 2 + (self.positiony - self.targety) ** 2)

        print(f'Distance from target: {distance:.2f}')

        plt.figure(1)
        plt.title("X Position vs. Time")
        plt.xlabel("t (s)")
        plt.ylabel("x Distance (m)")
        plt.plot(self.timesteps, self.posx)

        plt.figure(2)
        plt.title("X Position vs. Z Position (Height)")
        plt.xlabel("X Distance (m)")
        plt.ylabel("Z Height(m)")
        plt.plot(self.posx, self.posz)

        plt.figure(3)
        plt.title("X Position vs. Y Position")
        plt.xlabel("X Distance (m)")
        plt.ylabel("Y Distance(m)")
        plt.plot(self.posx, self.posy)
        plt.plot(targetx, targety, 'ro')

        plt.figure(4)
        plt.title("3D Path of Ball")
        ax = plt.axes(projection='3d')
        plt.title("3D Path of Ball")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        plt.plot(self.posx, self.posy, self.posz)
        plt.plot(targetx, targety, 0, 'ro')

        plt.figure(5)
        plt.title("Velocity vs. time")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.plot(self.timesteps, self.velocity)
        plt.show()


if __name__ == '__main__':
    sim = ProjectileSim()
    sim.runSim(30, 15, 40, 20, 170, 10, 20, .01)
    # sim.runSim(40, -35, 50, 25, 90, 30, 30, .01)
