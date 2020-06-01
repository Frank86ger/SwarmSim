"""
@author: Frank Ehebrecht / May2020
"""

import numpy as np
import matplotlib.pyplot as plt
import sklearn.metrics as me

from matplotlib.animation import FuncAnimation


class Boid(object):
    def __init__(self, velocity, size=(10, 10)):
        self.size = size
        self.velocity = velocity
        self.x = np.random.random() * size[0]
        self.y = np.random.random() * size[1]
        self.vx = np.random.random()*velocity * (2 * np.random.randint(0, 2)-1)
        self.vy = np.sqrt(velocity**2 - self.vx**2) * (2 * np.random.randint(0, 2)-1)

    def move(self):
        """ Move boid by `vx` and `vy`. """
        self.x = self.x + self.vx
        self.y = self.y + self.vy

        # border fail-safe
        if self.x > self.size[0]:
            self.x = self.size[0]-0.0001
        if self.y > self.size[1]:
            self.y = self.size[1]-0.0001
        if self.x < 0:
            self.x = 0+.0001
        if self.y < 0:
            self.y = 0+0.0001


class Swarm(object):
    def __init__(self, amount_boids, size=(10, 10)):
        self.amount_boids = amount_boids
        self.boids = []
        self.positions = None
        self.velocities = None
        self.pos_diff_x = None
        self.pos_diff_y = None
        self.base_velocity = 0.05
        self.max_velocity = 0.20
        self.perception_angle = 50
        self.drive = 0.001  # as in: opposite of inertia
        self.size = size
        for _ in range(self.amount_boids):
            self.boids.append(Boid(self.base_velocity, size=self.size))

    def move_boids(self):
        self.adjust_velocities()
        for boid in self.boids:
            boid.move()
        return self.positions, self.velocities

    def adjust_velocities(self):
        self.gather_velocities()
        self.gather_positions()
        distances = self.get_distances()
        border_distances = self.calc_border_distances()
        angle_decisions = self.angular_perception_filter(self.perception_angle)
        print(np.sum(angle_decisions))

        # calculate magnitudes from potentials
        separation_magnitudes = separation_potential_func(distances)  # steer AWAY from weighted center of mass
        alignment_magnitudes = alignment_potential_func(distances)  # adjust to weighted average of directions
        cohesion_magnitudes = cohesion_potential_func(distances)  # steer TOWARDS weighted center of mass
        border_magnitudes = border_potential_func(border_distances)

        # magnitudes for diagonal elements must be zero
        np.fill_diagonal(separation_magnitudes, 0)
        np.fill_diagonal(alignment_magnitudes, 0)
        np.fill_diagonal(cohesion_magnitudes, 0)

        # TODO: this does not work yet
        # set magnitudes to zero for boids, that are not within perceptive field
        separation_magnitudes[np.where(angle_decisions is False)] = 0
        alignment_magnitudes[np.where(angle_decisions is False)] = 0
        cohesion_magnitudes[np.where(angle_decisions is False)] = 0

        # calculate vectors for all potentials
        separation_vectors = self.separation(separation_magnitudes)
        alignment_vectors = self.alignment(alignment_magnitudes)
        cohesion_vectors = self.cohesion(cohesion_magnitudes)
        border_repulsion_vectors = self.border_repulsion(border_magnitudes)
        vector_update = separation_vectors + 20*alignment_vectors + cohesion_vectors + 300*border_repulsion_vectors
        vector_update = vector_update.T

        # update velocities of boids
        for idx, boid in enumerate(self.boids):
            norm = np.sqrt(boid.vx**2 + boid.vy**2)
            boid.vx /= norm / self.base_velocity
            boid.vy /= norm / self.base_velocity

            new_x = self.drive * vector_update[idx, 0] + boid.vx
            new_y = self.drive * vector_update[idx, 1] + boid.vy
            norm = np.sqrt(new_x**2 + new_y**2)

            # if velocity bigger than `max_velocity`, norm it to `max_velocity
            if norm >= self.max_velocity:
                boid.vx = self.max_velocity * new_x / norm
                boid.vy = self.max_velocity * new_y / norm
            else:
                boid.vx = self.drive * vector_update[idx, 0] + boid.vx
                boid.vy = self.drive * vector_update[idx, 1] + boid.vy

    def gather_positions(self):
        x_data = []
        y_data = []
        for boid in self.boids:
            x_data.append(boid.x)
            y_data.append(boid.y)
        self.positions = np.array([x_data, y_data]).T

        positions_x = self.positions[:, 0].reshape(-1, 1)
        positions_y = self.positions[:, 1].reshape(-1, 1)
        self.pos_diff_x = (positions_x - positions_x.T)
        self.pos_diff_y = (positions_y - positions_y.T)

    def gather_velocities(self):
        x_data = []
        y_data = []
        for boid in self.boids:
            x_data.append(boid.vx)
            y_data.append(boid.vy)
        self.velocities = np.array([x_data, y_data]).T

    def get_distances(self):
        return me.pairwise_distances(self.positions)

    def angular_perception_filter(self, alpha):
        # TODO: this still has bugs
        vel = np.sqrt(self.velocities[:, 0]**2 + self.velocities[:, 1]**2)
        pos = np.sqrt(self.pos_diff_x**2 + self.pos_diff_y**2)
        a = self.pos_diff_x * self.velocities[:, 0]
        b = self.pos_diff_y * self.velocities[:, 1]

        np.fill_diagonal(pos, 1)

        acos_stuff = a + b / (vel*pos)

        angles = np.arccos(acos_stuff)
        angles = angles/2/np.pi*360
        angle_decision = np.zeros_like(angles, dtype=bool)
        angle_decision[np.where(np.abs(angles) < alpha)] = True
        return angle_decision

    def calc_border_distances(self):
        # left right top down
        left = self.positions[:, 0]
        left[np.where(left < 0.)] = 0
        right = self.size[0] - self.positions[:, 0]
        right[np.where(right < 0)] = 0
        bot = self.positions[:, 1]
        bot[np.where(bot < 0.)] = 0
        top = self.size[1] - self.positions[:, 1]
        top[np.where(top < 0.)] = 0
        return np.vstack([left, right, bot, top]).T

    def separation(self, magnitudes):
        norms = np.sqrt(self.pos_diff_x ** 2 + self.pos_diff_y ** 2)
        norms[np.where(norms == 0)] = 1
        x = np.sum((magnitudes * -self.pos_diff_x/norms), axis=0)
        y = np.sum((magnitudes * -self.pos_diff_y/norms), axis=0)
        return np.array([x, y])

    def alignment(self, magnitudes):
        # TODO: no norm should be fine, but try with norm later on
        x = np.dot(magnitudes, self.velocities[:, 0])
        y = np.dot(magnitudes, self.velocities[:, 1])
        return np.array([x, y])

    def cohesion(self, magnitudes):
        norms = np.sqrt(self.pos_diff_x ** 2 + self.pos_diff_y ** 2)
        norms[np.where(norms == 0)] = 1
        x = np.sum((magnitudes * -self.pos_diff_x/norms), axis=0)
        y = np.sum((magnitudes * -self.pos_diff_y/norms), axis=0)
        return np.array([x, y])

    def border_repulsion(self, magnitudes):
        # left
        x = magnitudes[:, 0] * self.positions[:, 0]
        # right
        x += magnitudes[:, 1] * (self.positions[:, 0] - self.size[0])
        # bottom
        y = magnitudes[:, 2] * self.positions[:, 1]
        # top
        y += magnitudes[:, 3] * (self.positions[:, 1] - self.size[1])
        return np.array([x, y])


def separation_potential_func(dist):
    return 0.1/(dist+0.0001)**2


def alignment_potential_func(dist):
    return 1.0*np.exp(-dist**2)


def cohesion_potential_func(dist):
    return 1./(dist+1.)


def border_potential_func(dist):
    return separation_potential_func(dist)


if __name__ == "__main__":
    fig, ax = plt.subplots()
    size_ = (10, 10)
    amount_boids_ = 40
    swarm = Swarm(amount_boids_, size=size_)
    sc, = plt.plot([], [], 'ro')

    def init():
        ax.set_xlim(0 - 0.1 * size_[0], size_[0] + 0.1 * size_[0])
        ax.set_ylim(0 - 0.1 * size_[1], size_[1] + 0.1 * size_[1])
        ax.axhline(y=0, color='red')
        ax.axhline(y=size_[0], color='red')
        ax.axvline(x=0, color='red')
        ax.axvline(x=size_[1], color='red')
        return sc,

    def update(_):
        positions, velocities = swarm.move_boids()
        sc.set_data(positions.T)
        return sc,

    ani = FuncAnimation(fig, update, frames=np.arange(10000),
                        init_func=init, blit=True, interval=20)
    plt.show()
