# This file is used to generate random snake friction configurations and support their reproducing features.

from typing import Union
from math import sin, cos
import copy, random
import numpy as np
import scipy.special

pi = 3.14159

"""
TO DO: Support different magnitude values
"""
class FrictionalSnake:
    def __init__(
            self,
            linkNum: int,
            magnitude: int, # Uniform magnitude for all links
            frictionRate: float, # How possible for a link to be frictional. Should be high.
            flagMutateRate: float, # How possible for a link to change its frictional status. Should be low.
            dirMutateRate: float # How possible for a link friction to change its direction. Should be low.
    ):
        self.linkNum = linkNum
        self.magnitude = [magnitude] * linkNum
        self.frictionRate = frictionRate
        self.flagMutateRate = flagMutateRate
        self.dirMutateRate = dirMutateRate

        self.direction = self.generateRandDir()
        self.flag = self.generateRandFlag()
        self.frictionConcigurations = self.snakeFriction()

        self.velocity = 0 # Wait to be changed after each run

    def generateRandDir(self):
        dir = []
        for i in range(self.linkNum):
            dir += [np.random.randint(0, 90)]
        return dir
    
    def generateRandFlag(self):
        flag = []
        for i in range(self.linkNum):
            rand_float = np.random.rand()
            if rand_float <= self.frictionRate:
                flag += [1]
            else:
                flag += [0]
        return flag
    
    def snakeFriction(self) -> dict:
        magnitude_flag = [x * y for x, y in zip(self.magnitude, self.flag)]
        frictionConfigurations = {}
        # i and j represents for the snake and link, seperately.
        for i in range(self.linkNum):
            frictionConfigurations[i] = {
                "lateralFriction": 1,
                "anisotropicFriction": [magnitude_flag[i]*sin(self.direction[i]*pi/180), magnitude_flag[i]*cos(self.direction[i]*pi/180), 0.01],
                "angularDamping": 3,
                'restitution': 3.0
            }
        return frictionConfigurations
    
    def mutateFlag(self):
        for i in range(self.linkNum):
            rand_float = np.random.rand()
            if rand_float <= self.flagMutateRate:
                self.flag[i] = 1 - self.flag[i]

    def mutateDir(self):
        for i in range(self.linkNum):
            rand_float = np.random.rand()
            if rand_float <= self.dirMutateRate:
                self.direction[i] = np.random.randint(0, 90)

    def setVelocity(self, velocity):
        self.velocity = velocity

    def crossover(self, other):
        assert isinstance(other, FrictionalSnake), f"Unexpected type of other snake: {type(other)}"

        startPoint = np.random.randint(0, self.linkNum)
        endPoint = np.random.randint(0, self.linkNum) % (self.linkNum - startPoint) + startPoint

        newSnake1 = copy.deepcopy(self)
        newSnake2 = copy.deepcopy(other)
        tmpDir = newSnake1.direction[startPoint:endPoint + 1]
        tmpFlag = newSnake1.flag[startPoint:endPoint + 1]
        newSnake1.direction[startPoint:endPoint + 1] = newSnake2.direction[startPoint:endPoint + 1]
        newSnake2.direction[startPoint:endPoint + 1] = tmpDir
        newSnake1.flag[startPoint:endPoint + 1] = newSnake2.flag[startPoint:endPoint + 1]
        newSnake2.flag[startPoint:endPoint + 1] = tmpFlag

        newSnake1.frictionConcigurations = newSnake1.snakeFriction()
        newSnake2.frictionConcigurations = newSnake2.snakeFriction()

        return [newSnake1, newSnake2]
     
    def __str__(self):
        return f"The friction of this snake is: {self.frictionConcigurations}\n And the its speed is: {self.velocity}"

class FrictionalSnakeGroup:
    def __init__(
        self,
        frictionalSnakes: list,
    ):
        self.snakeNum = len(frictionalSnakes)
        self.frictionalSnakes = frictionalSnakes
        self.isFiltered = True

    def __add__(self, other):
        assert isinstance(other, FrictionalSnakeGroup), f"other has to be of type FrictionalSnakeGroup"
        newGroup = FrictionalSnakeGroup(self.frictionalSnakes + other.frictionalSnakes)
        newGroup.isFiltered = False
        return newGroup

    def linkCrossover(self):
        # exchange segments between two snakes, return crossovered snakes.
        # REMEMBER to merge them into one group before execute the simulation.
        assert self.isFiltered == True, "The snakes have to be filtered before crossover."
        crossoveredSnakes = []

        snakeNumList = list(range(self.snakeNum))
        random.shuffle(snakeNumList)
        snakePairs = []
        for i in range(0, len(snakeNumList), 2):
            if i+1 < len(snakeNumList):
                snakePairs.append([snakeNumList[i], snakeNumList[i+1]])

        for pair in snakePairs:
            snake1, snake2 = copy.deepcopy(self.frictionalSnakes[pair[0]]), copy.deepcopy(self.frictionalSnakes[pair[1]])
            crossoveredSnakes += snake1.crossover(snake2)

        if len(crossoveredSnakes) < self.snakeNum:
            lastPair = np.random.choice(snakeNumList, 2, replace = False)
            snake1, snake2 = self.frictionalSnakes[lastPair[0]], self.frictionalSnakes[lastPair[1]]
            crossoveredSnakes += snake1.crossover(snake2)
            crossoveredSnakes = crossoveredSnakes[:-1]

        return FrictionalSnakeGroup(crossoveredSnakes)

    def mutate(self):
        for snake in self.frictionalSnakes:
            snake.mutateFlag()
            snake.mutateDir()

    def filterSlowSnakes(self, ratio = 0.5):
        # filter out snakes with velocity slower than threshold
        if not self.isFiltered:
            self.bigDisaster()
            velocities = [snake.velocity for snake in self.frictionalSnakes]
            probabilities = scipy.special.softmax(velocities)

            filteredVelocities = np.random.choice(velocities, int(self.snakeNum * ratio), replace = False, p=probabilities)
            # print(filteredVelocities)
            self.frictionalSnakes = [snake for snake in self.frictionalSnakes if snake.velocity in filteredVelocities]
            # print([snake.velocity for snake in self.frictionalSnakes])

            self.snakeNum = len(self.frictionalSnakes)
            self.isFiltered = True
        return self

    def setVelocities(self, velocities):
        for v in velocities:
            self.frictionalSnakes[v[0]].setVelocity(v[1])

    def bigDisaster(self):
        # filter the snakes with the same velocity
        individuals = {}
        for snake in self.frictionalSnakes:
            individuals[snake.velocity] = snake
        tmp = []
        for key in individuals:
            tmp.append(individuals[key])
        self.frictionalSnakes = tmp
