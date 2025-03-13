import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import examples.snake.GASnake.RandSnakeFriction as rsf

dir = os.path.join(os.path.dirname(__file__), "resultsGA")

def record_maker():
    count = 1
    def createCsvEveryRun():
        nonlocal count
        with open(os.path.join(dir,"everysnake",f"configuration_run_{count}.csv"), "w") as f:
            f.write("Friction, Flag, Direction, Velocity\n")

    def printEverything(snake_group: rsf.FrictionalSnakeGroup):
        nonlocal count
        with open(os.path.join(dir,"everysnake", f"configuration_run_{count}.csv"), "a") as f:
            for snake in snake_group.frictionalSnakes:
                f.write(f"{snake.frictionConcigurations}, {snake.flag}, {snake.direction}, {snake.velocity}\n")

    def plotConvergence(data: pd.DataFrame):
        nonlocal count
        plt.clf()
        x_data = data.iloc[:, 0].values
        y_data = data.iloc[:, 1].values
        plt.plot(x_data, y_data, label=f"Run {count}")
        plt.savefig(os.path.join(dir, f"convergence_run_{count}.png"))
        plt.close

    def printFastest(snake: rsf.FrictionalSnake, time = 0):
        nonlocal count
        with open(os.path.join(dir,"txt", f"configuration_run_{count}.csv"), "w") as f:
            f.write(f"The friction of this snake is: {snake.frictionConcigurations}\nAnd the degree is {snake.direction}\nAnd the flag is {snake.flag}\nAnd its speed is: {snake.velocity}\nTime: {time}")
        count += 1

    return plotConvergence, printFastest, createCsvEveryRun, printEverything
        