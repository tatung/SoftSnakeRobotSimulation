import os

pathAffix = os.path.join("~" ,"somo", "examples", "snake", "resultsGA", "txt")
txtAffix = "configuration_run_"
fileNumber = len(os.listdir(os.path.expanduser(pathAffix)))

# print the second row of the file

max = 0
maxNumber = 0
print(fileNumber)
for i in range(4, 219):
    file = os.path.expanduser(os.path.join(pathAffix, txtAffix + str(i) + ".txt"))
    with open(file, "r") as f:
        lines = f.readlines()
        curSpeed = float(lines[1][23:])
    if curSpeed > max:
        maxNumber = i
        max = curSpeed
print(maxNumber, max)