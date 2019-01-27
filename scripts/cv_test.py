import subprocess

with open("stdout_cv.txt", "a+") as stdout:
    for i in range(1000):
        p = subprocess.run(["sys161", "kernel", "sy3;q"], cwd="../root", stdout=stdout, stderr=stdout)
