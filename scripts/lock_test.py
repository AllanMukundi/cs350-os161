import subprocess

with open("stdout_lock.txt", "a+") as stdout:
    for i in range(1000):
        p = subprocess.run(["sys161", "kernel", "sy2;q"], cwd="../root", stdout=stdout, stderr=stdout)
