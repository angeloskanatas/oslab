#!/usr/bin/python3

import os

os.dup2(2, 99)
os.execv("./riddle", ["./riddle"])
