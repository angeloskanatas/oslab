#!/usr/bin/python3

import os

r1, w1=os.pipe()
r2, w2=os.pipe()

os.dup2(r1, 33)
os.dup2(w1, 34)
os.dup2(r2, 53)
os.dup2(w2, 54)

os.execv("./riddle", ["./riddle"])
