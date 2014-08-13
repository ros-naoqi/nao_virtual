#!/usr/bin/env python

# Copyright (C) 2014 Aldebaran Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# authors: Mikael Arguedas [mikael DOT arguedas AT gmail DOT com]
#FIXME Add : Fingers for Humanoids and Sensors

import sys
import argparse
import subprocess
import os
import math
from os import path

parser = argparse.ArgumentParser(usage='Load an URDF file')
parser.add_argument('-i','--input', default=None, help='file containing the trajectory (rostopic echo controller/follow_joint_trajectory/goal)')


args = parser.parse_args()
if os.path.isfile(args.input):
    output = args.input[0:args.input.rfind('.')] + '_modified' + args.input[args.input.rfind('.'):]
    print output
    file = open(args.input,'r')
    lines = file.readlines()
    outfile = open(output,'w+')
    for i in range(12,len(lines)-6):
        outfile.write(lines[i][2:])
    file.close()
    outfile.close()
else:
    print "input file doesn't exist"

