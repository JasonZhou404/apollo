#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
###############################################################################

import ctypes
from ctypes import *
import math

lib = cdll.LoadLibrary(
    '/apollo/bazel-bin/modules/planning/open_space/tools/iterative_anchoring_wrapper_lib.so')

lib.IACreateObstaclesPtr.argtypes = []
lib.IACreateObstaclesPtr.restype = c_void_p
lib.CreateHybridAPtr.argtypes = []
lib.CreateHybridAPtr.restype = c_void_p
lib.CreateIterativeAnchoringPtr.argtypes = []
lib.CreateIterativeAnchoringPtr.restype = c_void_p
lib.IACreateResultPtr.argtypes = []
lib.IACreateResultPtr.restype = c_void_p
lib.IAAddVirtualObstacle.argtypes = [c_void_p, POINTER(c_double), POINTER(c_double), c_int]
lib.IterativeAnchoringPlan.restype = c_bool
lib.IterativeAnchoringPlan.argtypes = [c_void_p, c_void_p, c_void_p, c_void_p, c_double, c_double, c_double, c_double, 
                     c_double, c_double, POINTER(c_double)]
lib.IAGetResult.argtypes = [c_void_p, POINTER(c_double), POINTER(c_double), POINTER(c_double),
                    POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), 
                    POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), 
                    POINTER(c_double), POINTER(c_ushort), POINTER(c_ushort), POINTER(c_double), 
                    POINTER(c_double), POINTER(c_double), POINTER(c_double)]

class IterativeAnchoringPlanner(object):
    def __init__(self):
        self.warm_start_planner = lib.CreateHybridAPtr()
        self.smoothing_planner = lib.CreateIterativeAnchoringPtr()
        self.obstacles = lib.IACreateObstaclesPtr()
        self.result = lib.IACreateResultPtr()

    def AddVirtualObstacle(self, obstacle_x, obstacle_y, vertice_num):
        lib.IAAddVirtualObstacle(self.obstacles, POINTER(c_double)(obstacle_x),
                               POINTER(c_double)(obstacle_y), (c_int)(vertice_num))

    def IterativeAnchoringPlan(self, sx, sy, sphi, ex, ey, ephi, XYbounds):
        return lib.IterativeAnchoringPlan(self.warm_start_planner, self.smoothing_planner, self.obstacles, self.result, c_double(sx),
                        c_double(sy), c_double(sphi), c_double(ex), c_double(ey), c_double(ephi), POINTER(c_double)(XYbounds))
    
    def IAGetResult(self, x, y, phi, v, a, kappa, opt_x, opt_y, opt_phi, opt_v, opt_a, opt_kappa, hybrid_a_output_size, \
        iterative_anchoring_output_size, hybrid_a_time, iterative_total_time, path_time, speed_time):
        lib.IAGetResult(self.result, POINTER(c_double)(x), POINTER(c_double)(y), \
                      POINTER(c_double)(phi), POINTER(c_double)(v), POINTER(c_double)(a), POINTER(c_double)(kappa), \
                      POINTER(c_double)(opt_x), POINTER(c_double)(opt_y), \
                      POINTER(c_double)(opt_phi), POINTER(c_double)(opt_v), POINTER(c_double)(opt_a), POINTER(c_double)(opt_kappa), \
                      POINTER(c_ushort)(hybrid_a_output_size), POINTER(c_ushort)(iterative_anchoring_output_size), \
                      POINTER(c_double)(hybrid_a_time), POINTER(c_double)(iterative_total_time), POINTER(c_double)(path_time), \
                      POINTER(c_double)(speed_time))
