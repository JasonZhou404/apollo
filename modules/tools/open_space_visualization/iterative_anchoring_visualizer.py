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

from iterative_anchoring_python_interface import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import numpy as np
import time
import math

def SmoothTrajectory(visualize_flag, sx, sy):
    # initialze object
    OpenSpacePlanner = IterativeAnchoringPlanner()

    # parameter(except max, min and car size is defined in proto)
    num_output_buffer = 10000
    # sx = -8
    # sy = 1.5
    # sphi = 0.5
    sphi = 0.0

    scenario = "backward"
    # scenario = "parallel"

    if scenario == "backward":
        # obstacles for distance approach(vertices coords in clock wise order)
        left_boundary_x = (
            c_double * 3)(*[-13.6407054776, 0.0, 0.0515703622475])
        left_boundary_y = (
            c_double * 3)(*[0.0140634663703, 0.0, -5.15258191624])
        down_boundary_x = (c_double * 2)(*[0.0515703622475, 2.8237895441])
        down_boundary_y = (c_double * 2)(*[-5.15258191624, -5.15306980547])
        right_boundary_x = (
            c_double * 3)(*[2.8237895441, 2.7184833539, 16.3592013995])
        right_boundary_y = (
            c_double * 3)(*[-5.15306980547, -0.0398078878812, -0.011889513383])
        up_boundary_x = (c_double * 2)(*[16.3591910364, -13.6406951857])
        up_boundary_y = (c_double * 2)(*[5.60414234644, 5.61797800844])
        # obstacles(x, y, size)
        OpenSpacePlanner.AddVirtualObstacle(left_boundary_x, left_boundary_y, 3)
        OpenSpacePlanner.AddVirtualObstacle(
            down_boundary_x, down_boundary_y, 2)
        OpenSpacePlanner.AddVirtualObstacle(
            right_boundary_x, right_boundary_y, 3)
        OpenSpacePlanner.AddVirtualObstacle(
            up_boundary_x, up_boundary_y, 2)
        ex = 1.359
        ey = -3.86443643718
        ephi = 1.581
        XYbounds = [-13.6406951857, 16.3591910364, -
            5.15258191624, 5.61797800844]

    x = (c_double * num_output_buffer)()
    y = (c_double * num_output_buffer)()
    phi = (c_double * num_output_buffer)()
    v = (c_double * num_output_buffer)()
    a = (c_double * num_output_buffer)()
    kappa = (c_double * num_output_buffer)()
    opt_x = (c_double * num_output_buffer)()
    opt_y = (c_double * num_output_buffer)()
    opt_phi = (c_double * num_output_buffer)()
    opt_v = (c_double * num_output_buffer)()
    opt_a = (c_double * num_output_buffer)()
    opt_kappa = (c_double * num_output_buffer)()
    hybrid_a_output_size = (c_ushort * 1)()
    iterative_anchoring_output_size = (c_ushort * 1)()
    XYbounds_ctype = (c_double * 4)(*XYbounds)
    hybrid_a_time = (c_double * 1)(0.0)
    iterative_anchoring_time = (c_double * 1)(0.0)

    success = True
    start = time.time()
    print("planning start")
    if not OpenSpacePlanner.IterativeAnchoringPlan(sx, sy, sphi, ex, ey, ephi, XYbounds_ctype):
        print("planning fail")
        success = False
    planning_time = time.time() - start
    print("planning time is " + str(planning_time))

    x_out = []
    y_out = []
    phi_out = []
    v_out = []
    a_out = []
    kappa_out = []
    opt_x_out = []
    opt_y_out = []
    opt_phi_out = []
    opt_v_out = []
    opt_a_out = []
    opt_kappa_out = []

    if visualize_flag and success:
        # load result
        OpenSpacePlanner.IAGetResult(x, y, phi, v, a, kappa, opt_x,
                                        opt_y, opt_phi, opt_v, opt_a, opt_kappa, 
                                        hybrid_a_output_size, iterative_anchoring_output_size,
                                        hybrid_a_time, iterative_anchoring_time)
        for i in range(0, hybrid_a_output_size[0]):
            x_out.append(float(x[i]))
            y_out.append(float(y[i]))
            phi_out.append(float(phi[i]))
            v_out.append(float(v[i]))
            a_out.append(float(a[i]))
            kappa_out.append(float(kappa[i]))
        for i in range(0, iterative_anchoring_output_size[0]):
            opt_x_out.append(float(opt_x[i]))
            opt_y_out.append(float(opt_y[i]))
            opt_phi_out.append(float(opt_phi[i]))
            opt_v_out.append(float(opt_v[i]))
            opt_a_out.append(float(opt_a[i]))
            opt_kappa_out.append(float(opt_kappa[i]))

        # trajectories plot
        fig1 = plt.figure(1)
        ax = fig1.add_subplot(111)
        for i in range(0, hybrid_a_output_size[0]):
            # warm start
            downx = 1.055 * math.cos(phi_out[i] - math.pi / 2)
            downy = 1.055 * math.sin(phi_out[i] - math.pi / 2)
            leftx = 1.043 * math.cos(phi_out[i] - math.pi)
            lefty = 1.043 * math.sin(phi_out[i] - math.pi)
            x_shift_leftbottom = x_out[i] + downx + leftx
            y_shift_leftbottom = y_out[i] + downy + lefty
            warm_start_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                            angle=phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='r', facecolor='none')
            warm_start_arrow = patches.Arrow(
                x_out[i], y_out[i], 0.25*math.cos(phi_out[i]), 0.25*math.sin(phi_out[i]), 0.2, edgecolor='r',)
            # ax.add_patch(warm_start_car)
            ax.add_patch(warm_start_arrow)
        for i in range(0, iterative_anchoring_output_size[0]):
            # iterative_anchoring
            downx = 1.055 * math.cos(opt_phi_out[i] - math.pi / 2)
            downy = 1.055 * math.sin(opt_phi_out[i] - math.pi / 2)
            leftx = 1.043 * math.cos(opt_phi_out[i] - math.pi)
            lefty = 1.043 * math.sin(opt_phi_out[i] - math.pi)
            x_shift_leftbottom = opt_x_out[i] + downx + leftx
            y_shift_leftbottom = opt_y_out[i] + downy + lefty
            smoothing_car = patches.Rectangle((x_shift_leftbottom, y_shift_leftbottom), 3.89 + 1.043, 1.055*2,
                                            angle=opt_phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='y', facecolor='none')
            smoothing_arrow = patches.Arrow(
                opt_x_out[i], opt_y_out[i], 0.25*math.cos(opt_phi_out[i]), 0.25*math.sin(opt_phi_out[i]), 0.2, edgecolor='y',)
            ax.add_patch(smoothing_car)
            ax.add_patch(smoothing_arrow)

        ax.plot(sx, sy, "s")
        ax.plot(ex, ey, "s")
        if scenario == "backward":
            left_boundary_x = [-13.6407054776, 0.0, 0.0515703622475]
            left_boundary_y = [0.0140634663703, 0.0, -5.15258191624]
            down_boundary_x = [0.0515703622475, 2.8237895441]
            down_boundary_y = [-5.15258191624, -5.15306980547]
            right_boundary_x = [2.8237895441, 2.7184833539, 16.3592013995]
            right_boundary_y = [-5.15306980547, -0.0398078878812, -0.011889513383]
            up_boundary_x = [16.3591910364, -13.6406951857]
            up_boundary_y = [5.60414234644, 5.61797800844]
            ax.plot(left_boundary_x, left_boundary_y, "k")
            ax.plot(down_boundary_x, down_boundary_y, "k")
            ax.plot(right_boundary_x, right_boundary_y, "k")
            ax.plot(up_boundary_x, up_boundary_y, "k")

        plt.axis('equal')

        # input plot
        fig2 = plt.figure(2)
        v_graph = fig2.add_subplot(311)
        v_graph.title.set_text('v')
        v_graph.plot(np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), v_out)
        v_graph.plot(np.linspace(0, iterative_anchoring_output_size[0], iterative_anchoring_output_size[0]), opt_v_out)
        a_graph = fig2.add_subplot(312)
        a_graph.title.set_text('a')
        a_graph.plot(np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), a_out)
        a_graph.plot(np.linspace(0, iterative_anchoring_output_size[0], iterative_anchoring_output_size[0]), opt_a_out)
        kappa_graph = fig2.add_subplot(313)
        kappa_graph.title.set_text('kappa')
        kappa_graph.plot(np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), kappa_out)
        kappa_graph.plot(np.linspace(0, iterative_anchoring_output_size[0], iterative_anchoring_output_size[0]), opt_kappa_out)
        plt.show()
        return True

    if not visualize_flag :
        if success :
            # load result
            OpenSpacePlanner.IAGetResult(x, y, phi, v, a, kappa, opt_x,
                                        opt_y, opt_phi, opt_v, opt_a, opt_kappa, 
                                        hybrid_a_output_size, iterative_anchoring_output_size,
                                        hybrid_a_time, iterative_anchoring_time)
            for i in range(0, hybrid_a_output_size[0]):
                x_out.append(float(x[i]))
                y_out.append(float(y[i]))
                phi_out.append(float(phi[i]))
                v_out.append(float(v[i]))
                a_out.append(float(a[i]))
                kappa_out.append(float(kappa[i]))
            for i in range(0, iterative_anchoring_output_size[0]):
                opt_x_out.append(float(opt_x[i]))
                opt_y_out.append(float(opt_y[i]))
                opt_phi_out.append(float(opt_phi[i]))
                opt_v_out.append(float(opt_v[i]))
                opt_a_out.append(float(opt_a[i]))
                opt_kappa_out.append(float(opt_kappa[i]))
        return [success, opt_x_out, opt_y_out, opt_phi_out, opt_v_out, opt_a_out, opt_kappa_out, \
            hybrid_a_time, iterative_anchoring_time, planning_time]
    return False

if __name__ == '__main__':
    # visualize_flag = True
    # SmoothTrajectory(visualize_flag)

    visualize_flag = True
    planning_time_stats = []
    hybrid_a_time_stats = []
    iterative_anchoring_time_stats = []


    test_count = 0
    success_count = 0
    # for sx in np.arange(-10, 10, 1.0): 
    for sx in [-8]: 
        # for sy in np.arange(2, 4, 0.5):
        for sy in [3]:
            # print("sx is "+ str(sx) + " and sy is " + str(sy))
            test_count += 1
            result = SmoothTrajectory(visualize_flag, sx, sy)
            if result[0] :
                success_count += 1
                planning_time_stats.append(result[-1])
                iterative_anchoring_time_stats.append(result[-2])
                hybrid_a_time_stats.append(result[-3])

    print("success rate is "+ str(float(success_count) / float(test_count)))            

    module_timing = np.asarray([planning_time_stats, hybrid_a_time_stats, iterative_anchoring_time_stats])
    np.savetxt(result_file, module_timing, delimiter=",")

    print("average hybird time: %4.4f, with max: %4.4f, min: %4.4f" % (
        sum(hybrid_a_time_stats) / len(hybrid_a_time_stats), max(hybrid_a_time_stats),
        min(hybrid_a_time_stats)))
    print("average iterative_anchoring_time: %4.4f, with max: %4.4f, min: %4.4f" % (
        sum(iterative_anchoring_time_stats) / len(iterative_anchoring_time_stats), max(iterative_anchoring_time_stats),
        min(iterative_anchoring_time_stats)))
    print("average total time: %4.4f, with max: %4.4f, min: %4.4f" % (
        sum(planning_time_stats) / len(planning_time_stats), max(planning_time_stats),
        min(planning_time_stats)))