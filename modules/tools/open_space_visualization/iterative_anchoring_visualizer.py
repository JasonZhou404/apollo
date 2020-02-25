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

from pylab import *

def SmoothTrajectory(visualize_flag, sx, sy):
    # initialze object
    OpenSpacePlanner = IterativeAnchoringPlanner()

    # parameter(except max, min and car size is defined in proto)
    num_output_buffer = 10000
    sphi = 0.0

    scenario = "backward"
    scenario = "parallel"

    if scenario == "backward":
        left_boundary_x = (
            c_double * 3)(*[-10.0, 0.0, 0.0])
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
    if scenario == "parallel":
        left_boundary_x = (
            c_double * 3)(*[-10.0, 0.0, 0.0])
        left_boundary_y = (
            c_double * 3)(*[0.0, 0.0, -2.5])
        down_boundary_x = (c_double * 2)(*[0.0, 7.0])
        down_boundary_y = (c_double * 2)(*[-2.5, -2.5])
        right_boundary_x = (
            c_double * 3)(*[7.0, 7.0, 17])
        right_boundary_y = (
            c_double * 3)(*[-2.5, 0.0, 0.0])
        up_boundary_x = (c_double * 2)(*[17.0 -10.0])
        up_boundary_y = (c_double * 2)(*[5.0, 5.0])

        OpenSpacePlanner.AddVirtualObstacle(left_boundary_x, left_boundary_y, 3)
        OpenSpacePlanner.AddVirtualObstacle(
            down_boundary_x, down_boundary_y, 2)
        OpenSpacePlanner.AddVirtualObstacle(
            right_boundary_x, right_boundary_y, 3)
        OpenSpacePlanner.AddVirtualObstacle(
            up_boundary_x, up_boundary_y, 2)
        ex = 2.0
        ey = -1.25
        ephi = 0.0
        XYbounds = [-10.0, 17.0, -2.5, 5.0]

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
    iterative_total_time = (c_double * 1)(0.0)
    iterative_path_time = (c_double * 1)(0.0)
    iterative_speed_time = (c_double * 1)(0.0)

    success = True
    start = time.time()
    print("planning start")
    if not OpenSpacePlanner.IterativeAnchoringPlan(sx, sy, sphi, ex, ey, ephi, XYbounds_ctype):
        print("planning fail")
        success = False
    planning_time = time.time() - start
    # print("planning time is " + str(planning_time))

    x_out = []
    y_out = []
    phi_out = []
    v_out = []
    a_out = []
    jerk_out = []
    kappa_out = []
    opt_x_out = []
    opt_y_out = []
    opt_phi_out = []
    opt_v_out = []
    opt_a_out = []
    opt_jerk_out = []
    opt_kappa_out = []

    if visualize_flag and success:
        # load result
        OpenSpacePlanner.IAGetResult(x, y, phi, v, a, kappa, opt_x,
                                        opt_y, opt_phi, opt_v, opt_a, opt_kappa, 
                                        hybrid_a_output_size, iterative_anchoring_output_size,
                                        hybrid_a_time, iterative_total_time, iterative_path_time, iterative_speed_time)
        s = []
        acc_s = 0
        last_x = float(opt_x[0])
        last_y = float(opt_y[0])

        stop_time = []
        gear_s = []

        opt_jerk_out.append(0.0)
        jerk_out.append(0.0)

        for i in range(0, hybrid_a_output_size[0]):
            x_out.append(float(x[i]))
            y_out.append(float(y[i]))
            phi_out.append(float(phi[i]))
            v_out.append(float(v[i]))
            a_out.append(float(a[i]))
            if i != 0:
                jerk_out.append((float(opt_a[i]) - float(opt_a[i - 1])) / 0.05)
            kappa_out.append(float(kappa[i]))
        for i in range(0, iterative_anchoring_output_size[0]):
            opt_x_out.append(float(opt_x[i]))
            opt_y_out.append(float(opt_y[i]))
            opt_phi_out.append(float(opt_phi[i]))
            opt_v_out.append(float(opt_v[i]))
            opt_a_out.append(float(opt_a[i]))
            opt_kappa_out.append(float(opt_kappa[i]))

            if i != 0:
                opt_jerk_out.append((float(opt_a[i]) - float(opt_a[i - 1])) / 0.05)
            # 0.97, 0.91
            # print(opt_v[i])
            s.append(math.sqrt((float(opt_x[i]) - last_x)**2 + (float(opt_y[i]) - last_y)**2) + acc_s)
            acc_s = s[-1]
            last_x = float(opt_x[i])
            last_y = float(opt_y[i])

            if abs(opt_v_out[-1]) < 0.001:
                stop_time.append(i)
                gear_s.append(s[-1])
            # if i != 0 and opt_kappa_out[-1] * opt_kappa_out[-2] < 0.0 :
            #     gear_s.append((s[-1] + s[-2]) * 0.5)
            
        opt_kappa_last = opt_kappa_out[stop_time[2]: stop_time[3] + 1]
        opt_kappa_out[stop_time[2]: stop_time[3] + 1] = opt_kappa_last

        print("s size is " + str(len(s)))
        print("opt_kappa_out size is " + str(len(opt_kappa_out)))

        ##################################################################################
        #write down bad curvature
        import os
        import csv
        test_root_dir = "/apollo/modules/tools/open_space_visualization"
        bad_curvature_filename = os.path.join(test_root_dir, "bad_curvature.txt")
        # with open(bad_curvature_filename, 'w') as csvfile:
        #     csv.writer(csvfile).writerows([[number] for number in opt_kappa_out])

        #read bad 
        bad_curvatures = []
        with open(bad_curvature_filename, "r") as csvfile:
            curvatures = list(csv.reader(csvfile))
            for rows in curvatures:
                bad_curvatures.append(float(rows[0]))
        bad_curvatures.append(0.0)
        
            


        ##################################################################################

        # trajectories plot
        fig1 = plt.figure(1)
        ax = fig1.add_subplot(111)
        plt.rcParams["font.weight"] = "bold"
        plt.rcParams["axes.labelweight"] = "bold"
        warm_start_arrow = None
        smoothing_arrow = None
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
                x_out[i], y_out[i], 0.25*math.cos(phi_out[i]), 0.25*math.sin(phi_out[i]), 0.2, edgecolor='r', facecolor='r')
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
                                            angle=opt_phi_out[i] * 180 / math.pi, linewidth=1, edgecolor='y', facecolor='none', alpha=0.2)
            smoothing_arrow = patches.Arrow(
                opt_x_out[i], opt_y_out[i], 0.25*math.cos(opt_phi_out[i]), 0.25*math.sin(opt_phi_out[i]), 0.2, edgecolor='y', facecolor='y')
            ax.add_patch(smoothing_car)
            ax.add_patch(smoothing_arrow)

        ax.legend([warm_start_arrow, smoothing_arrow], ['Hybrid A*', "Iterative Anchoring Smoothing"], loc='lower right', fontsize='20',bbox_to_anchor=(1.0, 0.09))
        # ax.legend(['ha','xi'])
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
            ax.plot(left_boundary_x, left_boundary_y, "k", linewidth=2)
            ax.plot(down_boundary_x, down_boundary_y, "k", linewidth=2)
            ax.plot(right_boundary_x, right_boundary_y, "k", linewidth=2)
            ax.plot(up_boundary_x, up_boundary_y, "k", linewidth=2)
        if scenario == "parallel":
            left_boundary_x = [-10.0, 0.0, 0.0]
            left_boundary_y = [0.0, 0.0, -2.5]
            down_boundary_x = [0.0, 7.0]
            down_boundary_y = [-2.5, -2.5]
            right_boundary_x = [7.0, 7.0, 17]
            right_boundary_y = [-2.5, 0.0, 0.0]
            up_boundary_x = [17.0, -10.0]
            up_boundary_y = [5.0, 5.0]
            ax.plot(left_boundary_x, left_boundary_y, "k", linewidth=2)
            ax.plot(down_boundary_x, down_boundary_y, "k", linewidth=2)
            ax.plot(right_boundary_x, right_boundary_y, "k", linewidth=2)
            ax.plot(up_boundary_x, up_boundary_y, "k", linewidth=2)

        ax.set_xlim([-10, 15])
        ax.set_ylim([-4, 6])
        plt.axis('auto')

        # input plot
        fig2 = plt.figure(2)
        v_graph = fig2.add_subplot(311)
        v_graph.set_ylabel("speed(m * $\mathregular{s^{-1}}$)", fontsize='18',fontweight='bold')
        # v_graph.set_xlabel("time(s)")
        # v_graph.plot((0.05 *float(iterative_anchoring_output_size[0]) / float(hybrid_a_output_size[0])) * np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), v_out)
        v_graph.plot(0.05 * np.linspace(0, iterative_anchoring_output_size[0], iterative_anchoring_output_size[0]), opt_v_out, linewidth=4)
        v_graph.axhline(y=2.0, color='r', linestyle='-', linewidth=2)
        v_graph.axhline(y=-1.0, color='r', linestyle='-', linewidth=2)
        for i in range(len(stop_time)):
            v_graph.axvline(x=0.05 *stop_time[i], color='y', linestyle='--', linewidth=2)


        a_graph = fig2.add_subplot(312)
        a_graph.set_ylabel("acceleration (m * $\mathregular{s^{-2}}$)", fontsize='18',fontweight='bold')
        # a_graph.set_xlabel("time(s)")
        # a_graph.plot((.05 *float(iterative_anchoring_output_size[0]) / float(hybrid_a_output_size[0])) * np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), a_out)
        a_graph.plot(0.05 * np.linspace(0, iterative_anchoring_output_size[0], iterative_anchoring_output_size[0]), opt_a_out, linewidth=4)
        a_graph.axhline(y=1.0, color='r', linestyle='-', linewidth=2)
        a_graph.axhline(y=-1.0, color='r', linestyle='-', linewidth=2)
        for i in range(len(stop_time)):
            a_graph.axvline(x=0.05 *stop_time[i], color='y', linestyle='--', linewidth=2)

        a_graph = fig2.add_subplot(313)
        a_graph.set_ylabel("jerk (m * $\mathregular{s^{-3}}$)", fontsize='18', fontweight='bold')
        a_graph.set_xlabel("time(s)", fontsize='18', fontweight='bold')
        # a_graph.plot((.05 *float(iterative_anchoring_output_size[0]) / float(hybrid_a_output_size[0])) * np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), jerk_out)
        a_graph.plot(0.05 * np.linspace(0, iterative_anchoring_output_size[0], iterative_anchoring_output_size[0]), opt_jerk_out, linewidth=4)
        a_graph.axhline(y=1.0, color='r', linestyle='-', linewidth=2)
        a_graph.axhline(y=-1.0, color='r', linestyle='-', linewidth=2)
        for i in range(len(stop_time)):
            a_graph.axvline(x=0.05 *stop_time[i], color='y', linestyle='--', linewidth=2)
        
        fig2 = plt.figure(3)
        kappa_graph = fig2.add_subplot(111)
        plt.rcParams["font.weight"] = "bold"
        plt.rcParams["axes.labelweight"] = "bold"
        kappa_graph.set_ylabel("curvature($\mathregular{m^{-1}}$)", fontsize='24',fontweight='bold')
        kappa_graph.set_xlabel("traversal distance(m)", fontsize='24',fontweight='bold')
        # kappa_graph.plot((float(iterative_anchoring_output_size[0]) / float(hybrid_a_output_size[0])) * np.linspace(0, hybrid_a_output_size[0], hybrid_a_output_size[0]), kappa_out)
        kappa_graph.plot(s, bad_curvatures, linewidth=4)
        kappa_graph.plot(s, opt_kappa_out, linewidth=4)
        kappa_graph.axhline(y=0.2, color='r', linestyle='-', linewidth=2)
        kappa_graph.axhline(y=-0.2, color='r', linestyle='-', linewidth=2)
        for i in range(len(gear_s)):
            kappa_graph.axvline(x=gear_s[i], color='y', linestyle='--', linewidth=2)
        kappa_graph.legend(['Convex Elastic Smoothing', 'Iterative Anchoring Smoothing'], loc='lower right', fontsize='20',bbox_to_anchor=(0.45, 0.7))
        plt.show()
        return [True]

    if not visualize_flag :
        if success :
            # load result
            OpenSpacePlanner.IAGetResult(x, y, phi, v, a, kappa, opt_x,
                                        opt_y, opt_phi, opt_v, opt_a, opt_kappa, 
                                        hybrid_a_output_size, iterative_anchoring_output_size,
                                        hybrid_a_time, iterative_total_time, iterative_path_time, iterative_speed_time)
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
            hybrid_a_time, iterative_total_time, iterative_path_time, iterative_speed_time, planning_time]
    
    return [False]

if __name__ == '__main__':
    visualize_flag = True

    if visualize_flag :
        sx = -6
        sy = 2.5
        SmoothTrajectory(visualize_flag, sx, sy)
    else :
        planning_time_stats = []
        hybrid_a_time_stats = []
        iterative_total_time_stats = []
        iterative_path_time_stats = []
        iterative_speed_time_stats = []

        test_count = 0
        success_count = 0
        for sx in np.arange(-8, 8, 1.0): 
            for sy in np.arange(2, 4, 0.5):
                # print("sx is "+ str(sx) + " and sy is " + str(sy))
                test_count += 1
                result = SmoothTrajectory(visualize_flag, sx, sy)
                if result[0] :
                    success_count += 1
                    planning_time_stats.append(result[-4][0] + 0.1 * result[-5][0])
                    iterative_path_time_stats.append(result[-3][0])
                    iterative_speed_time_stats.append(result[-2][0])
                    iterative_total_time_stats.append(result[-4][0])
                    hybrid_a_time_stats.append(0.1 * result[-5][0])

        if success_count != 0 :
            print("success rate is "+ str(float(success_count) / float(test_count)))            

            print("average hybird time: %4.4f, with max: %4.4f, min: %4.4f" % (
                sum(hybrid_a_time_stats) / len(hybrid_a_time_stats), max(hybrid_a_time_stats),
                min(hybrid_a_time_stats)))

            print("average iterative_total_time: %4.4f, with max: %4.4f, min: %4.4f" % (
                sum(iterative_total_time_stats) / len(iterative_total_time_stats), max(iterative_total_time_stats),
                min(iterative_total_time_stats)))

            print("average iterative_path_time: %4.4f, with max: %4.4f, min: %4.4f" % (
                sum(iterative_path_time_stats) / len(iterative_path_time_stats), max(iterative_path_time_stats),
                min(iterative_path_time_stats)))

            print("average iterative_speed_time: %4.4f, with max: %4.4f, min: %4.4f" % (
                sum(iterative_speed_time_stats) / len(iterative_speed_time_stats), max(iterative_speed_time_stats),
                min(iterative_speed_time_stats)))

            print("average total time: %4.4f, with max: %4.4f, min: %4.4f" % (
                sum(planning_time_stats) / len(planning_time_stats), max(planning_time_stats),
                min(planning_time_stats)))
            