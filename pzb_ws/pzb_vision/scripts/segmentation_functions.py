#!/usr/bin/env python

import numpy as np
import cv2 as cv2
import math
from numpy import linalg
from matplotlib import pyplot as plt

class Point:
    id = 0
    x = 0.0
    y = 0.0
    fwd = -1
    back = -1
    left = -1
    right = -1
    lines = [0., 0.]

    def __init__(self, id, x, y, lines):
        self.id = id
        self.x = x
        self.y = y
        self.lines = lines.copy()

    def dist(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def display(self):
        print("\nID: ", self.id)
        print("x: ", self.x, ", y: ", self.y)
        print("f: ", self.fwd, "b: ", self.back, "l: ", self.left, "r: ", self.right)
        print("l1: ", self.lines[0])
        print("l2: ", self.lines[1])

def p_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def get_ang(line):
    ang = math.atan2(line[2] - line[0], line[3] - line[1])
    return norm_ang(ang)

def midpoint(line):
    return [(line[0] + line[2]) / 2, (line[1] + line[3]) / 2]

def angle_diff(ang1, ang2):
    dif = (abs(ang1 - ang2) + math.pi) % (2*math.pi) - math.pi
    dif = dif + 2 * math.pi if dif < -math.pi else dif
    return dif
    return min(dif, math.pi - dif)

def norm_ang(ang):
    new_ang = (ang + math.pi) % (2*math.pi) - math.pi
    return new_ang + 2 * math.pi if new_ang < -math.pi else new_ang

def dist(p, l):
    p1 = np.array([l[0], l[1]])
    p2 = np.array([l[2], l[3]])
    p3 = np.array(p)
    return np.cross(p2-p1, p1-p3)/linalg.norm(p2-p1)

def line_ang_diff(l1, l2):
    a1 = get_ang(l1)
    a2 = get_ang(l2)
    ang_d = abs(angle_diff(a1, a2))
    return min(ang_d, math.pi - ang_d)
    return ang_d

def is_line_similar(l1, l2, ang_tres, dist_threshold):
    ang_d = line_ang_diff(l1, l2)

    d1 = abs(dist([l1[0],l1[1]], l2))
    d2 = abs(dist([l1[2],l1[3]], l2))
    d3 = abs(dist([l2[0],l2[1]], l1))
    d4 = abs(dist([l2[2],l2[3]], l1))
    d_max = max(d1, d2, d3, d4)
    d_min = min(d1, d2, d3, d4)

    if(abs(ang_d) < ang_tres and d_max < dist_threshold):
        return True
    return False

def is_line_similar2(l1, l2, ang_tres, dist_threshold):
    ang_d = line_ang_diff(l1, l2)

    m1 = midpoint(l1)
    m2 = midpoint(l2)
    dis = p_dist(m1, m2)

    if(ang_d < ang_tres and dis < dist_threshold):
        return True
    return False

def point_in_bounds(x, y, lim):
    if x > lim[0] and x < lim[1] and y > lim[2] and y < lim[3]:
        return True
    return False

def extend_line(line, lim):
    for x1,y1,x2,y2 in [line]:
        if x2 != x1:
            m = (y2 - y1) / (x2 - x1)
        else:
            y1 = lim[2]
            y2 = lim[3]
            return [int(x1),int(y1),int(x2),int(y2)]
        
        while point_in_bounds(x1-1,y1-m, lim):
            x1 -= 1
            y1 -= m
        while point_in_bounds(x2+1, y2+m, lim):
            x2 += 1
            y2 += m

        return [int(x1),int(y1),int(x2),int(y2)]
    
def intersect(l1, l2):
    if(l1[2] == l1[0]):
        l1[2] -= 1
    if(l2[2] == l2[0]):
        l2[2] -= 1
    m1 = (l1[3] - l1[1]) / (l1[2] - l1[0])
    b1 = l1[1] - m1 * l1[0]
    m2 = (l2[3] - l2[1]) / (l2[2] - l2[0])
    b2 = l2[1] - m2 * l2[0]

    if m1 == m2:
        m2 -= 1
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    return [x,y]

def get_clusters(lines):
    angles = []
    for line in lines:
        ang = get_ang(line)
        angles.append([math.cos(ang), math.sin(ang)])

    kmeans = KMeans(n_clusters=2)
    kmeans.fit(angles)
    # arr = np.array(angles)
    # plt.scatter(arr[:,0], arr[:,1], c=kmeans.labels_)
    # plt.show()
    return kmeans.labels_

def p1_less_than_p2(p1, p2):
    if p1[0] < p2[0]:
        return True
    elif p1[0] > p2[0]:
        return False
    else:
        if p1[1] < p2[1]:
            return True
        else:
            return False
        
def sort_line_points(line):
    if p1_less_than_p2(line[2:], line[0:2]):
        line = [line[2], line[3], line[0], line[1]]
    return line

def l1_less_than_l2(l1, l2):
    lims = [0, max(l1[0],l1[2],l2[0],l2[2]), 0, max(l1[1],l1[3],l2[1],l2[3])]
    l1 = extend_line(l1, lims)
    l2 = extend_line(l2, lims)
    mid1 = [(l1[0] + l1[2]) / 2, (l1[1] + l1[3]) / 2]
    mid2 = [(l2[0] + l2[2]) / 2, (l2[1] + l2[3]) / 2]
    if p1_less_than_p2(mid1, mid2):
        return True
    else:
        return False

def invert_order(line):
    max_val = max(line)
    for i in range(len(line)):
        line[i] = max_val - line[i]

# bubbleSort
def line_sort(lines, last_midpoint_coord, lim):
    line_list = []

    for i in range(len(lines)):
        lines[i] = sort_line_points(lines[i])

    ave_line = [0.] * 4
    for i in lines:
        for j in range(4):
            ave_line[j] = ave_line[j] + i[j]
    for k in range(4):
        ave_line[k] = ave_line[k] / len(lines)
    
    if ave_line[0] == ave_line[2]:
        ave_line[2] += 1
    
    ave_line[2] -= ave_line[0]
    ave_line[3] -= ave_line[1]
    ave_line[0] = 0.
    ave_line[1] = 0.

    for i in lines:
        line_list.append(dist(midpoint(i), ave_line))
    
    sorted_list = line_list.copy()
    sort(sorted_list)

    sorted_idxs = [sorted_list.index(i) for i in line_list]

    if last_midpoint_coord[0] != -1 and abs(p_dist(last_midpoint_coord, midpoint(lines[sorted_idxs.index(0)]))) > abs(p_dist(last_midpoint_coord, midpoint(lines[sorted_idxs.index(len(sorted_idxs)-1)]))):
        invert_order(sorted_idxs)
         
    return ave_line, sorted_idxs
    
def sort(arr):
    n = len(arr)
    for i in range(n-1):
        swapped = False
        for j in range(0, n-i-1):
            if arr[j] > arr[j + 1]:
                swapped = True
                tmp = arr[j]
                arr[j] = arr[j + 1]
                arr[j+1] = tmp
        if not swapped:
            return

def same_line(l1, l2):
    for i in range(4):
        if l1[i] != l2[i]:
            return False
    return True

def share_lines(p1, p2):
    for i in range(2):
        if p1.lines[i] == p2.lines[i]:
            return True, i
    return False, -1

def point_vertical_relation(p1, p2):
    if p1.fwd == -1 and p2.back == -1:
        p1.fwd = p2.id
        p2.back = p1.id

def point_horizontal_relation(p1, p2):
    if p1.left == -1 and p2.right == -1:
        p1.left = p2.id
        p2.right = p1.id

def get_point_by_id(id, point_list):
    for i in point_list:
        if i.id == id:
            return i

def same_point(p1, p2):
    if p1 is None or p2 is None:
        return False
    if p1.dist(p2) < 5:
        return True
    return False

def op_idx(i):
    return (i//2) * 2 + (i+1)%2

def find_rotation(p1, p2):
    p1_near = [p1.fwd, p1.back, p1.left, p1.right]
    p2_near = [p2.fwd, p2.back, p2.left, p2.right]

    relations = [-1] * 4
    
    for i in range(4):
        for j in range(4):
            if p1_near[i] == p2_near[j] and p1_near[i] != -1 and relations[i] != -1:
                i_opp = op_idx(i)
                if i == j:
                    relations[i] = 0
                    relations[i_opp] = 0
                elif i_opp == j:
                    relations[i] = 180
                    relations[i_opp] = 180
                elif (i + 2) % 4 == j:
                    relations[i] = 90
                    relations[i_opp] = 90
                else:
                    relations[i] = -90
                    relations[i_opp] = -90

    if relations.count(-1) > 0:
        return False, []
    else:
        return True, relations
    
def get_pair(i, pp):
    return [pp[i], pp[op_idx(i)]]

def point_translate(p, trans):
    p.lines[0] -= trans[1]
    p.lines[1] -= trans[0]
    
def point_rotate(p, point_rotation):
    tmp = [-1] * 4
    pp = [p.fwd, p.back, p.left, p.right]

    for n in range(2):
        i = n * 2
        j = op_idx(i)
        if point_rotation[n] == 0:
            tmp[i], tmp[j] = get_pair(i, pp)
        elif point_rotation[n] == 90:
            tmp[i], tmp[j] = get_pair((i + 2) % 4, pp)
        elif point_rotation[n] == -90:
            tmp[i], tmp[j] = get_pair((j + 2) % 4, pp)
        elif point_rotation[n] == 180:
            tmp[i], tmp[j] = get_pair(j, pp)
    

    p.fwd, p.back, p.left, p.right = [*tmp]

def share_adj(points, point_lists):
    adj = []
    # Fill adj with points
    for i in range(2):
        idxs = [point_lists[i][points[i]].fwd, 
                point_lists[i][points[i]].back, 
                point_lists[i][points[i]].left, 
                point_lists[i][points[i]].right]
        adj.append(idxs)
    
    # Find how many adj points are shared between both 'center' points
    shared_0 = []
    shared_1 = []
    for i in range(4):
        for j in range(4):
            if same_point(get_point_by_id(adj[0][i], point_lists[0]),
                          get_point_by_id(adj[1][j], point_lists[1])):
                if i < 2:
                    shared_0.append([i,j])
                else:
                    shared_1.append([i,j])
    if len(shared_0) == 0 or len(shared_1) == 0:
        return False, []
    
    shared = [shared_0,shared_1]
    
    # If enough points shared, find its relations
    rot_relation = []
    for i in range(2):
        rot_relation.append(get_rel(shared[i][0]))

    return True, rot_relation

def get_trans_rel(p1, p2):
    return [p2.lines[0] - p1.lines[0], p2.lines[1] - p1.lines[1]]

def get_rel(rel):
    i,j = rel
    if i == j:
        return 0
    elif op_idx(i) == j:
        return 180
    elif (i + 2) % 4 == j:
        return 90
    else:
        return -90

def h_line_similar(prev, hv_arr):
    if len(hv_arr[0]) == 0:
        return False
    if len(hv_arr[1]) == 0:
        return True
    ang1 = get_ang(hv_arr[0][0])
    ang2 = get_ang(hv_arr[1][0])
    if abs(angle_diff(prev, ang1)) < abs(angle_diff(prev, ang2)):
        return True
    else:
        return False
    
def h_line_similar2(prev_h_lines, hv_arr):
    if len(prev_h_lines) == 0:
        return False
    if len(hv_arr[0]) == 0:
        return False
    if len(hv_arr[1]) == 0:
        return True
    ang_threshold = 1.0
    dist_threshold = 50.
    for i in prev_h_lines:
        for j in hv_arr[0]:
            if is_line_similar(i, j, ang_threshold, dist_threshold):
                return True
    for i in prev_h_lines:
        for k in hv_arr[1]:
            if is_line_similar(i, k, ang_threshold, dist_threshold):
                return False
    return False

def find_offset(current_lines, past_lines):
    ang_threshold = 1.0
    dist_threshold = 50.
    for i in range(len(current_lines)):
        for j in range(len(past_lines)):
            if is_line_similar(current_lines[i], past_lines[j], ang_threshold, dist_threshold):
                return max(min(i - j, 2),-2)
    return 0.

def sorted_lines(arr, indexes):
    sorted = []
    for i in range(2):
        tmp_sorted = []
        for j in range(len(arr[i])):
            tmp_sorted.append(arr[i][indexes[i].index(j)].copy())
        sorted.append(tmp_sorted)
    return sorted

def get_sign(num):
    if num < 0:
        return -1
    elif num == 0:
        return 0
    else:
        return 1
    
def tensor_to_midpoint(box):
    arr = []
    for i in box:
        arr.append([(i[0]+i[2])/2,(i[1]+i[3])/2,])
    return arr

def cv2_conc(imgs):
    frames = imgs[0].copy()
    if len(frames.shape) == 2:
        frames = cv2.cvtColor(frames, cv2.COLOR_GRAY2RGB)

    for i in range(len(imgs)-1):
        tmp = imgs[i+1].copy()
        if len(tmp.shape) == 2:
            tmp = cv2.cvtColor(tmp, cv2.COLOR_GRAY2RGB)
        frames = np.concatenate((frames, tmp), axis=0) 
    return frames

def validate_window(in_p, out_p, lims):
    diff = out_p - in_p
    diff_valid = 1 if (diff < lims[1] and diff > lims[0]) else 0
    return [in_p, out_p, diff_valid]



def pure_ring(arr, ring, lims, n):
    in_p = 0
    out_p = 0
    last_p = 0

    windows = []

    for i in range(len(arr[0])):
        # print(len(arr) * ring // n)
        if arr[(len(arr) * ring // n)][i]:
            if last_p == 0:
                in_p = i
            out_p = i

            if len(windows) > 0 and i == len(arr[0]) - 1 and (out_p != windows[-1][1]):
                windows.append(validate_window(in_p, out_p, lims))

        else:
            if last_p != 0:
                windows.append(validate_window(in_p, out_p, lims))

        last_p = arr[len(arr) * ring // n][i]

    # print(ring, windows)
    return windows

def is_joined_window(w1, w2):
    if max(w1[0], w2[0]) < min(w1[1], w2[1]):
        return True
    return False

def pure_thresh(arr):
    window_list = []
    n = 10
    for i in range(n-1):
        window_list.append(pure_ring(arr , i+1 , [70,180], n))

    windows_pure = []
    for windows in window_list:
        for window in windows:
            if window[2]:
                for i in range(len(windows_pure)):
                    if is_joined_window(windows_pure[i], window):
                        windows_pure[i] = [min(windows_pure[i][0], window[0]), max(windows_pure[i][1], window[1]), windows_pure[i][2] + window[2]]
                    else:
                        windows_pure.append(window)
                if len(windows_pure) == 0:
                    windows_pure.append(window)

    pure_image = np.copy(arr) * 0
    for windows in windows_pure:
        if windows[2] > n * 0.5:
            window_offset = len(arr[0]) // 50
            w_lims = [windows[0] - window_offset, windows[1] + window_offset]
            pure_image[::, w_lims[0]:w_lims[1]] = arr[::, w_lims[0]:w_lims[1]]

    return pure_image
