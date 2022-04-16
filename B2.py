import argparse
import os
import numpy as np
import d3.model.tools as mt
import functools as fc
import open3d as o3d
from plyfile import PlyData
from d3.model.basemodel import Vector
import math
import sys
import copy
import random
import string
import json
'''OUTPUT direction of files'''
#outpath_augment= "/home/airlab/Desktop/Annotation3D/DATA_AUG_TRAIN"
#fix here if you want to create test dataset
outpath_augment ="/home/airlab/Desktop/Annotation3D/DATA_AUG_TEST"
outpath_txt="/home/airlab/Desktop/Annotation3D/shape_data"
# Calculates Rotation Matrix given euler angles.
def eul2rot(theta) :
    theta[0]=theta[0]*180/3.14
    theta[1] = theta[1] * 180 / 3.14
    theta[2] = theta[2] * 180 / 3.14
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R
def write_ply_aug(out_filename, points, norm, colors):
    with open(out_filename, "w") as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write('element vertex %d\n' % 2048)
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property float nx\n')
        f.write('property float ny\n')
        f.write('property float nz\n')
        f.write('property uchar red\n')
        f.write('property uchar green\n')
        f.write('property uchar blue\n')
        f.write('element face %d\n' % 0)
        f.write('end_header')
        for (i, inx) in enumerate(points):
            f.write('\n')
            if round(colors[i][0], 0) == 1:
                f.write("".join(
                    '{} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1] , 5), round(inx[2] , 5), round(norm[i][0]), round(norm[i][1]), round(norm[i][2]), 255, 0, 0)))
            elif round(colors[i][1], 0) == 1:
                f.write("".join(
                    '{} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5), round(norm[i][0]), round(norm[i][1]), round(norm[i][2]), 0, 255, 0)))
            elif round(colors[i][2], 0) == 1:
                f.write("".join(
                    '{} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5), round(norm[i][0]), round(norm[i][1]), round(norm[i][2]), 0, 0, 255)))
            else:
                f.write("".join(
                    '{} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5),round(inx[2], 5), round(norm[i][0]), round(norm[i][1]), round(norm[i][2]), 0, 0, 0)))
        f.close()
def write_ply_label(out_filename, points, norm, colors):
    with open(out_filename, "w") as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write('element vertex %d\n' % 2048)
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        f.write('property float nx\n')
        f.write('property float ny\n')
        f.write('property float nz\n')
        f.write('property uchar label\n')
        f.write('element face %d\n' % 0)
        f.write('end_header')
        for (i, inx) in enumerate(points):
            f.write('\n')
            if round(colors[i][0], 0) == 1:
                f.write("".join(
                    '{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5),
                                                  round(inx[2], 5), round(norm[i][0]), round(norm[i][1]),
                                                  round(norm[i][2]), 42)))
            elif round(colors[i][1], 0) == 1:
                f.write("".join(
                    '{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5),
                                                  round(inx[2], 5), round(norm[i][0]), round(norm[i][1]),
                                                  round(norm[i][2]), 41)))
            elif round(colors[i][2], 0) == 1:
                f.write("".join(
                    '{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5),
                                                  round(inx[2], 5), round(norm[i][0]), round(norm[i][1]),
                                                  round(norm[i][2]), 43)))
            else:
                f.write("".join(
                    '{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5),
                                                  round(inx[2], 5), round(norm[i][0]), round(norm[i][1]),
                                                  round(norm[i][2]), 44)))
        f.close()
def augment(int_path,out_path,number,scale):
    if not os.path.exists(out_path):
        os.mkdir(out_path)
   # files = os.listdir(int_path)
    #print(out_path)
    for (i,folder) in enumerate(os.listdir(int_path)):
        for (j,file) in enumerate(os.listdir(int_path+'/'+folder)):
            #print(int_path+'/'+folder)
            pcd = o3d.io.read_point_cloud(int_path + "/" + folder+"/"+"{}".format(file))
            pcd_s = copy.deepcopy(pcd)
            pcd_s.scale(scale, center=pcd.get_center())
            if not os.path.exists(out_path + '/' + '{}'.format(file.split('.')[0])):
                os.makedirs(out_path + '/' + '{}'.format(file.split('.')[0]))
            for k in range(0,number):
                pcd_ra = copy.deepcopy(pcd)
                pcd_r = copy.deepcopy(pcd_s)
                rotate_r_x = pcd_r.rotate(pcd.get_rotation_matrix_from_xyz((0.15 * k *np.pi, 0, 0)), center=pcd.get_center())
                rotate_r_x1 = pcd_ra.rotate(pcd.get_rotation_matrix_from_xyz((0.15 * k*np.pi, 0, 0)), center=pcd.get_center())
                rotate_r_y = pcd_r.rotate(pcd.get_rotation_matrix_from_xyz((0, 0.15 * k * np.pi, 0)), center=pcd.get_center())
                rotate_r_y1 = pcd_ra.rotate(pcd.get_rotation_matrix_from_xyz((0, 0.15 * k * np.pi, 0)), center=pcd.get_center())
                rotate_r_z = pcd_r.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, 0.15 * k * np.pi)), center=pcd.get_center())
                rotate_r_z1 = pcd_ra.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, 0.15 * k * np.pi)), center=pcd.get_center())
                rotate_r_x_y_z = pcd_r.rotate(pcd.get_rotation_matrix_from_xyz((0.25 * k * np.pi, 0.35 * k * np.pi, 0.15 * k * np.pi)), center=pcd.get_center())
                rotate_r_x_y_z = pcd_ra.rotate(pcd.get_rotation_matrix_from_xyz((0.25 * k * np.pi, 0.35 * k * np.pi, 0.15 * k * np.pi)), center=pcd.get_center())
                # out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}.ply'.format(k) is direction
                #data of rotate around x axis
                points_x = np.asarray(rotate_r_x.points)
                norm_x = np.asarray(rotate_r_x.normals)
                colors_x = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(k,scale), points_x, norm_x, colors_x)
                points_x1 = np.asarray(rotate_r_x.points)
                norm_x1 = np.asarray(rotate_r_x.normals)
                colors_x1 = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(number+k,scale), points_x1, norm_x1, colors_x1)
                # data of rotate around y axis
                points_y = np.asarray(rotate_r_x.points)
                norm_y = np.asarray(rotate_r_x.normals)
                colors_y = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(2*number+k,scale), points_y,norm_y, colors_y)
                points_y1 = np.asarray(rotate_r_x.points)
                norm_y1 = np.asarray(rotate_r_x.normals)
                colors_y1 = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(3*number + k,scale), points_y1, norm_y1, colors_y1)
                # data of rotate around z axis
                points_z = np.asarray(rotate_r_x.points)
                norm_z = np.asarray(rotate_r_x.normals)
                colors_z = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(4*number + k,scale), points_z, norm_z, colors_z)
                points_z1 = np.asarray(rotate_r_x.points)
                norm_z1 = np.asarray(rotate_r_x.normals)
                colors_z1 = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(5*number + k,scale), points_z1, norm_z1, colors_z1)
                # data of rotate around x_y_z axis
                points_x_y_z = np.asarray(rotate_r_x.points)
                norm_x_y_z = np.asarray(rotate_r_x.normals)
                colors_x_y_z = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(6 * number + k,scale), points_x_y_z, norm_x_y_z, colors_x_y_z)
                points_x_y_z1 = np.asarray(rotate_r_x.points)
                norm_x_y_z1 = np.asarray(rotate_r_x.normals)
                colors_x_y_z1 = np.asarray(rotate_r_x.colors)
                write_ply_aug(out_path + '/' + '{}'.format(file.split('.')[0]) + '/' + '{}_{}.ply'.format(7 * number + k,scale), points_x_y_z1, norm_x_y_z1, colors_x_y_z1)
def ply2tx(folder_path,output_path):
    if not os.path.exists(output_path):
        os.mkdir(output_path)
    if not os.path.exists(output_path+'/'+'35792468'): #fix here if you want to create test dataset 35792468
        os.mkdir(output_path+'/'+'35792468')
    pathcheck="/home/airlab/Desktop/Annotation3D"
    if not os.path.exists(pathcheck+'/'+'check1'):#fixe here check
        os.mkdir(pathcheck+'/'+'check1')
    path = os.listdir(folder_path)
    for (i,folder) in enumerate(path):
        path_1= os.listdir(folder_path+'/'+folder)
        for (j,file) in enumerate(path_1):
            pcd = o3d.io.read_point_cloud(folder_path + '/' + folder + '/' + file)
            points = np.asarray(pcd.points)
            norm = np.asarray(pcd.normals)
            colors = np.asarray(pcd.colors)
            n_v = points.shape[0]
            #=============================================
            final_file_name = ''.join(random.choice(string.ascii_letters) for _ in range(17))
            with open(output_path+'/'+'35792468'+'/'+final_file_name + ".txt", "w") as f: #fix here if you want to create test dataset
                for (k, inx) in enumerate(points):
                    if k>0:
                        f.write('\n')
                    if round(colors[k][0], 0) == 1:
                        f.write("".join('{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5), round(norm[k][0]), round(norm[k][1]),round(norm[k][2]), 42)))
                    elif round(colors[k][1], 0) == 1:
                        f.write("".join('{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5), round(norm[k][0]), round(norm[k][1]),round(norm[k][2]), 41)))
                    elif round(colors[k][2], 0) == 1:
                        f.write("".join('{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5),round(norm[k][0]), round(norm[k][1]),round(norm[k][2]), 43)))
                    else:
                        f.write("".join('{} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5), round(norm[k][0]), round(norm[k][1]),round(norm[k][2]), 44)))
                f.close()
            with open(pathcheck+'/'+'check1'+'/'+final_file_name + ".ply","w") as f:
                f.write('ply\n')
                f.write('format ascii 1.0\n')
                f.write('element vertex %d\n' % n_v)
                f.write('property float x\n')
                f.write('property float y\n')
                f.write('property float z\n')
                f.write('property float nx\n')
                f.write('property float ny\n')
                f.write('property float nz\n')
                f.write('property uchar red\n')
                f.write('property uchar green\n')
                f.write('property uchar blue\n')
                f.write('property uchar label\n')
                f.write('element face %d\n' % 0)
                f.write('end_header')
                for (h, inx) in enumerate(points):
                    f.write('\n')
                    if round(colors[h][0], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5),
                                                                   round(norm[h][0]), round(norm[h][1]),
                                                                   round(norm[h][2]), 255, 0, 0, 42)))
                    elif round(colors[h][1], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5),
                                                                   round(norm[h][0]), round(norm[h][1]),
                                                                   round(norm[h][2]), 0, 255, 0, 41)))
                    elif round(colors[h][2], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5),
                                                                   round(norm[h][0]), round(norm[h][1]),
                                                                   round(norm[h][2]), 0, 0, 255, 43)))
                    else:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 5), round(inx[1], 5), round(inx[2], 5),
                                                                   round(norm[h][0]), round(norm[h][1]),
                                                                   round(norm[h][2]), 0, 0, 0, 44)))
                f.close()
def make_json(path):
    file_for_json = []
    for folder in os.listdir(path):
        for file in os.listdir(os.path.join(path,folder)):
            # print()
            path_file = path+"/"+folder+"/"+file
            file_for_json.append(path_file[0:-4])

            # print(os.path.join(path,folder)+"/"+file)

    with open('../PARTSEG/NET++/data/shapenetcore_partanno_segmentation_benchmark_v0_normal/train_test_split/shuffled_test_file_list.json', 'w') as f: #fix here when you change test
        print("here")
        json.dump(file_for_json, f)
if __name__=="__main__":
    parser = argparse.ArgumentParser("Code processing")
    parser.add_argument("-i", "--input", help="display a square of a given number", type=str)
    args = parser.parse_args()
    intfolder = args.input
    '''step 1 augment dataset  run first and close step 2 and step3 to change run code : for train run code: " python B2.py -i /home/airlab/Desktop/Annotation3D/DATATRAIN"
    For test, you change running code " python B2.py -i /home/airlab/Desktop/Annotation3D/DATATEST"
    '''
    #step 1, augment
    repeat = 10
    scale = [0.7, 0.85, 1.15, 1.3]
    for [i, _] in enumerate(scale):
        augment(intfolder, outpath_augment, repeat, scale[i])
    ##############################################################
    #IN TWO CASE, you want to create train file and test file

    #step 2 convert ply to txt
    ply2tx(outpath_augment,outpath_txt)
    #step 3 make json
    make_json('shape_data')