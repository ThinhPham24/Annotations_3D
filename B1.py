import argparse
import os
import numpy as np
import d3.model.tools as mt
import functools as fc
import open3d as o3d
from plyfile import PlyData
from d3.model.basemodel import Vector
'''OUTPUT direction of files'''
outpath_merge = "/home/airlab/Desktop/Annotation3D/DATAMERGE"
outpath_down = "/home/airlab/Desktop/Annotation3D/DATADOWNSAMPLE"
outpath_down0color= "/home/airlab/Desktop/Annotation3D/DATASCALE"

def obj2ply(folder_obj, foler_ply):
    if not os.path.exists(foler_ply):
        os.mkdir(foler_ply)
    for (i,folder) in enumerate(os.listdir(folder_obj)):
        if not os.path.exists(foler_ply+'{}'.format(folder)):
            os.mkdir(foler_ply+'{}'.format(folder))
        for (j,file) in enumerate(os.listdir(folder_obj+folder)):
            name=file.strip('.obj').split("_",2)
            #print(foler_ply+'/'+folder+'/'+'{}_{}.ply'.format(name[1],name[2]))
            up_conversion = None
            if args.from_up is not None:
                up_conversion = (args.from_up, args.to_up)
    #duong dan luu ket qua
            output= os.path.join(foler_ply + folder + '/' + '{}_{}.ply'.format(name[1], name[2]))
            #print(output)
            result = mt.convert(folder_obj+folder+'/'+file, output, up_conversion)
            if args.output is None:
                print(result)
            else:
                with open(os.path.join(foler_ply+'/'+folder+'/'+'{}_{}.ply'.format(name[1],name[2])),'w') as f:
                    f.write(result)
def merge(int_path,out_path):
    if not os.path.exists(out_path):
        os.mkdir(out_path)
    for (i, folder) in enumerate(os.listdir(int_path)):
        #print(out_path+'/'+'{}'.format(folder))
        vertexes = []
        faces = []
        total_ver = 0
        if not os.path.exists(out_path+'/'+'{}'.format(folder)):
            os.mkdir(out_path+'/'+'{}'.format(folder))
        for (j,path) in enumerate(os.listdir(int_path+'/'+folder)):
            labels = path.split("_")
            label = int(labels[0])
            #read mesh by o3d triangle
            mesh = o3d.io.read_triangle_mesh(int_path +'/'+folder+'/'+ "{}".format(path))
            #split points and face
            vertex_buff = np.asarray(mesh.vertices)
            #split points and face
            face_buff = np.asarray(mesh.triangles)
            num_ver = vertex_buff.shape[0]
            num_face = face_buff.shape[0]
            #insert data in array
            for (_, inx) in enumerate(vertex_buff):
                vertexes.append([inx[0], inx[1], inx[2], label])
            for (_, ins) in enumerate(face_buff):
                faces.append([ins[0] + total_ver, ins[1] + total_ver, ins[2] + total_ver])
            total_ver = total_ver + num_ver
        with open(os.path.join(out_path+'/'+folder+'/'+'{}_final.ply'.format(folder)),'w') as f:
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write('element vertex %d\n' % len(vertexes))
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            f.write('property uchar red\n')
            f.write('property uchar green\n')
            f.write('property uchar blue\n')
            f.write('property uchar label\n')
            f.write('element face %d\n' % len(faces))
            f.write('property list uchar int vertex_indices\n')
            f.write('end_header\n')
            for (i, inv) in enumerate(vertexes):
                if inv[3] == 41:
                    f.write("".join(
                        '{} {} {} {} {} {} {}'.format(round(inv[0], 2), round(inv[1], 2), round(inv[2], 2), 0, 255, 0,
                                                      inv[3])) + "\n")
                if inv[3] == 42:
                    f.write("".join(
                        '{} {} {} {} {} {} {}'.format(round(inv[0], 2), round(inv[1], 2), round(inv[2], 2), 255, 0, 0,
                                                      inv[3])) + "\n")
                if inv[3] == 43:
                    f.write("".join(
                        '{} {} {} {} {} {} {}'.format(round(inv[0], 2), round(inv[1], 2), round(inv[2], 2), 0, 0, 255,
                                                      inv[3])) + "\n")
                if inv[3] == 44:
                    f.write("".join(
                        '{} {} {} {} {} {} {}'.format(round(inv[0], 2), round(inv[1], 2), round(inv[2], 2), 0, 0, 0,
                                                      inv[3])) + "\n")
            for (i, inf) in enumerate(faces):
                f.write("".join('{} {} {} {}'.format(3, round(inf[0], 2), round(inf[1], 2), round(inf[2], 2))) + "\n")
            f.close()
def downsample(int_path,out_path):
    if not os.path.exists(out_path):
        os.mkdir(out_path)
    for (i, folder) in enumerate(os.listdir(int_path)):
        print(out_path+'/'+'{}'.format(folder))
        if not os.path.exists(out_path+'/'+'{}'.format(folder)):
            os.mkdir(out_path+'/'+'{}'.format(folder))
        for (j, path) in enumerate(os.listdir(int_path + '/' + folder)):
            print("Testing mesh in open3d ...")
            mesh1 = o3d.io.read_triangle_mesh(int_path + '/' + folder + "/{}".format(path))  # choose one in two
            downpcd = o3d.geometry.sample_points_poisson_disk(mesh1, 2048, init_factor=5, pcl=None)
            num_v = np.asarray(downpcd.points).shape[0]
            o3d.geometry.estimate_normals(
                downpcd,
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                                  max_nn=30))
            norm = np.asarray(downpcd.normals)
            points = np.asarray(downpcd.points)
            colors = np.asarray(downpcd.colors)
            n_v= points.shape[0]
            with open(out_path+'/'+'{}'.format(folder + "/{}.ply".format(folder)), "w") as f:
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
                for (i, inx) in enumerate(points):
                    f.write('\n')
                    if round(colors[i][0], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 3), round(inx[1], 3), round(inx[2], 3), round(norm[i][0]), round(norm[i][1]),round(norm[i][2]), 255, 0, 0, 42)))
                    elif round(colors[i][1], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 3), round(inx[1], 3), round(inx[2], 3), round(norm[i][0]), round(norm[i][1]),round(norm[i][2]), 0, 255, 0, 41)))
                    elif round(colors[i][2], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 3), round(inx[1], 3), round(inx[2], 3),round(norm[i][0]), round(norm[i][1]),round(norm[i][2]) ,0, 0, 255, 43)))
                    else:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {} {}'.format(round(inx[0], 3), round(inx[1], 3), round(inx[2], 3), round(norm[i][0]), round(norm[i][1]),round(norm[i][2]), 0, 0, 0, 44)))
                f.close()
def downsample2(int_path,out_path):
    if not os.path.exists(out_path):
        os.mkdir(out_path)
    for (i, folder) in enumerate(os.listdir(int_path)):
        print(out_path+'/'+'{}'.format(folder))
        if not os.path.exists(out_path+'/'+'{}'.format(folder)):
            os.mkdir(out_path+'/'+'{}'.format(folder))
        for (j, path) in enumerate(os.listdir(int_path + '/' + folder)):
            print("Testing mesh in open3d ...")
            mesh1 = o3d.io.read_triangle_mesh(int_path + '/' + folder + "/{}".format(path))  # choose one in two
            downpcd = o3d.geometry.sample_points_poisson_disk(mesh1, 2048, init_factor=5, pcl=None)
            num_v = np.asarray(downpcd.points).shape[0]
            o3d.geometry.estimate_normals(
                downpcd,
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2,
                                                                  max_nn=30))
            norm = np.asarray(downpcd.normals)
            points = np.asarray(downpcd.points)
            colors = np.asarray(downpcd.colors)
            n_v = points.shape[0] #number points
            with open(out_path+'/'+'{}'.format(folder + "/{}_finaldown.ply".format(folder)), "w") as f:
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
                f.write('element face %d\n' % 0)
                f.write('end_header')
                for (i, inx) in enumerate(points):
                    f.write('\n')
                    if round(colors[i][0], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {}'.format(round(inx[0]/1000, 5), round(inx[1]/1000, 5), round(inx[2]/1000, 5),
                                                                   round(norm[i][0]), round(norm[i][1]),
                                                                   round(norm[i][2]), 255, 0, 0)))
                    elif round(colors[i][1], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {}'.format(round(inx[0]/1000, 5), round(inx[1]/1000, 5), round(inx[2]/1000, 5),
                                                                   round(norm[i][0]), round(norm[i][1]),
                                                                   round(norm[i][2]), 0, 255, 0)))
                    elif round(colors[i][2], 0) == 1:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {}'.format(round(inx[0]/1000, 5), round(inx[1]/1000, 5), round(inx[2]/1000, 5),
                                                                   round(norm[i][0]), round(norm[i][1]),
                                                                   round(norm[i][2]), 0, 0, 255)))
                    else:
                        f.write("".join(
                            '{} {} {} {} {} {} {} {} {}'.format(round(inx[0]/1000, 5), round(inx[1]/1000, 5), round(inx[2]/1000, 5),
                                                                   round(norm[i][0]), round(norm[i][1]),
                                                                   round(norm[i][2]), 0, 0, 0)))
                f.close()
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    #parser.set_defaults(func=obj2ply)
    parser.add_argument('-v', '--version', action='version', version='1.0')
    parser.add_argument("-i","--input", help="display a square of a given number",type=str)
    parser.add_argument("-o", "--output", help="display a square of a given number", type=str)
    parser.add_argument('-t', '--type', metavar='type',
                        help='Export type, useless if output is specified')
    parser.add_argument('-fu', '--from-up', metavar='fup', default=None,
                        help="Initial up vector")
    parser.add_argument('-tu', '--to-up', metavar='fup', default=None,
                        help="Output up vector")
    args = parser.parse_args()
    #args.func(args)
    objFolder = args.input
    plyFolder = args.output
    #run step 1 (convert each obj to each ply)
    obj2ply(objFolder, plyFolder)
    int_path_merge= "/home/airlab/Desktop/Annotation3D/DATA" #fix here
    #run step2 (merge each plys into one ply)
    merge(int_path_merge,outpath_merge)
    #run step 3 (downsample 2048 point)
    int_path_down= "/home/airlab/Desktop/Annotation3D/DATAMERGE" #fix here direction
    downsample(int_path_down,outpath_down)
    #run code downsample without color and scale around metric
    downsample2(int_path_down,outpath_down0color)
    #run step 4 (Augmentation)
    #run step 5 (convert ply to txt)
    #run step 6 (make json)
    #############################################

    '''run code python B1.py -i <folder direction of obj file> -o <folder direction>'''
'''example: python B1.py -i /home/airlab/Desktop/Annotation3D/DATACUTTING/ -o /home/airlab/Desktop/Annotation3D/DATA/
'''
'''Run code with open3d==0.7.0.0
    install : pip install plyfile
    pip install open3d-python
    include : d3 folder
    Introduction: We should prepare "DATACUTTING" that defines each part's name correctly.
    - Firstly, we have a folder containing all the data of each object, consisting of each part being separated and named correctly with it. Eg: leaves_41_1.obj
    It will be saved in the DATA folder after being converted from obj to ply.
    - Secondly, in the "DATA" folder containing each part of one object, it will be merged into one object, completely. It will be saved under the name "DATAMERGE".
    - Then, in "DATAMERGE", we will start downsampling the 2048 point cloud and saved it to two folders named "DATADOWNSAMPLE" and "DATASCALE", "DATASCALE" contains items that are scaled in the metric.
    - Finally, we have all the files. From in the "DATASCALE" folder, you divide into two folder named "DATATRAIN" and "DATATEST", respectively.     
'''
