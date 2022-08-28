#install Mesh_Labeler-main.zip

# 3D_segmentation_annotation.

#You should prepare dataset that is separated and save each part into a folder.

#First, run B1.py: create new enviroment.

-To run this code, you should install librarys:
- pip install open3d-python (version --0.7.00)
- pip install plyfile
- (1) You should fix something about  input and output folder direction to match your folder
- (2) Eg: run code "python B1.py -i /home/airlab/Desktop/Annotation3D/DATACUTTING/ -o /home/airlab/Desktop/Annotation3D/DATA/"
- After you select object for train and test. And save it into two folder named: "DATATRAIN and DATATEST".

#Then run B2.py: create new enviroment.

-Intruction: create new enviroment 
- (1) Install: 
 -  pip install open3d=0.13.0.0
 -  pip install plyfile
 -  add d3 folder
- (2) you should fix something about direction
    * Eg: run code: "python B2.py -i /home/airlab/Desktop/Annotation3D/DATATRAIN" and "python B2.py -i /home/airlab/Desktop/Annotation3D/DATATEST".
    
#Library is to process 3D point cloud:

- open3d
- plyfile
- trimesh
- pyVista
- openGL
- PCL with C++
This project is a work in progress.
