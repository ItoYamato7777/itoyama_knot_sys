import numpy as np
import open3d as o3d
import cv2
from sklearn.decomposition import PCA
from sklearn.cluster import DBSCAN
from geo import FRAME, MATRIX, VECTOR


def plot_points(points, colors = None):
    '''
    plot pointcloud
    When using o3d.geometry.PointCloud;
    in: points: data
    
    When using np.array;
    in: points: np.array([[x1,y1,z1],
                          [x2,y2,z2],
                              :
                          [xn,yn,zn]])

        colors(opt.): np.array([[b1,g1,r1],
                                [b2,g2,r2],
                                    :
                                [bn,gn,rn]])
    '''
    if type(points) is o3d.geometry.PointCloud:
        pcd = points

    elif type(points) is np.ndarray:
        # remove np.nan
        is_point = np.where(np.logical_not(np.isnan(points)))
        points = points[is_point]
        # reshape
        points = points.reshape((int(points.size/3),3))
        # convert to py3d.PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
    
        if not colors is None:
            colors = np.array(colors[is_point], np.uint8)
            colors = colors.reshape((1, int(colors.size/3), 3))
            colors = cv2.cvtColor(colors, cv2.COLOR_BGR2RGB) / 255.0
            pcd.colors = o3d.utility.Vector3dVector(colors[0])
  
    o3d.visualization.draw_geometries([pcd])

    
def coordinate_transform(frame, points):
    """
    in: 1. frame: transform matrix(T) by geo.FRAME or np.array
        2. points: np.array([[x1,y1,z1], [x2,y2,z2], ... , [xn,yn,zn]])
    
    out: transformed points: np.array([[x1',y1',z1'], [x2',y2',z2'], ... , [xn',yn',zn']])
    [[xi'],[yi'],[zi'],[1]] = np.dot(T.toarray(), [[xi],[yi],[zi],[1]])
    """
    shape = points.shape  
    pm = points.reshape(int(points.size/3), 3).T

    if isinstance(frame, FRAME):
        frame = frame.toarray()
        
    mat = frame[0:3, 0:3]
    vec = frame[0:3, 3:]
    newpm = (np.dot(mat, pm) + vec).T.reshape(shape)
    return newpm
  
def extraction(points, func, frame = np.eye(4)):
    """
    in: 1. points: np.array([[x1,y1,z1], [x2,y2,z2], ... , [xn,yn,zn]])
        2. func: lambda x,y,z: ________
        3. frame: FRAME or np.array 
    out: extracted points
  
    some examples of func:
    lambda x,y,z: x**2+y**2+z**2 < 1
    lambda x,y,z: np.argmax(x)
    """
    frm_inv = -FRAME(frm = frame)
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    tmp = coordinate_transform(frm_inv, points)
    mask = func(tmp[:,0],tmp[:,1],tmp[:,2])
    return np.array([x[mask],y[mask],z[mask]]).T
  
def pca(dat):
    '''
    In : dat
    Out : 1. mean
          2. components
          3. transformed
    transformed = np.dot((dat - mean), components.T)
    '''
    pca_obj = PCA()
    
    transformed = pca_obj.fit_transform(dat)
    mean = pca_obj.mean_
    components = pca_obj.components_
    return mean, components, transformed



def dbscan(points, eps = 0.03, min_samples = 10):
    """
    This is DBSCAN clustering function.
    Parameters:
    1:points: n * 3 pointcloud.
    2:eps
    3:min_samples
    Return value:
    1:Number of clusters
    2:Labels of data
    3:High confidence label(Data type is "bool". Size is same to data)
    """
    db = DBSCAN(eps, min_samples).fit(points)
    high_confidence_data_mask = np.zeros_like(db.labels_, dtype=bool)
    high_confidence_data_mask[db.core_sample_indices_] = True
    
    # Number of clusters is length of db.labels_.
    # But if -1 in the "db.labels_" number of clusters should decrease 1.
    num_clusters = len(set(db.labels_)) - (1 if -1 in db.labels_ else 0)

    return num_clusters, db.labels_, high_confidence_data_mask
