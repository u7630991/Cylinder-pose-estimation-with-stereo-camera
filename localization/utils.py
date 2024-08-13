import numpy as np
import sklearn.metrics.pairwise
from scipy import sparse
import operation

def gen_rectangle_edge_points(x_width, y_width, density=10):
    '''Generate an array of points evenly distributed on the edges of a rectangle.
    Assumed the rectangle is center-aligned at the origin (0,0)

    Args:
        x_width: Width in x direction
        y_width: Width in y direction
        density: How many points distributed along the edge

    Returns:
        A 2D Numpy array for the 2D points
    '''
    min_x, max_x = -x_width/2, x_width/2
    min_y, max_y = -y_width/2, y_width/2

    series_x = np.linspace(min_x, max_x, num=density+1)
    series_y = np.linspace(min_y, max_y, num=density+1)

    pts = []
    for x in series_x:
        for y in series_y:
            if x>min_x and x<max_x and y>min_y and y<max_y:
                continue
            pts.append((x,y))

    return np.array(pts)
    
    
def gen_cuboid_wall_points(height, base_width, density=20):
    '''Generate an array of points evenly distributed on the vertical walls of a cuboid.
    Assumed the cuboid is placed upright with the base center at the origin (0,0,0)

    Args:
        height: Height of the cuboid
        base_width: Width of the square base
        density: How many points distributed along the edge

    Returns:
        A 2D Numpy array for the 3D points
    '''
    min_pt = -base_width/2
    max_pt = base_width/2

    ### Construct the evenly spaced 'point cloud' of the cuboid
    # [Primary] at even layers:  corner vertices included
    # [Secondary] at odd layers: corner vertices excluded

    series_primary_x = np.linspace(min_pt, max_pt, num=density+1)
    series_primary_y = np.linspace(min_pt, max_pt, num=density+1)

    step = base_width / density
    series_secondary_x = series_primary_x[:-1] + step/2
    series_secondary_y = series_primary_y[:-1] + step/2

    num_step_z = int(height/step + 1)

    pts = []

    for layer in range(num_step_z):
        z = step * layer 

        if layer%2 == 0:
            for x in series_primary_x:
                for y in series_primary_y:
                    if x>min_pt and x<max_pt and y>min_pt and y<max_pt:
                        continue
                    pts.append((x,y,z))
        else:
            for x in series_secondary_x:
                for y in (min_pt, max_pt):
                    pts.append((x,y,z))
            for y in series_secondary_y:
                for x in (min_pt, max_pt):
                    pts.append((x,y,z))

    return np.array(pts)

    

def gen_cylinder_circumference_points(height, base_radius, density=20):
    '''Generate an array of points evenly distributed on the circumference surface of a cylinder.
    Assumed the cylinder is placed upright with the base center at the origin (0,0,0)

    Args:
        height: Height of the cylinder
        base_radius: Radius of the circle base
        density: How many points distributed along the circumference

    Returns:
        A 2D Numpy array for the 3D points
    '''

    #if height>10:
     #density = height*2
  
    min_pt = -base_radius/2
    max_pt = base_radius/2

    ### Construct the evenly spaced 'point cloud' of the cylinder
    # [Primary] at even layers:  0 degree vertices included
    # [Secondary] at odd layers: 0 degree vertices excluded

    series_primary_x = np.linspace(min_pt, max_pt, num=density+1)
    series_primary_u = np.linspace(0, 2 * np.pi, density+1)

    step = base_radius / density
    step_u = 2*np.pi / (density)

    series_secondary_x = series_primary_x[:-1] + step/2
    series_secondary_u = series_primary_u[:-1] + step_u/2

    num_step_z = int(density + 1)

    pts = []

    for layer in range(num_step_z):
        z = height/density * layer 

        if layer%2 == 0:
                    for u in series_primary_u:
                        pts.append(( base_radius * np.cos(u), base_radius *  np.sin(u),z))
        else:
                      for u in series_secondary_u:
                        pts.append(( base_radius * np.cos(u), base_radius *  np.sin(u),z))


    return np.array(pts)

def gen_half_cylinder_circumference_points(height, base_radius, density=20):
    '''Generate an array of points evenly distributed on the circumference surface of a cylinder.
    Assumed the cylinder is placed upright with the base center at the origin (0,0,0)

    Args:
        height: Height of the cylinder
        base_radius: Radius of the circle base
        density: How many points distributed along the circumference

    Returns:
        A 2D Numpy array for the 3D points
    '''

    #if height>10:
     #density = height*2
  
    min_pt = -base_radius/2
    max_pt = base_radius/2

    ### Construct the evenly spaced 'point cloud' of the cylinder
    # [Primary] at even layers:  0 degree vertices included
    # [Secondary] at odd layers: 0 degree vertices excluded

    series_primary_x = np.linspace(min_pt, max_pt, num=density+1)
    series_primary_u = np.linspace(0,   np.pi/4, density+1)

    step = base_radius / density
    step_u =  np.pi / (density)

    series_secondary_x = series_primary_x[:-1] + step/2
    series_secondary_u = series_primary_u[:-1] + step_u/2

    num_step_z = int(density + 1)

    pts = []

    for layer in range(num_step_z):
        z = height/density * layer 

        if layer%2 == 0:
            for x in series_primary_x:
                    for u in series_primary_u:
                        if (x>min_pt) and (x<max_pt) :
                            continue
                        pts.append(( base_radius * np.cos(u), base_radius *  np.sin(u),z))
        else:
            for x in series_secondary_x:
                      for u in series_secondary_u:
                        pts.append(( base_radius * np.cos(u), base_radius *  np.sin(u),z))


    return np.array(pts)


def compare_rectangles(pts, pts_ref):
    '''Compare the edge points of a rectangle to those of the reference one

    Args:
        pts: a 2D Numpy array of 2D points for the rectangle to be examined
        pts_ref: a 2D Numpy array of 2D points for the reference rectangle

    Returns:
        The score of dissimilarity; A non-negative integer. 
        0 means 'the same'. The smaller the score, the more similar they are.
    '''   
    ### In this approach, we try to estimate the dissimilarity of two shapes
    ### by bucketing into low-resolution images
    BOUND = 16
    IMG_SIZE = BOUND*2

    # Draw the points of the rectangles on an image respectively
    img = np.zeros((IMG_SIZE, IMG_SIZE), np.uint8)
    img_ref = np.zeros((IMG_SIZE, IMG_SIZE), np.uint8)

    for (curr_pts, curr_img) in [(pts, img), (pts_ref, img_ref)]:
        for pt in curr_pts:
            x, y = pt[0], pt[1]
            if x > -BOUND and x <= BOUND and y > -BOUND and y <= BOUND:
                row, col = round(y+BOUND-1), round(x+BOUND-1)
                curr_img[row, col] = 1

    # Count how many of 1's overlap
    num_not_matched_ref = np.sum((img != img_ref) & (img_ref>0))

    return num_not_matched_ref

def compare_cylinders(pts, pts_ref):
    '''Compare the edge points of a rectangle to those of the reference one

    Args:
        pts: a 3D Numpy array of 3D points for the cylinder to be examined
        pts_ref: a 3D Numpy array of 3D points for the reference cylinder

    Returns:
        The score of similarity; A float number from 0 to 1. 
        1 means 'the same'. The closer the score to 1, the more similar they are.
    '''
    #Convert the input 3D arrays into sparse matrix   
    sA = sparse.csr_matrix(pts_ref)
    sB = sparse.csr_matrix(pts)

    #Compare the difference of the cylinders with cosine similarity
    ref = sklearn.metrics.pairwise.cosine_similarity(sA, sA)
    similarity = sklearn.metrics.pairwise.cosine_similarity(sA, sB)

    (row, col) = pts.shape
    sumxy = 0
    sumxy_ref = 0

    for i in range(row):
       for j in range(col):
           if (i == j) :
            sumxy = sumxy + similarity[i][j]
            sumxy_ref = sumxy_ref + ref [i][j]

    return (sumxy/sumxy_ref)