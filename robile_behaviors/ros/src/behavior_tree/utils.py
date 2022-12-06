import numpy as np
from geometry_msgs.msg import Twist
import random

class Line:
    """An object used to create a line from two points

    :param start: First point used to generate the line. It's an
                  array of form [x,y].
    :type start: numpy.ndarray
    :param end: Second point used to generate the line. It's an
                array of form [x,y].
    :type end: numpy.ndarray
    """
    def __init__(self, start: np.ndarray, end: np.ndarray):
        if np.shape(start)!= (2,):
            raise ValueError("Start point must have the shape (2,)")
        if np.shape(end) != (2,):
            raise ValueError("End point must have the shape (2,)")
        if (start==end).all():
            raise ValueError("Start and end points must be different")
        
        # Calculate useful properties of the line
        self.start = start
        self.line = end - start
        self.length = np.linalg.norm(self.line)
        self.unit_line = self.line / self.length
        
    def point_dist(self, point: np.ndarray):
        """Calculate the distance between a point and a line segment.
        adapted from code given by clued__init__ found here:
        https://www.py4u.net/discuss/186227

        To calculate the closest distance to a line, we calculate
        the orthogonal distance from the point to the line.

        :param point: Numpy array of form [x,y], describing the point.
        :type point: numpy.ndarray
        :return: The minimum distance to a point.
        :rtype: float
        """
        if np.shape(point)!= (2,):
            raise ValueError("Start point must have the shape (2,)")
        # compute the perpendicular distance to the theoretical infinite line
        return np.linalg.norm(np.cross(self.line, self.start - point)) /self.length
    
    def equation(self):
        """Calculate the basic linear equation parameters useful for plotting

        :return: (m, c) where m is the gradient of the line
                 and c is the y intercept
        :rtype: tuple of floats
        """
        m = self.line[1]/self.line[0]
        c = self.start[1] - m*self.start[0]
        return (m, c)

def np_polar2cart(polar_points):
    center = np.array([0,0])
    r = polar_points.T[0,]
    theta = polar_points.T[1,]
    x = r*np.cos(theta) # theta -> radians
    y = r*np.sin(theta)
    return np.array([x, y]).T    

def np_cart2polar(cart_points):
    """
    cart_points: n x 2 matrix of points in cartesian coordinates
    """
    r = np.linalg.norm(cart_points, axis=1)
    theta = np.arctan2(cart_points[:,1], cart_points[:,0])
    return np.array([r, theta]).T 
    
def process_data(range_data: np.ndarray, max_angle: float, min_angle: float, max_range: float, min_range: float, sigma: float, rf_max_pts: int, reduce_bool: bool=True):
	"""
	Filter data by removing all perceived points outside defined boundary
	"""
	
	angles = np.linspace(min_angle,max_angle,range_data.shape[0])
	processed_data = np_polar2cart(np.array([range_data,angles]).T)
	out_of_min_range = np.where(range_data<min_range)[0]
	out_of_max_range = np.where(range_data>max_range)[0]
	processed_data = np.delete(processed_data, np.concatenate([out_of_max_range,out_of_min_range]), axis=0)
	if reduce_bool:
		processed_data = reduction_filter(data_points=processed_data, sigma=sigma, rf_max_pts=rf_max_pts)    
	return processed_data

def RANSAC(points: list, dist_thresh: int, iterations: int, thresh_count: int):
	
	if len(points) == 0:
		return False 
	  
	indexes = list(range(0, len(points)))
	inliers = dict()
	for _ in range(0,iterations):
		sample_points = indexes.copy()
		random.shuffle(sample_points)
		start = sample_points.pop()
		end = sample_points.pop()

		if((start,end) not in inliers) and ((end,start) not in inliers):
			inliers[(start,end)]=[]
			line = Line(points[start],points[end])
			for point_idx in sample_points:
				if line.point_dist(points[point_idx]) < dist_thresh:
					inliers[(start,end)].append(points[point_idx])

	best_pair = None

	for cur_pair in inliers:
		if best_pair == None:
			best_pair = cur_pair
		elif len(inliers[cur_pair]) > len(inliers[best_pair]):
			best_pair = cur_pair
             
	if len(inliers[best_pair]) < thresh_count:
		return False
	best_point_1 = points[best_pair[0]]
	best_point_2 = points[best_pair[1]]        
	return best_point_1, best_point_2, inliers[best_pair]

def RANSAC_get_line_params(points: list, dist_thresh: int, iterations: int, thresh_count: int):
    
    d_m_c_endPts = [] # d: perpendicular distance, m: slope, c: constant in line equation
    while True:
        params = RANSAC(points, dist_thresh, iterations, thresh_count)
        if params:
            p1, p2, best_inliers = params
            # dtype(best_inliers): list of array of individual points
        else:
            break

        # updating points list by removing all inliers that were found
        for point in best_inliers:
            points = [x for x in points if not (x == point).all()]
        best_line = Line(p1, p2)
        m, c = best_line.equation()

        # getting starting and end points of individual lines
        best_inliers_arr = np.array(best_inliers)
        min_x, max_x = min(best_inliers_arr[:, 0]), max(best_inliers_arr[:, 0])
        min_y, max_y = min(best_inliers_arr[:, 1]), max(best_inliers_arr[:, 1])
        # each row of end_points consists of 'x' and 'y' coordinates
        if abs(min_x-max_x) >= abs(min_y-max_y):
            end_points = np.array([[min_x, m*min_x+c], [max_x, m*max_x+c]])
        else:
            end_points = np.array(
                [[(min_y-c)/m, min_y], [(max_y-c)/m, max_y]])

        center_wrt_laser = np.array([-0.45, 0]) # location of center of robot w.r.t. laser scanner
        d = best_line.point_dist(center_wrt_laser) # distance fom the center of the Robile
        d_m_c_endPts.append([d, m, c, end_points]) 
    return d_m_c_endPts

def get_occupancy_probability(laser_sub_cartesian: np.ndarray, grid_size_x: float, grid_size_y: float, grid_width: float):
    gridx = np.arange(0, grid_size_x+grid_width, grid_width)
    gridy = np.arange(-(grid_size_y/2), (grid_size_y/2)+grid_width, grid_width)
    grid, _, _ = np.histogram2d(laser_sub_cartesian[:,0], laser_sub_cartesian[:,1], bins=[gridx, gridy])
    print("np.max(grid)\n", np.max(grid))
    grid = grid/np.max(grid) 
    return grid

def get_array_of_occupied_cells(grid_cell_hits: np.ndarray, occupancy_thresh: float, grid_size_x: float, grid_size_y: float, grid_width: float):
    grid_center_x = np.arange(0+grid_width/2, grid_size_x, grid_width)
    grid_center_y = np.arange(-(grid_size_y/2)+(grid_width/2), grid_size_y/2, grid_width)
    occupied_indices = np.argwhere(grid_cell_hits>=occupancy_thresh)
    occupied_points_cart = np.array([grid_center_x[occupied_indices[:,0]], grid_center_y[occupied_indices[:,1]]]).T
    return occupied_points_cart

def online_get_line_params(points_array: np.ndarray, e=0.45, incr=0.01, max_dist=0.1, k=5):
    """
    arguments:
    points  : array of data points representing center of each occupied grid cell
    e       : allowed fraction of deviation of sum of distances between consecutive points and 
    the total length  of the line
    incr    : increment in the error values with the number of points
    max_dist: maximum distance between consecutive points allowed in a line segment
    k       : minimum number of points required in a line segment
    """

    # Convert the data points into list of ordered points
    points_polar = np_cart2polar(points_array)
    points_polar_sorted = points_polar[points_polar[:, 1].argsort()]
    points_cart = np_polar2cart(points_polar_sorted)
    points = [point for point in points_cart]

    grouped_points = []
    d_m_c_endPts = []           # line parameters: perpendicular distance, slope and deviation from laser scanner
    point = points[0]
    current_group = [point]
    aj = point                  # starting point of current line segment
    dist_sum = 0                # sum of distances between consecutive points o a line segment
    ak = point                  # latest point added to a line
    ek = e                      # incremented error

    for point_idx in range(1,len(points)): 
        dist_sum += np.linalg.norm(ak-points[point_idx])
        full_dist_ratio = np.linalg.norm(aj-points[point_idx])/dist_sum
        end_dist_ratio = 1 
        
        if len(current_group) >= k:
            prev_ak = points[point_idx-2]
            end_dist_ratio = (np.linalg.norm(prev_ak-points[point_idx])/ (np.linalg.norm(prev_ak-ak)+np.linalg.norm(ak-points[point_idx])))
        
        if(full_dist_ratio > 1-ek and end_dist_ratio > 1-e and np.linalg.norm(ak-points[point_idx]) <= max_dist):
            current_group.append(points[point_idx])
            ak = points[point_idx]
            ek += incr

        else:
            if len(current_group) >= k:
                grouped_points.append(np.array(current_group))
                
            current_group = [points[point_idx]]
            aj = points[point_idx]
            dist_sum = 0
            ak = points[point_idx]
            ek = e
            
    if len(current_group) >= k:
        grouped_points.append(np.array(current_group))

    if grouped_points:
        for points_on_line in grouped_points:
            line_segment = Line(points_on_line[0], points_on_line[-1])
            end_pts = np.array([points_on_line[0], points_on_line[-1]])
            m, c = line_segment.equation()
            center_wrt_laser = np.array([-0.45, 0])       # location of center of robot with respect to laser scanner
            d = line_segment.point_dist(center_wrt_laser) # distance fom the center of the Robile
            d_m_c_endPts.append([d, m, c, end_pts])      
    return d_m_c_endPts
     
def reduction_filter(data_points, sigma, rf_max_pts):

    '''
    Method to reduce number of laser scan points by replacing cluster of points
    with their representative.
    :param data_points: 2D array representing laser scan data in cartesian coordinates
    :param type: np.ndarray
    :param sigma: maximum distance between two consecutive points to consider them 
    in same cluster
    :param type: float    
    :param rf_max_pts: maximum number of points allowed in a cluster
    :param type: int    
    '''
    points_list = list(data_points)
    output = []
    while len(points_list) > 0:
        a_i = points_list.pop(0)
        cur_sum = np.array(a_i)
        i = 1
        while len(points_list) > 0 and abs(a_i[0] - points_list[0][0]) < sigma \
        and i <= rf_max_pts:
            cur_sum += np.array(points_list.pop(0))
            i += 1
        output.append(cur_sum/i)
    return np.array(output)    

