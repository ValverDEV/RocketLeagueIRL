import numpy as np
import matplotlib.pyplot as plt
import cv2
from Vector2D import Vector2D
from scipy.interpolate import CubicSpline
from numpy import deg2rad
from send_error import send_error

def make_triangle_vertices(base_center, H, angle):

    """Creates 3 vertices for a triangle

    Parameters
    ----------
    base_center: Vector2D
        Base center position to use for triangle
    H: number
        Triangle Height
    angle: number
        Triangle pointing direction in degrees

    Return
    ----------
    numpy.array
        an array of each of the 3 triangle's vertices
    """

    # Base lenght compared to height proportion
    base_prop = 1/3
    # Half base lenght
    l = H*base_prop 
    # Convert angle to radias
    rads = deg2rad(angle)
    # Get base center coordinates
    x = base_center.x
    y = base_center.y
    # Calculate base orientation
    a = deg2rad(angle - 90)
    # Create vertices array
    pts = np.array([(x+H*np.cos(rads), y+H*np.sin(rads)), (x+l*np.cos(a), y+l*np.sin(a)), (x-l*np.cos(a), y-l*np.sin(a))], dtype=int)
    pts = pts.reshape((-1, 1, 2))
    return pts

def get_line_function(Vec1,Vec2):

    """Create a line function that crosses points in Vec1 and Vec2

    The function has the form

    y = m*x + b

    Parameters
    ----------
    Vec1: Vector2D
        First point
    Vec2: Vector2D
        Second point

    Return
    ----------
    function
        line function
    """

    x1 = Vec1.x
    x2 = Vec2.x
    y1 = Vec1.y
    y2 = Vec2.y

    m = (y2-y1)/(x2-x1)
    b = y1 - m*x1

    return lambda x: m*x + b

def draw_trajectory(img, traj_func, from_, to_, color, thicnkess = 5):

    """Draw a trajectory over an image

    Parameters
    ----------
    img: numpy.array
        image to draw on
    traj_func: function
        function that defines the trajectory, y = f(x)
    from_: Vector2D
        trajectory beginning
    to_: Vector2D
        trajectory end
    color: tuple, list
        rgb values for color, between 0 and 255
    thickness: number, optional
        drawing line thickness

    Returns
    ----------
    numpy.array
        image with trajectory

    """

    # Create copy to draw on
    traj_img = img.copy()
    # Create x range
    traj_x = np.arange(from_.x, to_.x, 1)
    # Draw a straight line from one pixel to the next
    for i in range(len(traj_x) - 1):
        from_x = traj_x[i]
        to_x = traj_x[i+1]
        from_y = int(traj_func(from_x))
        to_y = int(traj_func(to_x))
        from_coord = (from_x, from_y)
        to_coord = (to_x, to_y)
        traj_img = cv2.line(traj_img, from_coord, to_coord, color, thicnkess)

    return traj_img

def main():
    
    # Generate white background image
    img = np.ones((480, 640, 3), dtype='uint8')*255

    # Define circle radius in pixels
    circle_radius = 20

    # Add padding to field
    pad_field_x = 100+circle_radius
    pad_field_y = 50+circle_radius

    # Create random location for ball in the field
    circle_center_x = np.random.randint(pad_field_x, 640 - pad_field_x)
    circle_center_y = np.random.randint(pad_field_y, 480 - pad_field_y)

    # Define triangle (car indicator) height
    H = 40
    
    # Define triangle (car) orientation
    angle = np.random.randint(-90, 90)

    # Create random location for triangle in the field
    triangle_center_x = np.random.randint(H*2, circle_center_x - 10)
    triangle_center_y = np.random.randint(H*2, circle_center_y - 10)

    # Turn location into Vector class
    triangle_center = Vector2D(triangle_center_x, triangle_center_y)

    # Create triangle vertices
    triangle_vertices = make_triangle_vertices(triangle_center, H, angle)

    # Draw circle and triangle on image
    cv2.circle(img, (circle_center_x, circle_center_y), circle_radius, (255, 0, 0), -1)
    cv2.fillPoly(img, [triangle_vertices], (0, 0, 255))

    # Draw field limits on image
    field_lims_img = img.copy()
    lims_corner1 = (pad_field_x, pad_field_y)
    lims_corner2 = (640 - pad_field_x, 480 - pad_field_y)

    cv2.rectangle(field_lims_img, lims_corner1, lims_corner2, (0, 255, 0), 3)

    plt.imshow(field_lims_img)
    # cv2.imwrite('positions_limits.png', field_lims_img)

    #####################################################
    # The following section of code was taken from
    # https://www.geeksforgeeks.org/how-to-detect-shapes-in-images-in-python-using-opencv/
    #####################################################

    # converting image into grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    i = 0

    shapes = []
    
    # list for storing names of shapes
    for contour in contours:
    
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue
    
        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.04 * cv2.arcLength(contour, True), True)
        
        # using drawContours() function
        # cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)
    
        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

            shape = {'center': Vector2D(x, y)}
    
        # putting shape name at center of each shape
        if len(approx) == 3:
            shape['shape'] = 'triangle'
        else:
            shape['shape'] = 'circle'
        
        shapes.append(shape)

    for shape in shapes:
        if shape['shape'] == 'triangle':
            car_center = shape['center']
        if shape['shape'] == 'circle':
            ball_center = shape['center']


    #############################################
    # End of code block
    #############################################    

    # Defince goal center
    goal_center = Vector2D(640, 480/2)

    # Get line fuction between ball and goal
    ball_goal_func = get_line_function(ball_center, goal_center)

    # Draw ball-goal line
    ball_goal_img = draw_trajectory(img, ball_goal_func, ball_center, goal_center, (255, 0, 0), 5)

    # Define a 3rd point for interpolation and improve the curvature of the car-ball trajectory
    ball_x_offest = 10
    point3_x = ball_center.x - ball_x_offest
    point3_y = ball_goal_func(point3_x)
    point3 = Vector2D(point3_x, point3_y)

    # Define spline points
    spline_x = [car_center.x, point3.x, ball_center.x]
    spline_y = [car_center.y, point3.y, ball_center.y]

    # Create spline using scipy CubicSpline
    spline = CubicSpline(spline_x, spline_y)

    # Draw car-ball trajectory
    car_traj_img = draw_trajectory(ball_goal_img, spline, car_center, ball_center, (0, 255, 0), 5)
    plt.imshow(car_traj_img)
    # cv2.imwrite('trajectories.png', car_traj_img)

    return


if __name__ == '__main__':
    main()