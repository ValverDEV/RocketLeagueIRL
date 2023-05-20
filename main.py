import numpy as np
#import matplotlib.pyplot as plt
import cv2
from Vector2D import Vector2D
from scipy.interpolate import CubicSpline
from scipy.signal  import medfilt
from numpy import deg2rad
import serial
from time import sleep
import sys

# Serial setup to use in Arduino
evenement=""
print("Start")
port="/dev/rfcomm0"
bluetooth=serial.Serial(port, 9600) # Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick

# Constantes 
circle_radius = 20
H = 40
# Defince goal center
goal_center = Vector2D(640, 480/2)
ball_x_offest = 10
ks = 10
kernel = np.ones((ks,ks), np.uint8)
# Define camera
cam = cv2.VideoCapture(2)

# Car modes
modes = {
    'stop': 0,
    'forward': 1,
    'turn': 2
}

def take_img():
    """Take an image from the webcam

    Return
    ---------
    img
        image taken from webcam
    """
    result, img = cam.read()
    return img


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

def draw_image(triangle_center, triangle_angle, circle_center):
    """Draw triangle and circle over white background
    
    Parameters
    ----------
    triangle_center: Vector2D
        triangle center position
    triangle_angle: number
        triangle direction in degrees
    circle_center: Vector2D
        circle center position    

    Return
    ---------
    img
        drawn image

    """

    # Define white background
    img = np.ones((480, 640, 3), dtype='uint8')*255
    # Extract circle center
    cc_x = circle_center.x
    cc_y = circle_center.y
    # Get triangle vertices from center and angle
    triangle_vertices = make_triangle_vertices(triangle_center, H, triangle_angle)
    # Draw figures
    cv2.circle(img, (cc_x, cc_y), circle_radius, (255, 0, 0), -1)
    cv2.fillPoly(img, [triangle_vertices], (0, 0, 255))
    return img

def find_shapes(img):

    """Finds triangles and circles on a white background image

    Parameters
    ----------
    img: numpy.array
        image to find the shapes
    
    Returns
    ----------
    shapes
        list of dictionaries, each shape dictionary
        has a center (Vector2D) and shape (String) attributes
    """

    # Convert image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Setting threshold of gray image
    _, threshold = cv2.threshold(gray, 66, 255, cv2.THRESH_BINARY)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
    cv2.imwrite('treshold.png', threshold)
    
    # Using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    i = 0

    # list for storing names of shapes
    shapes = []
    
    for contour in contours:
    
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue
    
        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.06 * cv2.arcLength(contour, True), True)

        cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)
        
    
        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0 and M['m00'] > 200: # Check that area is bigger than 200px
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

            shape = {'center': Vector2D(x, y)}

        else:
            shape = None
    
        # putting shape name at center of each shape
        if shape:
            if len(approx) == 3:
                shape['shape'] = 'triangle'
                cv2.putText(img, 'Triangle', (x, y),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                shape['shape'] = 'circle'
                cv2.putText(img, 'circle', (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        
        shapes.append(shape)


    cv2.imwrite('contounrs.png', img)
        
    return shapes

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

def get_angle(point1, point2):
    
    """Finds the hypothenuse angle of the right angle formed by points 1 and 2

    Parameters
    ----------
    point1: tuple, list
        (x, y) coordinates
    point2: tuple, list
        (x, y) coordinates
    
    Returns
    ---------
    angle
        in degrees
    """

    x1, y1 = point1
    x2, y2 = point2
    return np.rad2deg(np.arctan((y2 - y1)/(x2 - x1)))


def find_triangle_orientation(img):

    """Find the orientation of the car (triangle)

    Parameters
    ----------
    img: numpy.array
        image to scan

    Return
    ----------
    direction
        triangle direction in degrees 
    """

    # Find contours on grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    _, threshold = cv2.threshold(gray, 66, 255, cv2.THRESH_BINARY)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)

    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    i = 0
    triangle_vertices = None

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

        cv2.drawContours(img, [contour], 0, (0,255,0), 5)
        cv2.imwrite('angle_img.png', img)

        if approx.shape[0] == 3:
            triangle_vertices = approx.reshape(3, 2)

    # Find the base of the triangle and direction to the furthest vertice
    Lmin = np.inf
    # 0 = 1 - 2, 1 = 2 - 3, 2 = 3, 1
    shortest = 0
    excluded_points = [2, 0, 1]
    for i in range(3):
        if i == 2:
            i = -1
        x1, y1 = triangle_vertices[i]
        x2, y2 = triangle_vertices[i+1]
        L = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        if L < Lmin:
            Lmin = L
            shortest = i
            middle_point = ((x1+x2)/2, (y1+y2)/2)
    excluded_point = excluded_points[shortest] 
    direction_point = triangle_vertices[excluded_point]
    angle_direction = get_angle(middle_point, direction_point)
    return angle_direction

def get_angle_error(ref_angle, img):
    """Get angle differnce between ref_angle and triangle orientation

    Parameters
    ----------
    ref_angle: number
        angle with the spline initial direction in degrees
    img:
        image to find triangle and get its orientation
    
    Return
    ----------
    angle_error
        differente between ref_angle and orientation_angle
    """
    return int(ref_angle - find_triangle_orientation(img))


def get_error(img, spline, x_range):
    
    """Calculate the vertical position (y) error from an image

    Finds the triangle (car) on an image and gets the error,
    defined as the difference between the triangle vertical position (y)
    and the vertical position on the spline for the given
    triangle's horizontal position (x)

    Parameters
    ----------
    img: numpy.array
        image to measure error on
    traj_func: CubicSpline
        scipy.interpolate.CubicSpline object with interpolation function,
        this is the trajectory the car must follow to hit the ball
    x_range: tuple,list
        tuple containing the initial horizontal position (x) of the car 
        and the ball horizontal position
    
    Return
    ---------
    error
        y_trajectory - y_car
    """
    
    car_found = False
    shapes = find_shapes(img)
    for shape in shapes:
        if shape:
            if shape['shape'] == 'triangle':
                car_center = shape['center']
                car_found = True

    if car_found:
        if car_center.x >= x_range[0]-20 and car_center.x <= x_range[1]:
            # Car was found and is still on the trajectory limits
            return spline(car_center.x) - car_center.y
        else:
            print('Car out of trajectory')
            return None
    else:
        print('Car not found')
        return None

def send_bt(mode, value):

    """Send data through bluetooth

    Parameters
    ----------
    m_type: string
        send m for mode or d for data
    value: int
        value to send
    """
    # Sleep must will change based on the delay on the Arduino,
    # so the rate at which we send that and it receives match
    sleep(0.03)
    error_bytes = f'{modes[mode]}-{int(value)}'.encode()
    bluetooth.write(error_bytes)
    print('data sent')
    print(error_bytes)
    return

def main():

    ## Initial Scan
    car_ball_found = True
    while True:
        
        #############################
        # Generate image artificially
        #############################

        # # Add padding to field
        # pad_field_x = 100+circle_radius
        # pad_field_y = 50+circle_radius

        # # Create random location for ball in the field
        # circle_center_x = np.random.randint(pad_field_x, 640 - pad_field_x)
        # circle_center_y = np.random.randint(pad_field_y, 480 - pad_field_y) # circle_center = Vector2D(circle_center_x, circle_center_y)

        # # Define triangle (car) orientation
        # angle = np.random.randint(-90, 90)

        # # Create random location for triangle in the field
        # triangle_center_x = np.random.randint(H*2, circle_center_x - 10)
        # triangle_center_y = np.random.randint(H*2, circle_center_y - 10)

        # # Turn location into Vector class
        # triangle_center = Vector2D(triangle_center_x, triangle_center_y)
        # triangle_initial = Vector2D(triangle_center_x, triangle_center_y)

        # img = draw_image(triangle_center, angle, circle_center)

        #################################
        # End image artificial generation
        #################################

        # Take image from camera
        img = take_img()
        cv2.imwrite('camera.png', img)

        # Find shapes on image
        shapes = find_shapes(img)

        for shape in shapes:
            if shape:
                if shape['shape'] == 'triangle':
                    car_center = shape['center']
                    triangle_center = shape['center']
                    triangle_initial = shape['center']
                elif shape['shape'] == 'circle':
                    ball_center = shape['center']
                    circle_center = shape['center']
                    circle_initial = shape['center']
                else:
                    print('Ball not found')
                    car_ball_found = False

        if car_ball_found:
            print('car and ball found')
            break
        
        if input() == 'quit':
            exit()

    ## Car and ball were found, now we generate trajectory once

    # Get line fuction between ball and goal
    ball_goal_func = get_line_function(ball_center, goal_center)

    # Draw ball-goal line
    ball_goal_img = draw_trajectory(img, ball_goal_func, ball_center, goal_center, (255, 0, 0), 5)

    # Define a 3rd point for interpolation and improve the curvature of the car-ball trajectory
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
    # plt.imshow(car_traj_img)
    cv2.imwrite('trajectories.png', car_traj_img)



    ######################
    # Correctly direct car
    ######################
    
    # Calculate initial direction angle based on trajectory and initial triangle position
    ref_angle = -1*get_angle(triangle_initial.coord, (triangle_initial.x + 10, spline(triangle_initial.x)))
    orientation_error = get_angle_error(ref_angle, img)
    # print('or_err')
    # print(orientation_error)
    # send_bt('turn', orientation_error)
    # Continue adjusting until triangle is correctly oriented
    # while abs(orientation_error) > 5: # degrees
    #     img = take_img()
    #     #angle += orientation_error
    #     #img = draw_image(triangle_center, angle, circle_center)
    #     orientation_error = get_angle_error(ref_angle, img)
    #     print(orientation_error)
    #     send_bt('turn', orientation_error)
    #     # if input() == 'quit':
    #     #     exit()
        
    #     # img = draw_image(triangle_center, angle, circle_center)
    #     # oriented_img = draw_trajectory(img, spline, triangle_center, ball_center, (0, 255, 0), 5)
    #     # cv2.imwrite('oriented.png', oriented_img)
    # send_bt('stop', 0)

    ##############################
    # Move Car Until Hit
    ##############################
    x_range = (triangle_center.x - 20, circle_center.x)

    car_found = False

    while True:
        img = take_img()
        shapes = find_shapes(img)
        for shape in shapes:
            if shape:
                if shape['shape'] == 'triangle':
                    triangle_center = shape['center']      
                    car_found = True
                    break
        if car_found:
            error = get_error(img, spline, x_range)
            #print(error)
            send_bt('forward', error)
        else:
            print('car not found')
            pass
        
        
        # x_range = (triangle_center_x, circle_center_x) triangle_center.move(np.random.randint(50, 200), np.random.randint(-200, 200))
        # img = draw_image(triangle_center, angle, circle_center)
        # car_traj_img = draw_trajectory(img, spline, triangle_initial, ball_center, (0, 255, 0), 5)
        # cv2.imwrite('move_img.png', car_traj_img)
        # error = get_error(img, spline, x_range)
        # if error:
        #     error = int(error)
        #     print(error)
        #     send_error(error)
        # return


if __name__ == '__main__':
    main()
    try:
        main()
    except Exception as e:
        print('something happened')
        print(e)
        for i in range(10): 
            send_bt('stop', 0)