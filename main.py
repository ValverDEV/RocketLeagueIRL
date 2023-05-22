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
# 800x448
evenement=""
print("Start")
port="/dev/rfcomm0"
bluetooth=serial.Serial(port, 9600) # Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick

# Constantes 
circle_radius = 20
H = 40
# Camera pixels dimentions
cam_height = 488
cam_length = 800
# Defince goal center
goal_center = Vector2D(cam_length, int(cam_height/2))
# Offset to generate ball-goal line
ball_x_offest = 30
# Kernel for morphological transformation
ks = 10
kernel = np.ones((ks,ks), np.uint8)
# Lower-bound threshould for grayscale convertion
grey_th = 80
# Minimum area for shapes
shape_min_area = 400
# Define camera
cam = cv2.VideoCapture(2)
# Set camera resolution
cam.set(3, 800)
cam.set(4, 488)
# Make camera object not buffer frames
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

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
    img = cam.read()[1]
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
    _, threshold = cv2.threshold(gray, grey_th, 255, cv2.THRESH_BINARY)
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
        if M['m00'] != 0.0 and M['m00'] > shape_min_area: # Check that area is bigger than 200px
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


    #cv2.imwrite('contounrs.png', img)
        
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
    return -1*np.rad2deg(np.arctan((y2 - y1)/(x2 - x1)))


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
    _, threshold = cv2.threshold(gray, grey_th, 255, cv2.THRESH_BINARY)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)

    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    i = 0
    triangle_vertices = None


    # list for storing names of shapes
    for contour in contours:

        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.06 * cv2.arcLength(contour, True), True)

        cv2.drawContours(img, [contour], 0, (0,255,0), 5)
        #cv2.imwrite('angle_img.png', img)

        M = cv2.moments(contour)
        if M['m00'] != 0.0 and M['m00'] > shape_min_area:

            if approx.shape[0] == 3:
                triangle_vertices = approx.reshape(3, 2)

    if triangle_vertices is None:
        print('triangle not found')
        return None


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
    # Angle correction when triangle is facing backwards
    if direction_point[0] < middle_point[0]:
        if angle_direction <= 0:
            if angle_direction > -90:
                angle_direction += 180 
        elif angle_direction > 0 and angle_direction < 90:
            angle_direction -= 180
    #angle_direction *= -1
    return angle_direction

def spline_angle(x, spline, x_h = 1):
    return get_angle((x, spline(x)), (x_h, spline(x_h)))

def get_orientation_error(ref_angle, img):
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

    orientation = find_triangle_orientation(img)
    print('orientation: ', orientation)
    if not(orientation is None):
        return int(ref_angle - find_triangle_orientation(img))
    return 0 
def find_triangle_data(img):

    # Find contours on grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, grey_th, 255, cv2.THRESH_BINARY)
    threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)

    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    i = 0
    triangle_vertices = None


    # list for storing names of shapes
    for contour in contours:

        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.06 * cv2.arcLength(contour, True), True)

        cv2.drawContours(img, [contour], 0, (0,255,0), 5)
        #cv2.imwrite('angle_img.png', img)

        M = cv2.moments(contour)
        if M['m00'] != 0.0 and M['m00'] > shape_min_area:

            if approx.shape[0] == 3:
                triangle_vertices = approx.reshape(3, 2)

    if triangle_vertices is None:
        print('triangle not found')
        return None


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
    # Angle correction when triangle is facing backwards
    if direction_point[0] < middle_point[0]:
        if angle_direction <= 0:
            if angle_direction > -90:
                angle_direction += 180 
        elif angle_direction > 0 and angle_direction < 90:
            angle_direction -= 180
    #angle_direction *= -1
    return middle_point, angle_direction

def get_angle_error(triangle_base, triangle_angle, spline, x_range):

    #triangle_base, triangle_angle = find_triangle_data(img)
    if triangle_base[0] >= x_range[0]-100 and triangle_base[0] <= x_range[1]:
        spline_orientation = spline_angle(triangle_base[0], spline)
        print(f'sp: {spline_orientation}, tf: {triangle_angle}')
        return spline_orientation - triangle_angle

def get_error(car_center, spline, x_range):
    
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
    
    if car_center[0] >= x_range[0]-100 and car_center[0] <= x_range[1]:
        # Car was found and is still on the trajectory limits
        return spline(car_center[0]) - car_center[1]
    
    print('Car out of trajectory')
    return None

def create_trajectory():
    ## Initial Scan
    car_ball_found = True
    while True:
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
                else:
                    #print('Ball not found')
                    car_ball_found = False

        if car_ball_found:
            print('car and ball found')
            break
        
        print('Ball and car not found')
        
        if input() == 'quit':
            exit()

    ## Car and ball were found, now we generate trajectory once

    # Get line fuction between ball and goal
    ball_goal_func = get_line_function(ball_center, goal_center)

    # Draw ball-goal line
    ball_goal_img = draw_trajectory(img, ball_goal_func, ball_center, goal_center, (0, 0, 255), 5)

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

    x_range = (0, ball_center.x)

    return triangle_initial, ball_center, spline, x_range

def orient(triangle_initial, spline):

    img = take_img()
    ref_angle = get_angle(triangle_initial.coord, (triangle_initial.x + 30, spline(triangle_initial.x + 30)))
    orientation_error = get_orientation_error(ref_angle, img)
    print(f'ref angle: {ref_angle}')
    print('or_err')
    print(orientation_error) 
    
    while True:
        img = take_img()
        error = -1*int(get_orientation_error(ref_angle, img))
        if abs(error) < 5:
            break
        send_bt('turn', error)

    for i in range(10):
        send_bt('stop', 0)

    print(10*'#')
    print('CORRECTLY ORIENTED')
    print(10*'#')

def reorient():
    triangle_initial, ball_position, spline, x_range = create_trajectory()
    # Orient car
    orient(triangle_initial, spline)
    return spline, x_range


def follow_trajectory(spline, x_range, save_video = False):
    i = 0
    while True:
        img = take_img()
        triangle_base,  triangle_angle = find_triangle_data(img)
        spline_angle_val = spline_angle(triangle_base[0], spline)
        angle_error = -get_angle_error(triangle_base, triangle_angle, spline, x_range)
        angle_error *= k
        vertical_error = get_error(triangle_base, spline, x_range)
        if abs(vertical_error) > 100:
            spline, x_range = reorient()
        send_bt('forward', angle_error)
        if save_video:
            print(i)
            traj_img = draw_trajectory(img, spline, Vector2D(triangle_base[0], triangle_base[1]), Vector2D(x_range[1], 0), (0, 255, 0), 5)
            traj_img = cv2.putText(traj_img, f'ang:{round(triangle_angle)}, ref:{round(spline_angle_val)}, err:{round(angle_error)}', (100,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0 ), 2, cv2.LINE_AA)
            traj_img = cv2.putText(traj_img, f'vertical_error: {round(vertical_error)}', (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0 ), 2, cv2.LINE_AA)
            traj_img = cv2.putText(traj_img, f'car:{triangle_base}', (100,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0 ), 2, cv2.LINE_AA)
            traj_img = cv2.putText(traj_img, f'spline:{triangle_base[0]}, {spline(triangle_base[0])}', (100,200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0 ), 2, cv2.LINE_AA)
            traj_img = cv2.putText(traj_img, f'bt_sent: {angle_error}', (100,250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0,0 ), 2, cv2.LINE_AA)
            cv2.imwrite(f'./traj_images/traj{i}.png', traj_img)
        i += 1

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
    value = int(value)
    sleep(0.1)
    error_bytes = f'{modes[mode]}-{int(value)}'.encode()
    bluetooth.write(error_bytes)
    print('data sent')
    print(error_bytes)
    return

def main():

    # Create intial trajectory
    triangle_initial, ball_position, spline, x_range = create_trajectory()

    # Orient car
    orient(triangle_initial, spline)
    
    # Show image of trajectory
    img = take_img()
    car_traj_img = draw_trajectory(img, spline, triangle_initial, ball_position, (0, 255, 0), 5)
    cv2.imwrite('oriented.png', car_traj_img)
    for i in range(10):
        send_bt('stop', 0)

    follow_trajectory(spline, x_range, True)



if __name__ == '__main__':
    main()
    try:
        main()
    except Exception as e:
        print('something happened')
        print(e)
        for i in range(10): 
            send_bt('stop', 0)