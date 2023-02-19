# Copyright David Jantz September 2019

# Overall goals:
# 3d physics -- make the balls bounce around, gravity, etc. Could also interact with the spheres somehow.
    # to do this I should probably learn how to use classes to make ball objects
# Stop drawing spheres and make lines or shapes connecting the points instead.
# Make a game with this 3D viewing engine where you have to dodge obstacles or something.

print("Mark 6, baby! Let's get this rolling!")
print()

import pygame, math, sys, random
from pygame.locals import *

FPS = 60

WINDOWWIDTH = 1000
WINDOWHEIGHT = 700

WHITE = (255, 255, 255)
BLACK = (  0,   0,   0)
RED =   (255,   0,   0)
GREEN = (  0, 255,   0)
BLUE =  (  0,   0, 255)

# syntactic sugar to keep cartesian axes straight.
X = 0
Y = 1
Z = 2

# syntactic sugar to keep geographic coordinates straight.
RAD = 0
LAT = 1
LON = 2

class Ball:
    def __init__(self, apparentRadius, boundaries, color, distance, elasticity, gravity, position, radius, speed, velocity):
        self.apparentRadius = apparentRadius
        self.boundaries = boundaries
        self.color = color
        self.distance = distance
        self.elasticity = elasticity
        self.gravity = gravity
        self.position = position
        self.radius = radius
        self.speed = speed
        self.velocity = velocity
    
    def calculateApparentRadius(self): # Calculates the apparent size of the ball so it scales (inversely) with distance.
        if self.distance <= 0.001: # Without this I can end up dividing by zero, which is a mathematical no-no
            self.apparentRadius = WINDOWWIDTH
        else:
            self.apparentRadius = round(self.radius / self.distance)
    
    def assignColor(self):
        red = (self.position[X] + 10) * 255/20
        green = (self.position[Y] + 10) * 255/20
        blue = (self.position[Z] + 10)* 255/20
        self.color = (red, green, blue)
    
    def setVelocity(self): # randomly assigns x, y, z direction and speed on initial iteration
        x = random.uniform(-self.speed, self.speed)
        y = random.uniform(-self.speed, self.speed)
        z = random.uniform(-self.speed, self.speed)
        self.velocity = [x, y, z]
    
    def move(self):
        self.position[X] += self.velocity[X]
        self.position[Y] += self.velocity[Y]
        self.position[Z] += self.velocity[Z]
    
    def bounce(self, axis, direction):
        bounced = False
        if direction != 1:
            direction = 0 # zero means down because the negative value of the boundaries is the zeroth element in the list.
        for i in range(3):
            #if self.position[Y] <= self.boundaries[Y][0]: 

                #bounced = True
            if self.position[i] <= self.boundaries[i][0]: # if the position on that axis is less than the minimum on that axis
                self.velocity[i] = abs(self.velocity[i]) # make the velocity positive
            elif self.position[i] >= self.boundaries[i][1]: # if the position on that axis is greater than the maximum on that axis
                self.velocity[i] = -abs(self.velocity[i]) # make the velocity negative
        
        if abs(self.position[axis]) >= abs(self.boundaries[axis][direction]): # if the ball is striking or touching the bottom plane of the container.
            # the line above is sloppy, if the ball makes it past the top of the container it gets stuck.
            bounced = True
            for i in range(3):
                if abs(self.velocity[i]) < 0.005:
                    self.velocity[i] = 0
                else:
                    self.velocity[i] *= self.elasticity
        
        return bounced
    
    def fall(self, axis, direction): # the balls fall in a particular direction
        self.velocity[axis] += self.gravity * direction

def main():
    pygame.init()
    pygame.display.set_caption('"3D Fun" by David Jantz')
    
    global SCREEN, SCREEN2, MEDIUMFONT, CLOCK
    SCREEN = pygame.display.set_mode((WINDOWWIDTH, WINDOWHEIGHT))
    SCREEN2 = SCREEN.convert_alpha()
    MEDIUMFONT = pygame.font.SysFont('pristina', 100)
    CLOCK = pygame.time.Clock()
    
    # set ball attributes (everything but position)
    apparentRadius = None
    b = 5 # Not a direct ball attribute
    m = 5 # also not a direct ball attribute
    boundaries = [[-m*b, m*b], [-m*b, m*b], [-m*b, m*b]]
    color = None
    distance = 0
    elasticity = 0.5
    gravity = 0.005
    radius = 500
    speed = 0.15
    velocity = 0.00
    
    cube3D = []
    cubeRange = range(-b, b+1)
    for i in cubeRange:
        for j in cubeRange:
            for k in cubeRange:
                if abs(i) == b or abs(j) == b or abs(k) == b:
                    position = [i, j, k]
                    ball = Ball(apparentRadius, boundaries, color, distance, elasticity, gravity, position, radius, speed, velocity)
                    ball.assignColor()
                    ball.setVelocity()
                    cube3D.append(ball) # store an arbitrary integer as the fourth value until distance from the viewer gets calculated later on.
                
    focus = [0] * 3 # The point the viewer is looking at (the center of their vision)
    cube2D = [None] * len(cube3D) # Coordinates to be used to draw to the screen.
    viewer = [None] * 3 # The viewer's location in the 3D Cartesian space.
    up = [None, None, None] # Will calculate this continually based on viewer and focus position.
    lensAngle= 1000 # can set to convert to degrees for troubleshooting. I didn't know what to call this variable... it controls the "bubbleness" of the lens like you see in security footage.
    maxViewingAngle = 45
    fieldOfView = lensAngle * maxViewingAngle * (math.pi / 180) # I think this is the right math to make the view angle max out at 45 degrees.
    # you can think of the viewer's position as being at a certain latitude / longitude on a sphere of a given radius. (Makes viewer movement more intuitive)
    geographicViewerPosition = [20, # radius: distance from the focal point in cartesian graph units. Can be anywhere from 0 to infinity.
                                math.pi/3.5, # latitude: angle between up, focus, and viewer. Can be anywhere from 0 to pi.
                                math.pi/4] # longitude: On an XZ plane at viewer's Y value, the angle between the viewer, the origin, and the X axis. Ranges from 0 to 2pi.
    geographicViewerVelocity = [0] * 3
    geographicViewerAcceleration = [0] * 3
    ballMovement = False
    
    while True:
        # event handling -- closing the program or moving the viewer's location. No controls yet for moving the focus.
        for event in pygame.event.get():
            geographicViewerAcceleration, lensAngle, ballMovement = handleEvents(event, geographicViewerVelocity, geographicViewerAcceleration, lensAngle, ballMovement)
        
        geographicViewerVelocity = changeGeographicViewerVelocity(geographicViewerPosition, geographicViewerVelocity, geographicViewerAcceleration)
        geographicViewerPosition = changeGeographicViewerPosition(geographicViewerPosition, geographicViewerVelocity)
        
        # A note about some notes later on: V, F, U, P1, and P2 are shorthand for viewer, focus, up, point observed, and the point after it's mapped onto the viewer's plane.

        viewer = convertGeographicToCartesian3D(geographicViewerPosition, focus)
        
        # prevent weird edge cases from happening due to points in the same spot
        if focus[X] == viewer[X] and focus[Y] == viewer[Y] and focus[Z] == viewer[Z]:
            viewer[X] += 0.0001
            viewer[Z] += 0.0001
        elif viewer[X] == 0 and viewer[Z] == 0:
            viewer[X] += 0.0001
            viewer[Z] += 0.0001
            
        up = calculateUp(viewer, focus)
        left, right = calculateLeftAndRight(viewer, focus)
        
        cube3D.sort(key = returnDistance, reverse = True) # sort the 3D list by distance from viewer so later on the dots get drawn to the screen in the right order.
        
        for i in range(len(cube3D)): # for every point in the cube
            ball = cube3D[i] # syntactic sugar
            if ballMovement:
                if not ball.bounce(Y, -1): # if the ball's velocity was reversed this iteration, don't do any gravity calculations.
                    ball.fall(Y, -1)
                ball.move()
                if ball.color == (127.5, 127.5, 127.5):
                    print(ball.velocity)
                #ball.assignColor()
            
            # Prevent some weird edge cases from happening (keep point, viewer, and focus from occupying the same location)
            if ball.position[X] == focus[X] and ball.position[Y] == focus[Y] and ball.position[Z] == focus[Z]:
                focus[Z] += 0.0001
            if ball.position[X] == viewer[X] and ball.position[Y] == viewer[Y] and ball.position[Z] == viewer[Z]:
                viewer[Y] += 0.0001
            
            # Pythagorean theorem in 3D to calculate sidelengths of the triangle made by V, F, and P1.
            triangleA = findTriangleSidelengths(focus, ball.position, viewer) # viewer has to be the last inputted variable.
            
            ball.distance = triangleA[1] # Add the distance between the viewer and the point as a fourth item in the list that contains the point's 3D coordinates.
            
            # With the triangle sidelengths, we can use the law of cosines to calculate the angle at vertex V. This wins half the battle!
            angleFVP1 = viewerAngleCalculator(triangleA)
            # multiplier here to get the perceived distance from focus right.
            perceivedDistanceFromFocus = angleFVP1 * lensAngle
            
            if perceivedDistanceFromFocus <= fieldOfView: # If the object is inside a reasonable field of view (45 to 90 degrees from focal point)
                
                # Put P1 on the same plane as the viewer, creating P2. This involves direction vectors and shit I understood when I was sober.
                mappedPoint = mapPointToViewerPlane(viewer, ball.position, focus, triangleA, angleFVP1)
                
                # Since P2, U, and V are all on the viewer's plane, it's easy from here:
                # Use the Pythagorean theorem in 3D to calculate sidelengths of the triangle made by V, U, and P2.
                triangleB = findTriangleSidelengths(up, mappedPoint, viewer) # viewer has to be the last inputed value.
                
                # With the triangle sidelengths, we can use the law of cosines to calculate the angle at the vertex V. This wins the other half of the battle!
                perceivedAngleFromVertical = viewerAngleCalculator(triangleB)
                
                # Figure out whether to display the point on the left or right side of the screen:
                leftOrRight = placePointLeftOrRight(ball.position, left, right)
                
                # Now we have two 2D variables that describe the point's location from the viewer's perspective.
                # They aren't Cartesian coordinates, so we convert them to X and Y values using one last bit of trig and we're ready to draw to the screen!
                point2D = convertToCartesian2D(perceivedDistanceFromFocus, perceivedAngleFromVertical, leftOrRight)
                ball.calculateApparentRadius() # determine the size of the dot to draw on the screen
                cube2D[i] = point2D
            
            else:
                cube2D[i] = None

        SCREEN.fill(WHITE)
        drawAllPoints(cube2D, cube3D)
        pygame.display.update()
        CLOCK.tick(FPS)

def handleEvents(event, velocity, acceleration, lensAngle, ballMovement): # handles all events, adjusting viewer acceleration and stuff.
    checkForTerminate(event)
    
    # Acceleration values for the three geographic coordinates
    radAccel = 0.01
    latAccel = 0.001
    lonAccel = 0.001
    
    if event.type == KEYDOWN:
        if event.key == K_i: # zoom the viewer in closer to focus.
            acceleration[RAD] = -radAccel
        elif event.key == K_o: # zoom the viewer out away from focus.
            acceleration[RAD] = radAccel
        elif event.key == K_UP: # Increase the viewer's latitude.
            acceleration[LAT] = latAccel
        elif event.key == K_DOWN: # Decrease the viewer's Y coordinate position.
            acceleration[LAT] = -latAccel
        elif event.key == K_RIGHT: # Increase the viewer's X coordinate position.
            acceleration[LON] = lonAccel
        elif event.key == K_LEFT: # Decrease the viewer's X coordinate position.
            acceleration[LON] = -lonAccel # The cartesian coordinate system I choose is chiral, which I didn't realize before...
                                    # Turns out I arbitrarily chose a right-handed coordinate system. I think this is standard, so it was a lucky guess :)
        elif event.key == K_z:
            lensAngle += 25
        elif event.key == K_x:
            lensAngle -= 25
        elif event.key == K_m:
            ballMovement = True
            
    elif event.type == KEYUP:
        if event.key == K_i and acceleration[RAD] == -radAccel: # if the user lifts the key that is currently controlling the direction of movement
            acceleration[RAD] = 0
        elif event.key == K_o and acceleration[RAD] == radAccel:
            acceleration[RAD] = 0
        elif event.key == K_UP and acceleration[LAT] == latAccel: # if the user lifts the key that is currently controlling the direction of movement
            acceleration[LAT] = 0
        elif event.key == K_DOWN and acceleration[LAT] == -latAccel:
            acceleration[LAT] = 0
        elif event.key == K_RIGHT and acceleration[LON] == lonAccel: # if the user lifts the key that is currently controlling the direction of movement
            acceleration[LON] = 0
        elif event.key == K_LEFT and acceleration[LON] == -lonAccel:
            acceleration[LON] = 0
        elif event.key == K_m:
            ballMovement = False
    
    return acceleration, lensAngle, ballMovement

def changeGeographicViewerVelocity(position, velocity, acceleration): # changes geographic velocity based acceleration of the viewer.
    radMaxSpeed = 0.3
    latMaxSpeed = 0.03
    lonMaxSpeed = 0.03
    
    radFriction = 0.01
    latFriction = 0.001
    lonFriction = 0.001
    
    velocity[RAD] = changeOneGeographicVelocityCoordinate(velocity[RAD], acceleration[RAD], radMaxSpeed, radFriction)
    velocity[LAT] = changeOneGeographicVelocityCoordinate(velocity[LAT], acceleration[LAT], latMaxSpeed, latFriction)
    velocity[LON] = changeOneGeographicVelocityCoordinate(velocity[LON], acceleration[LON], lonMaxSpeed, lonFriction)

    if position[RAD] <= 1 and velocity[RAD] < 0: # minimum radius stops viewer from getting any closer.
        velocity[RAD] = 0

    if position[LAT] <= latMaxSpeed and velocity[LAT] < 0: # minimum angle stops viewer from getting any closer to the "north pole"
        velocity[LAT] = 0
    elif position[LAT] >= math.pi - latMaxSpeed and velocity[LAT] > 0: # maximum angle stops viewer from getting any closer to the "south pole"
        velocity[LAT] = 0

    return velocity

def changeOneGeographicVelocityCoordinate(velocity, acceleration, maxSpeed, friction): # changes velocity of radius, latitude, or longitude
    if acceleration != 0: # If the user is trying to move the viewer location
        velocity += acceleration
        if velocity >= maxSpeed: # maximum positive velocity
            velocity = maxSpeed
        elif velocity <= -maxSpeed: # maximum negative velocity
            velocity = -maxSpeed
    else: # The user is not trying to move the viewer.
        almostZero = friction # setting this value to acceleration ensures we can't jump past zero while slowing down.
        if -almostZero < velocity < almostZero: # set to zero if we get close.
            velocity = 0
        elif velocity > 0: # if velocity is positive, subtract friction value
            velocity -= friction
        elif velocity < 0: # if velocity is negative, add friction value
            velocity += friction
    
    return velocity

def changeGeographicViewerPosition(position, velocity): # changes geographic coordinates based the velocity of the viewer.
    position[RAD] += velocity[RAD]
    position[LAT] += velocity[LAT]
    position[LON] += velocity[LON]
    
    # technically not necessary, but it keeps longitude values tidy and between 0 and 2pi.
    if position[LON] > math.pi * 2:
        position[LON] -= math.pi * 2
    elif position[LON] < 0:
        position[LON] += math.pi * 2
    
    return position

def calculateUp(viewer, focus): # Based on the viewer's location and the focus point's location, calculates an (x, y, z) point that is "up" for the viewer.
    if focus[Y] - 0.0001 < viewer[Y] < focus[Y] + 0.0001: # edge case where neither up or down intersect with the Y axis.
        return [viewer[X], viewer[Y] + 10, viewer[Z]]
    else:
        simpleUp = (focus[X], focus[Y] + 10, focus[Z]) # a temporary "up" point on the Y axis. shorthand is S
        
        triangleSFV = findTriangleSidelengths(simpleUp, viewer, focus) # focus has to be last inputted value since it's in the corner of the triangle we're calculating the angle in.
        sideVF = triangleSFV[1] # syntactic sugar
        angleSFV = viewerAngleCalculator(triangleSFV)
        # Below: good old "cah" part of soh cah toa to find the hypotenuse.
        # The solution happens to generalize because of how cosines work.
        # It performs a calculation based only on the angle and sidelength, not based on simpleUp directly.
        # Therefore, the effect is to simply find the distance, negative or positive, to EITHER "up" or "down" depending on which one is on the Y axis.
        yValue = sideVF / math.cos(angleSFV) # could also be called "sideUF", at least for positive values. This calculation finds the hypotenuse of right triangle UFV.

    if yValue > focus[Y]: # if the up value is actually up
        return [focus[X], focus[Y] + yValue, focus[Z]] # same x and z values as "focus" place "up" directly above focal point.
    else: # it's actually the "down" value so we need to find "up" from that.
        down = (focus[X], focus[Y] + yValue, focus[Z]) # a point on the Y axis with a negative Y value
        directionToUp = getDirectionVectors(down, viewer)
        up = (viewer[X] + directionToUp[X], viewer[Y] + directionToUp[Y], viewer[Z] + directionToUp[Z])
        return up
    
def calculateLeftAndRight(viewer, focus): # Same deal as the calculateUp function, but the goal is to be able to display points on the left side of the screen.
    # We need this function because the horizontal position of the point is defined by its angle from the "up" value but there are no negative angles, resulting
    #    in only right-side display. I can check P2's distance from the "right" and "left" points and put it on the appropriate side.
    
    # First, find slope of the line of sight from the viewer to the focal point.
    run = focus[X] - viewer[X]
    rise = focus[Z] - viewer[Z]
    
    # Find the negative reciprocal of the slope to get a perpendicular line. Not in fraction form, so it's kinda weird...
    # For some strange reason, negating the X value (run) of the new tuple always gives the direction to the "right" point and
    # negating the Z value (rise) of the new tuple gives the direction to the "left" point. Not sure why.
    leftDirection = (rise, 0, -run) # I left a zero in there as a reminder that the Y value isn't changing -- left and right are on the same ZX plane as viewer.
    rightDirection = (-rise, 0, run)
    
    left = (viewer[X] + leftDirection[X], viewer[Y], viewer[Z] + leftDirection[Z])
    right = (viewer[X] + rightDirection[X], viewer[Y], viewer[Z] + rightDirection[Z])
        
    return left, right

def getDirectionVectors(pointA, pointB): # returns 3d direction vectors from pointA to pointB.
    vectorX = pointB[X] - pointA[X]
    vectorY = pointB[Y] - pointA[Y]
    vectorZ = pointB[Z] - pointA[Z]
    return vectorX, vectorY, vectorZ

def pythagorean3D(pointA, pointB): # returns the distance between two points.
    return math.sqrt((pointA[X] - pointB[X])**2 + (pointA[Y] - pointB[Y])**2 + (pointA[Z] - pointB[Z])**2)

def findTriangleSidelengths(pointA, pointB, pointC): # Takes three 3D coordinates and uses the pythagorean theorem to calculate the distance of each side of the triangle.
    legAB = pythagorean3D(pointA, pointB)
    legBC = pythagorean3D(pointB, pointC)
    legAC = pythagorean3D(pointA, pointC)
    
    assert legAB > 0, "It's impossible to draw a triangle with a leg length of zero."
    assert legAB > 0, "It's impossible to draw a triangle with a leg length of zero.."
    assert legAB > 0, "It's impossible to draw a triangle with a leg length of zero."
    
    return legAB, legBC, legAC # Later, viewerAngleCalculator depends on the viewer being point C in this function. So watch out!

def viewerAngleCalculator(triangle): # Uses the law of cosines to calculate the angle at the viewer's vertex of the triangle.
    # Law of Cosines output hinges on the order of leg lengths in the "triangle" tuple, outputted from findTriangleSidelengths.
    acosValue = (triangle[0]**2 - triangle[1]**2 - triangle[2]**2) / (-2 * triangle[1] * triangle[2])
    # Edge case: miniscule errors in computer calculations make acosValue creep past 1 and -1 when the triangle being used has an area of zero.
    if acosValue > 1:
        viewerAngle = 0
    elif acosValue < -1:
        viewerAngle = math.pi
    else:
        viewerAngle = math.acos(acosValue) # units are radians

    return viewerAngle

def drawAllPoints(points2D, points3D): # draws all 2D points in the list to the screen.    
    for i in range(len(points2D)):
        point = points2D[i] # a little syntactic sugar
        ball = points3D[i] # some more syntactic sugar
        if point != None:
            pygame.draw.circle(SCREEN, (ball.color), (point[0], point[1]), ball.apparentRadius)
    return

def mapPointToViewerPlane(viewer, point, focus, triangle, viewerAngle):    
    # the first triangle found in main() is still useful here
    sideVP1 = triangle[1]
    sideFV = triangle[2]
    
    # cos(viewerAngle) = adjacent / hypotenuse can be rearranged to solve for adjacent side below:
    sideRV = math.cos(viewerAngle) * sideVP1 # sideVP1 is the same as the hypotenuse, and sideRV is the same as adjacent. "R" is a point that exists between F and V that
        # forms a right triangle with P1 and V so this cosine stuff works properly.
    ratio = sideRV / sideFV # the ratio to multiply by to get the right distance from P1 to P2.
    
    directionToP2 = getDirectionVectors(focus, viewer) # Not only direction but also distance. Multiplied to get proper distance below.
    distanceToP2 = (directionToP2[X] * ratio, directionToP2[Y] * ratio, directionToP2[Z] * ratio)

    P2 = [point[X] + distanceToP2[X], point[Y] + distanceToP2[Y], point[Z] + distanceToP2[Z]]

    if P2[X] == viewer[X] and P2[Y] == viewer[Y] and P2[Z] == viewer[Z]: # prevent an impossible triangle from forming due to two points in the same place.
        P2[X] += 1 # could be any number since the perceived distance from viewer is already calculated to be zero.
    
    return P2

def placePointLeftOrRight(point, left, right): # Whichever one "point" is closest to is the side of the screen it's on.
    
    leftDist, rightDist, extraDist = findTriangleSidelengths(left, point, right)
    # Determine multipliers used in convertToCartesian2D function.
    if leftDist > rightDist:
        screenSide = 1
    else:
        screenSide = -1
    return screenSide

def convertToCartesian2D(distanceFromOrigin, angleFromVertical, leftOrRight): # Converts to cartesian coordinates to draw on screen.
    # Convert the angle from vertical to angle from horizontal so it's more like the unit circle.
    theta = (math.pi / 2) - angleFromVertical
    # solve for adjacent, then opposite, legs
    screenX = math.cos(theta) * distanceFromOrigin
    screenY = math.sin(theta) * distanceFromOrigin
    # convert to the screen coordinate system
    screenX *= leftOrRight
    screenX += WINDOWWIDTH / 2
    screenY *= -1
    screenY += WINDOWHEIGHT / 2
    
    return [round(screenX), round(screenY)] # is a list so I can append a radius on later

def convertGeographicToCartesian3D(geoViewer, focus): # converts the viewer's geographic position to cartesian.
    viewerY = math.cos(geoViewer[LAT]) * geoViewer[RAD] # The latitude is the angle in the viewer's corner of the triangle. The radius is the hypotenuse
    
    hypotenuseXZ = math.sin(geoViewer[LAT]) * geoViewer[RAD] # We need this hypotenuse for unit circle calculations on the XZ plane below.
    viewerX = math.cos(geoViewer[LON]) * hypotenuseXZ # this line and the next one replicate convertToCartesian2D since it's just a 2 dimensional conversion from here on.
    viewerZ = math.sin(geoViewer[LON]) * hypotenuseXZ
    
    return[viewerX + focus[X], viewerY + focus[Y], viewerZ + focus[Z]] # Adding the focal point coordinates syncs up the viewer's movement and all the display calculations.

def returnDistance(ball):
    return ball.distance

def checkForTerminate(event): #check for any quit events, then run terminate()
    if event.type == QUIT or (event.type == KEYUP and event.key == K_ESCAPE):
        terminate()

def terminate():
    pygame.quit()
    sys.exit()
    
    
if __name__ == '__main__':
    main()
    
# this is a test edit to see what happens with Github editing
