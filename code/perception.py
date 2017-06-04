import numpy as np
import cv2
import math
import sys

# return the mean distances and mean angles for a list of polar (dist, angle) points
# Needed this function to deal with null lists correclty
def vector_means(v):
    if len(v) == 0:
        return 0.0, 0.0
    m = np.mean(v, axis=0)
    return m[0], m[1]

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# detect rocks by looking for yellow
def yellow_thresh(img, rgb_thresh=(100, 100, 75)):
    # Somewhat like color_thresh() but blue must be BELOW the value
    # for deteting yellow
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

# Trim the list to be in the forward light cone and limited distance
# This is used to trim out the non ground that we use for cannyon wall detection
def trim_coords(x_pixel, y_pixel, distance):
    # trim the list to only include the view cone, and also, not too far away
    # We will not trust distant pixels for mapping so we trim them here
    yx = [(y, x) for (y, x) in zip(y_pixel, x_pixel) if x < distance and abs(y) < x]
    y_pixel = np.float32([y for (y, x) in yx])
    x_pixel = np.float32([x for (y, x) in yx])
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpix_rot/scale + xpos)
    ypix_translated = np.int_(ypix_rot/scale + ypos)
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

#Functions to classify pixels as either unsave(R), rock to pick up(G), safe ground(B)
rock = (200, 200, 50)  # yellowish
ground = (200, 200, 200) # whiteish
wall = (50, 50, 50) # darkish

def pix_distance(a, b):
    s = (a[0]-b[0])**2
    s += (a[1]-b[1])**2
    s += (a[2]-b[2])**2
    return math.sqrt(s)

def pixel_classif_too_slowy(pix):
    rdist = pix_distance(pix, rock)
    gdist = pix_distance(pix, ground)
    wdist = pix_distance(pix, wall)
    m = min(rdist, gdist, wdist)

    if m == wdist:
        return (255, 0, 0) # red is wall

    if m == rdist:
        return (0, 255, 0) # red is wall

    return (0, 0, 255) # blue is safe ground

def pixel_classify(pix):
    if pix[0] > 180 and pix[1] > 170 and pix[2] > 160:
        return (0, 0, 255)      # safe ground
    if pix[0] > 180 and pix[1] > 170:
        return (0, 255, 0)      # rock
    return (255, 0, 0)          # danger

def map_image(img):
    map_img = np.zeros_like(img[:,:,:])
    for x in range(len(img)):
        for y in range(len(img[0])):
            map_img[x][y] = pixel_classify(img[x][y])
    return map_img

# Skip every other frame to solve CPU problems...
global skip_this_step
skip_this_step = True

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):

    global skip_this_step
    skip_this_step = not skip_this_step

    if skip_this_step:
        # print("NOT RUNNING perception_step()")
        return Rover

    # print("RUNNING perception_step()")

    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform

    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
		      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
		      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
		      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
		      ])
    # print("destination is:", destination)
    # destination is: [[ 155.  154.]
                     # [ 165.  154.]
                     # [ 165.  144.]
                     # [ 155.  144.]]


    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    sand_rgb_thresh=(198,180,160)
    sand_rgb_thresh=(190,170,160)

    rock_rgb_thresh=(150,150,100)
    rock_rgb_thresh=(80,80,30)
    rock_rgb_thresh=(100,100,60)

    sand = color_thresh(warped, rgb_thresh=sand_rgb_thresh)
    rock = yellow_thresh(warped, rgb_thresh=rock_rgb_thresh)

    wall = (1 - sand[:,:]) * (1 - rock[:,:])
    sxpix, sypix = rover_coords(sand)
    sxpix, sypix = trim_coords(sxpix, sypix, 100)
    rxpix, rypix = rover_coords(rock)
    wxpix, wypix = rover_coords(wall)
    wxpix, wypix = trim_coords(wxpix, wypix, 80)

    #
    # ROVER VISION DISPLAY
    #

    if 1: # overlay threshold ground on camara image
        Rover.vision_image[:,:,:] = Rover.img[:,:,:]
        Rover.vision_image[:,:,0] = sand[:,:] * 255
        #Rover.vision_image[:,:,1] = wall[:,:] * 255
        Rover.vision_image[:,:,2] = rock[:,:] * 255
        Rover.vision_image[150-np.int32(sxpix),150-np.int32(sypix),2] = 200

    if 0: # Show warped image
        Rover.vision_image[:,:,:] = warped[:,:,:]

    if 0: # color classified camara image
        #Rover.vision_image[:,:,:] = Rover.img[:,:,:]
        threshed_img = color_thresh(Rover.img, rgb_thresh=ground_rgb_thresh)
        rock_img = yellow_thresh(Rover.img, rgb_thresh=rock_rgb_thresh)
        # Whatever is not ground and rock, is wall
        #wall_img = (1 - threshed_img[:,:]) * (1 -rock_img[:,:])
        #image_ratio = 0.2 # Image is 20%
        #color = 255 * (1.0-image_ratio)
        ## Rover.vision_image[:,:,0] += (wall_img[:,:] * (color - (image_ratio * Rover.vision_image[:,:,0]))).astype(int)
        ## Rover.vision_image[:,:,1] += (rock_img[:,:] * (color - (image_ratio * Rover.vision_image[:,:,1]))).astype(int)
        ## Rover.vision_image[:,:,2] += (threshed_img[:,:] * (color - (image_ratio * Rover.vision_image[:,:,2]))).astype(int)
        #Rover.vision_image[:,:,0] = wall_img[:,:] * 255
        Rover.vision_image[:,:,1] = rock_img[:,:] * 255
        Rover.vision_image[:,:,2] = threshed_img[:,:] * 255

        if 0:
            # Overlay warrped ground map
            Rover.vision_image[:,:,0] += sand[:,:] * 0.5 * (255 - Rover.vision_image[:,:,0])
            Rover.vision_image[:,:,1] += sand[:,:] * 0.5 * (255 - Rover.vision_image[:,:,1])
            Rover.vision_image[:,:,2] += sand[:,:] * 0.5 * (255 - Rover.vision_image[:,:,2])

    if 0:
        map_img = map_image(Rover.img)
        Rover.vision_image[:,:,:] = map_img[:,:,:]

    #
    # Map rover pixel relative points to the world map grid points
    #

    world_size = 200
    scale = 10.0
    sxpix_world, sypix_world = pix_to_world(sxpix, sypix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw, world_size, scale)
    rxpix_world, rypix_world = pix_to_world(rxpix, rypix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw, world_size, scale)
    wxpix_world, wypix_world = pix_to_world(wxpix, wypix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw, world_size, scale)

    ## Note after the above mapping, there will be duplicate x,y world pixels due to mapping from
    ## from 10x10 threshold grid to 1x1 world grid

    #
    # Update World map from pixel data
    #

    if 1:
        # Only update world map when the rover is near flat
        var = 2         # deg varrance for trusting image data since we don't adjust for
                        # pitch or roll
        if (Rover.pitch < var or Rover.pitch > 360.0-var) and \
            (Rover.roll < var or Rover.roll > 360.0-var):
                Rover.worldmap[wypix_world, wxpix_world, 0] = 1
                Rover.worldmap[rypix_world, rxpix_world, 1] = 1
                Rover.worldmap[sypix_world, sxpix_world, 2] = 1

    #
    # Divide ground pixels into forward set, left set, an right set
    # Useful, and kmportant, but kills CPU
    #
    
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(sxpix, sypix) # (dists, angles)
    Rover.nav_vectors = list(zip(Rover.nav_dists, Rover.nav_angles))

    # Distance is in pixels not meters (10 pixels per meter)

    Rover.nav_l = [] # left nav vectors (angle, dist)
    Rover.nav_f = [] # forward
    Rover.nav_r = [] # right

    if 1:
        for v in Rover.nav_vectors:
            # Calculate distance from center line
            # we are one meter wide (10 pixels), so check path 1m wide forward.
            # We don't attempt to adjust for a turning path.
            dist, angle = v
            if abs(math.sin(angle) * dist) <= 5:
                Rover.nav_f.append(v)
            elif angle > 0:
                Rover.nav_l.append(v)
            else:
                Rover.nav_r.append(v)
            
    # Mean of distances and angels for all three sets

    if 1:
        Rover.nav_l_dists_mean, Rover.nav_l_angles_mean = vector_means(Rover.nav_l)
        Rover.nav_f_dists_mean, Rover.nav_f_angles_mean = vector_means(Rover.nav_f)
        Rover.nav_r_dists_mean, Rover.nav_r_angles_mean = vector_means(Rover.nav_r)

    # We can calculate this from the above insted of scannig the data
    # yet again. Just saying.
    Rover.nav_dists_mean, Rover.nav_angles_mean = vector_means(Rover.nav_vectors)

    # Set target safe velocity forward based on path forward
    # will go to zero if mean path forward is too small

    if 0:
        print("total vectors", len(Rover.nav_vectors))
        print("Path left  size={:5d} mean distance={:5.2f} mean angles={:5.2f}".format(len(Rover.nav_l), Rover.nav_l_dists_mean, Rover.nav_l_angles_mean))
        print("Path for   size={:5d} mean distance={:5.2f} mean angles={:5.2f}".format(len(Rover.nav_f), Rover.nav_f_dists_mean, Rover.nav_f_angles_mean))
        print("Path right size={:5d} mean distance={:5.2f} mean angles={:5.2f}".format(len(Rover.nav_r), Rover.nav_r_dists_mean, Rover.nav_r_angles_mean))

    mean_dists = np.clip(Rover.nav_f_dists_mean-10, 0.0, 40.0)
    # mean_dists = np.clip(Rover.nav_dists_mean-10, 0.0, 40.0)
    Rover.target_vel = Rover.max_vel*2.0 * mean_dists / 40.0


    # Now the ROCKs!

    Rover.nav_rock_dists, Rover.nav_rock_angles = to_polar_coords(rxpix, rypix) # (dists, angles)
    Rover.nav_rock_vectors = list(zip(Rover.nav_rock_dists, Rover.nav_rock_angles))

    Rover.nav_rock_dists_mean, Rover.nav_rock_angles_mean = vector_means(Rover.nav_rock_vectors)

    if Rover.fps is not None:
        print("")
        print("FPS {:3d} ".format(Rover.fps), end='')
    print("Target_vel {:6.3f}".format(Rover.target_vel), end='')
    if len(Rover.nav_rock_vectors) > 0:
        print(" ----------------------------- ",
            " ROCK AT {:6.2f} deg, {:6.2f} distx, {} pixels".format(Rover.nav_rock_angles_mean * 180.0/np.pi, Rover.nav_rock_dists_mean,
            len(Rover.nav_rock_vectors)))
    print("")

    # print("angle vectors", Rover.nav_rock_angles)
    # print("distance vectors", Rover.nav_rock_dists)

    return Rover
