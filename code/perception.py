import numpy as np
import cv2
import math
import sys

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

def rock_thresh(img, rgb_thresh=(160, 160, 75)):
    # Somewhat like color_thresh() but blue must be BELOW the value
    # for deteting yellow
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
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

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
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

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)
    ground_rgb_thresh=(198,180,160)
    rock_rgb_thresh=(100,100,100)
    threshed = color_thresh(warped, rgb_thresh=ground_rgb_thresh)
    rock = rock_thresh(warped, rgb_thresh=rock_rgb_thresh)
    wall = (1 - threshed[:,:]) * (1 - rock[:,:])
    xpix, ypix = rover_coords(threshed)
    rxpix, rypix = rover_coords(rock)
    wxpix, wypix = rover_coords(wall)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:,:,:] = Rover.img[:,:,:]

    if 1:
        threshed_img = color_thresh(Rover.img, rgb_thresh=ground_rgb_thresh)
        rock_img = rock_thresh(Rover.img, rgb_thresh=rock_rgb_thresh)
        # Whatever is not ground and rock, is wall
        wall_img = (1 - threshed_img[:,:]) * (1 -rock_img[:,:])
        image_ratio = 0.2 # Image is 20%
        color = 255 * (1.0-image_ratio)
        ## Rover.vision_image[:,:,0] += (wall_img[:,:] * (color - (image_ratio * Rover.vision_image[:,:,0]))).astype(int)
        ## Rover.vision_image[:,:,1] += (rock_img[:,:] * (color - (image_ratio * Rover.vision_image[:,:,1]))).astype(int)
        ## Rover.vision_image[:,:,2] += (threshed_img[:,:] * (color - (image_ratio * Rover.vision_image[:,:,2]))).astype(int)
        Rover.vision_image[:,:,0] = wall_img[:,:] * 255
        Rover.vision_image[:,:,1] = rock_img[:,:] * 255
        Rover.vision_image[:,:,2] = threshed_img[:,:] * 255

    if 0:
        map_img = map_image(Rover.img)
        Rover.vision_image[:,:,:] = map_img[:,:,:]

    world_size = 200
    scale = 10.0
    xpix_world, ypix_world = pix_to_world(xpix, ypix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw, world_size, scale)
    rxpix_world, rypix_world = pix_to_world(rxpix, rypix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw, world_size, scale)
    wxpix_world, wypix_world = pix_to_world(wxpix, wypix, Rover.pos[0],
                                          Rover.pos[1], Rover.yaw, world_size, scale)

    ## Note after the above mapping, there will be duplicate x,y world pixels due to mapping from
    ## from 10x10 threshold grid to 1x1 world grid

    ## OK, Simple first.  Sum up counts of pixels in each world grid for all three types
    world_cnt = np.zeros_like(Rover.worldmap[:,:,:])
    for i in range(len(wxpix_world)):
        world_cnt[wxpix_world[i]][wypix_world[i]][0] += 1 # wall
    for i in range(len(rxpix_world)):
        world_cnt[rxpix_world[i]][rypix_world[i]][1] += 1 # yellow rock
    for i in range(len(xpix_world)):
        world_cnt[xpix_world[i]][ypix_world[i]][2] += 1 # safe ground

    if 1:
        for x in range(world_size):
            for y in range(world_size):
                w = world_cnt[x][y][0] # dangerous wall
                r = world_cnt[x][y][1] # yellow rock
                g = world_cnt[x][y][2] # safe ground
                if w+g > 0:
                    if g >= w:
                        Rover.worldmap[y,x,0] = 0
                        Rover.worldmap[y,x,2] = 255
                    elif w > g:
                        Rover.worldmap[y,x,0] = 255
                        Rover.worldmap[y,x,2] = 0
                if r > 10:
                    Rover.worldmap[y,x,1] = 255

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    if 0:
        for i in range(len(xpix_world)):
            ## distance squared from rover for this pixel
            dist_rov = math.sqrt(xpix[i]**2 + ypix[i]**2)
            trust = 1.0
            d = dist_rov
            if 0:
                while d > 20:
                    trust /= 2.0
                    d -= 20
               ## print("dist {}, trust={} x{}y{}".format(dist_rov, trust, xpix[i], ypix[i]))

    ## Note after the above mapping, there will be duplicate x,y world pixels due to mapping from
    ## from 10x10 threshold grid to 1x1 world grid

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    if 0:
        for i in range(len(xpix_world)):
            ## distance squared from rover for this pixel
            dist_rov = math.sqrt(xpix[i]**2 + ypix[i]**2)
            trust = 1.0
            d = dist_rov
            if 0:
                while d > 20:
                    trust /= 2.0
                    d -= 20
               ## print("dist {}, trust={} x{}y{}".format(dist_rov, trust, xpix[i], ypix[i]))
            Rover.worldmap[ypix_world[i], xpix_world[i], 2] = min(Rover.worldmap[ypix_world[i], xpix_world[i], 0]+trust, 255.0)
        if 1:
            if dist_rov < 20:
                old = Rover.worldmap[ypix_world[i], xpix_world[i], 2] 
                Rover.worldmap[ypix_world[i], xpix_world[i], 2] += (255 - old) * 0.1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    Rover.nav_dist, Rover.nav_angles = to_polar_coords(xpix, ypix)

    return Rover
