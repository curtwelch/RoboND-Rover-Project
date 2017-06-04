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

# Convert from pixels coordinates to rover-centric coordinates
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

# Convert to radial coords in the rover space
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

# Skip frames to solve CPU problems...

Skip_ratio = 1.0/2.0 ## Skip 2 out of 3
Skip_ratio = 2.0/3.0 ## Skip 2 out of 3
Skip_cnt = 0.0
FramesSkipped = 0.0
FramesTotal = 0.0

MaxSafePixelCnt = 200 # a rough starting estimate

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):

    # We need to skip the processing of frames to keep up 

    global Skip_ratio, Skip_cnt
    global FramesSkipped
    global FramesTotal
    Skip_cnt += Skip_ratio

    FramesTotal += 1

    if Skip_cnt > 1.0:
        # Skip this frame
        Skip_cnt -= 1.0
        # print("SKIP ratio is {:4.1f}/10".format(FramesSkipped*10.0/FramesTotal))
        FramesSkipped += 1.0
        return Rover

    # Perform perception steps to update Rover object.

    # NOTE: camera image is coming to you in Rover.img

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

    # Sand pixels
    sxpix, sypix = rover_coords(sand)
    sxpixt, sypixt = trim_coords(sxpix, sypix, 50)

    # Pick out the "front" pixels in the 1m wide 2m long section in front of the rover
    # We use these to control speed and forward motion.  They are the ground right in front
    # of the rover.
    # Pixels are 10 units per meter so we trim to a 10x20 pixel area.
    # 0,0 is in front of us.  Y is _- 5 pix, x 20 is two meters in front of us (approx)

    save_zone_width = 15 # pixels (aka 1 meter per 10 pixel)
    save_zone_depth = 40

    fxylist  = [xy for xy in zip(sxpix, sypix) if xy[0] <= save_zone_depth and abs(xy[1]) <= save_zone_width/2]
    fxpix = [xy[0] for xy in fxylist]
    fypix = [xy[1] for xy in fxylist]

    # Rock pixels
    rxpix, rypix = rover_coords(rock)

    # Wall pixels
    wxpix, wypix = rover_coords(wall)
    wxpix, wypix = trim_coords(wxpix, wypix, 80)

    #
    # ROVER VISION DISPLAY
    #

    if 1: # overlay threshold ground on camara image
        #Rover.vision_image[:,:,:] = Rover.img[:,:,:]
        Rover.vision_image[:,:,:] = np.zeros_like(Rover.img)
        Rover.vision_image[:,:,0] = sand[:,:] * 255
        #Rover.vision_image[:,:,1] = wall[:,:] * 255
        Rover.vision_image[:,:,2] = rock[:,:] * 255
        Rover.vision_image[160-np.int32(fxpix),160-np.int32(fypix),2] = 255
        Rover.vision_image[160-np.int32(fxpix),160-np.int32(fypix),0] = 0

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

    #
    # Upate the world map.  Not used for navigation, but is used for the class!
    # Map rover pixel relative points to the world map grid points
    #

    # Only update if the rover is near flat -- we don't adjust for
    # pitch and roll distorations.  This keeps the map more accurate.

    var = 2         # deg variance for trusting image data since we don't adjust for
                    # pitch or roll

    if (Rover.pitch < var or Rover.pitch > 360.0-var) and \
        (Rover.roll < var or Rover.roll > 360.0-var):

        world_size = 200
        scale = 10.0    # 10 warpped pixes per one world map pixel

        sxpix_world, sypix_world = pix_to_world(sxpixt, sypixt, Rover.pos[0],
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

        Rover.worldmap[wypix_world, wxpix_world, 0] = 1
        Rover.worldmap[rypix_world, rxpix_world, 1] = 1
        Rover.worldmap[sypix_world, sxpix_world, 2] = 1

    #
    # Divide ground pixels into forward set, left set, an right set
    # Useful, and kmportant, but kills CPU
    #
    
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(sxpix, sypix) # (dists, angles)

    # Distance is in pixels not meters (10 pixels per meter)

    # Set target safe velocity forward based on path forward
    # will go to zero if mean path forward is too small

    # Base mean safe forwardd velocity on the number of pixels inside
    # the blue froward safe zone.  Typical number when all safe is 360
    # We calculate the actual safe limit by tracking the max number seen.
    # That way we can change the size of the safe zone and it works ok.

    global MaxSafePixelCnt

    safe_pixel_cnt = len(fxpix)

    MaxSafePixelCnt = max(MaxSafePixelCnt, safe_pixel_cnt)

    # When safe count drops to 25% of max, vel will be down to zero

    max_vel = Rover.max_vel
    zero_per = .50
    zero_pixel_cnt_point = MaxSafePixelCnt * zero_per

    v = (max_vel * (safe_pixel_cnt - zero_pixel_cnt_point)) / (MaxSafePixelCnt - zero_pixel_cnt_point)

    Rover.target_vel = np.clip(v, 0.0, max_vel)

    if len(Rover.nav_angles) == 0:
        Rover.target_angle = 0.0
    else:
        Rover.target_angle = np.mean(Rover.nav_angles) * 180.0 / np.pi

    #
    # Now the ROCKs!
    #

    Rover.see_rock = False
    Rover.rock_pixels = len(rxpix)
    Rover.rock_angle = 0
    Rover.rock_dist = 0

    if Rover.rock_pixels > 1:  # Must be more than 1 pix to cont as rock
        # Just take the first pixel as the angle and distance!
        # This helps if we see more than one rock at the same time
        # Assumes low odds of false positives
        Rover.see_rock = True
        Rover.rock_dist, Rover.rock_angle = to_polar_coords(rxpix[0], rypix[0])
        Rover.rock_angle = Rover.rock_angle * 180.0 / np.pi 

    #
    # Status output to tty
    #

    # Rover.fps calcuated in drive_rover.py and passed here just for info`
    # if we can't keep up, things start to go to go badly.

    if Rover.fps is not None:
        print("")
        print("FPS {:3d} ".format(Rover.fps), end='')

    print("Target V:{:6.3f}  A:{:6.3f}".format(Rover.target_vel, Rover.target_angle), end='')

    if Rover.see_rock:
        print(" -----------------------------  ROCK AT {:6.2f} deg, {:6.2f} distx, {} pixels". \
                format(Rover.rock_angle, Rover.rock_dist, Rover.rock_pixels))
    print("")

    return Rover
