import numpy as np
import cv2
import math
import sys
import time

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

def thin_coords(x_pixel, y_pixel, percent=0.50):
    n = 0.0
    yx = [(y, x) for (y, x) in zip(y_pixel, x_pixel) if (float(x)*float(y)) % 10.0 > percent * 10.0 ]
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
    yaw_rad = yaw * np.pi / 180.0
    # Apply a rotation
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale, truncate=True): 
    # Apply a scaling and a translation
    xpix_translated = xpix_rot/scale + xpos
    ypix_translated = ypix_rot/scale + ypos
    if truncate:
        xpix_translated = np.int_(xpix_translated)
        ypix_translated = np.int_(ypix_translated)
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale, truncate=True):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale, truncate=truncate)
    # Perform rotation, translation and clipping all at once
    if truncate:
        x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
        y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    else:
        x_pix_world = np.clip(xpix_tran, 0, world_size - 1)
        y_pix_world = np.clip(ypix_tran, 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
# Add a fudge for pitch to help accurcy
# Hard code source and destiation into function
def perspect_transform(img, Rover):
           
    # Map pitch from 0 to 360 to +- 180 keeping zero as zero

    pitch_offset = Rover.pitch
    if pitch_offset > 180:
         pitch_offset -= 360

    upshift = pitch_offset * -1.9

    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    src = np.float32([[14, 140+upshift], [301 ,140+upshift],[200, 96+upshift], [118, 96+upshift]])
    dst = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
		      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
		      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
		      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
		      ])

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

def perspect_transform_old(img, src, dst): 
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

# Skip processing of new video frames to speed up FPS

Skip_ratio = 1.0/2.0 ## Skip 1 out of 2
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
    warped = perspect_transform(Rover.img, Rover)

    sand_rgb_thresh=(198,180,160)
    sand_rgb_thresh=(190,170,160)
    # sand_rgb_thresh=(180,170,160)
    sand_rgb_thresh=(160,160,160)

    rock_rgb_thresh=(150,150,100)
    rock_rgb_thresh=(100,100,60)
    rock_rgb_thresh=(80,80,30) # tried this but saw one in test data  it couln't see
    rock_rgb_thresh=(90,90,60) # seemed to be good in test images of rocks

    sand = color_thresh(warped, rgb_thresh=sand_rgb_thresh)
    rock = yellow_thresh(warped, rgb_thresh=rock_rgb_thresh)

    wall = (1 - sand[:,:]) * (1 - rock[:,:])

    # Sand (or snow?) safe driving pixels
    sxpix, sypix = rover_coords(sand)

    # Thin the list by randomly thowing pixels away.
    # n = len(sxpix)
    # sxpix, sypix = thin_coords(sxpix, sypix, percent=.50)
    # print("Before thin sand pix are", n, "after they are", len(sxpix))

    sxpixt, sypixt = trim_coords(sxpix, sypix, 50)

    # Rock pixels
    rxpix, rypix = rover_coords(rock)

    # Wall pixels
    wxpix, wypix = rover_coords(wall)
    wxpix, wypix = trim_coords(wxpix, wypix, 80)

    #
    # Convert sand pixels to polar
    #

    Rover.nav_dists, Rover.nav_angles = to_polar_coords(sxpix, sypix)

    #
    # Calculate suggested safe heading forward as simple mean of angles
    # Convert from rads to degs as well
    #

    Rover.safe_angle = 0.0
    if len(Rover.nav_angles) > 0:
        Rover.safe_angle = np.mean(Rover.nav_angles) * 180.0 / np.pi

    #
    # Calcuate safe estimaded forward speed in the direction of safe_angle
    #
    # Pick out the pixels that are in front of us, in a limited size box and use
    # the number of pixels in this box, to judge safe speed in this direction.
    # We draw a trapazoid box that is 15 units wide (1.5 meters) becuase the frover is about 1m wide
    # and the distance
    #

    safe_zone_width = 15 # pixels (aka 1 meter per 10 pixel)
    safe_zone_depth = 20 + Rover.max_vel * 8

    xoffset = np.sin(Rover.safe_angle * np.pi / 180.0)

    fxylist  = [xy for xy in zip(sxpix, sypix) if xy[0] <= safe_zone_depth and abs(xy[1]-xoffset*xy[0]) <= safe_zone_width/2]
    fxpix = [xy[0] for xy in fxylist]
    fypix = [xy[1] for xy in fxylist]

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
    # When safe count drops to 20 pixels worth of depth, vel will drop to zero.
    # This puts us two meters away from a wall when we tell the rover to stop.

    max_vel = Rover.max_vel
    zero_per = 20.0 / safe_zone_depth
    zero_pixel_cnt_point = MaxSafePixelCnt * zero_per

    # print("Max safe pixel cnt current, max:", safe_pixel_cnt, MaxSafePixelCnt)

    v = (max_vel * (safe_pixel_cnt - zero_pixel_cnt_point)) / (MaxSafePixelCnt - zero_pixel_cnt_point)

    Rover.safe_vel = np.clip(v, 0.0, max_vel)

    #
    # ROVER VISION DISPLAY
    #

    if 1: # overlay threshold ground on camera image
        Rover.vision_image[:,:,:] = Rover.img[:,:,:]
        # Rover.vision_image[:,:,:] = np.zeros_like(Rover.img)

        # Rover.vision_image[:,:,0] = sand[:,:] * 255
        # Rover.vision_image[:,:,1] = wall[:,:] * 255
        # Rover.vision_image[:,:,2] = rock[:,:] * 255

        Rover.vision_image[160-np.int32(sxpix),160-np.int32(sypix),0] = 255
        Rover.vision_image[160-np.int32(sxpix),160-np.int32(sypix),1] = 0
        Rover.vision_image[160-np.int32(sxpix),160-np.int32(sypix),2] = 0

        Rover.vision_image[160-np.int32(fxpix),160-np.int32(fypix),2] = 255
        Rover.vision_image[160-np.int32(fxpix),160-np.int32(fypix),0] = 0

        # plot target_vel as white bar and vel as blue inside it

        l = np.clip(np.int(300.0 * Rover.target_vel / 5.0), 0, 340) + 10 
        Rover.vision_image[5:15,10:l,0:3] = 255
        l = np.clip(np.int(300.0 * Rover.vel / 5.0), 0, 340) + 10
        Rover.vision_image[7:13,10:l] = (0,0,255)

        # Plot throttle and brake
        # Brake is 0 to 1.5 on graph even though it's 0 to 10 in real life.
        # We clip becuase we want to display what the throttle PID is doing with
        # the brake

        l = np.clip(np.int(150.0 * Rover.throttle / Rover.throttle_max), 0, 150)
        Rover.vision_image[18:18+5,160:160+l] = (0, 255, 0)
        l = np.clip(np.int(150.0 * Rover.brake / 1.5), 0, 150)
        Rover.vision_image[18:18+5,160-l:160] = (255, 0, 0)


        # ROCK message and square

        flash = int(time.time()*4.0) % 4 == 0

        if Rover.saw_rock and flash:
            cv2.putText(Rover.vision_image, "ROCK", (10, 150), 
                      cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 0, 0), 1)
            
        if Rover.saw_rock:
            # Draw a yellow square where we think the rock is
            Rover.update_rock()
            x = 160 - int(Rover.rock_xpix)
            y = 160 - int(Rover.rock_ypix)
            # print("ROCK x, y = ", x, y)
            xmin = np.clip(x-4, 1, 320-2)
            ymin = np.clip(y-4, 1, 320-2)
            xmax = np.clip(x+4, 1, 320-2)
            ymax = np.clip(y+4, 1, 320-2)
            Rover.vision_image[xmin-1:xmax+1, ymin-1:ymax+1, 0:3] = 0 ## Black boundry
            if Rover.see_rock:
                Rover.vision_image[xmin:xmax, ymin:ymax, 0:2] = 255 ## Yellow
            else:
                Rover.vision_image[xmin:xmax, ymin:ymax, 1:2] = 255 ## Green (estimated location)
                

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
    # roll distorations but we do adjust for pitch.  This keeps the map more accurate.
    # The pitch algoirthm is a quick fudge, not accurately calibrated so we can accept
    # large pich errors than roll errors.

    var = 2         # deg variance for trusting image data since we don't adjust for
                    # pitch or roll

    world_size = 200
    world_scale = 10.0    # 10 warpped pixes per one world map pixel

    if (Rover.pitch < var*4 or Rover.pitch > 360.0-var*4) and \
        (Rover.roll < var or Rover.roll > 360.0-var):


        sxpix_world, sypix_world = pix_to_world(sxpixt, sypixt, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw, world_size, world_scale)
        rxpix_world, rypix_world = pix_to_world(rxpix, rypix, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw, world_size, world_scale)
        wxpix_world, wypix_world = pix_to_world(wxpix, wypix, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw, world_size, world_scale)

        ## Note after the above mapping, there will be duplicate x,y world pixels due to mapping from
        ## from 10x10 threshold grid to 1x1 world grid

        #
        # Update World map from pixel data
        #

        Rover.worldmap[wypix_world, wxpix_world, 0] = 1
        Rover.worldmap[rypix_world, rxpix_world, 1] = 1
        Rover.worldmap[sypix_world, sxpix_world, 2] = 1




    # Now the ROCKs!
    # We both spot rocks, and remember when we last saw it.  The memory
    # is needed because sometimes we see a rock, but can't stop fast enough.
    # By the time we stop, the rock is no longer in our vision.  We must try
    # to spin around in the righ direction to find it, so we need to remember
    # we saw it.
    #

    Rover.see_rock = False
    Rover.rock_pixels = len(rxpix)

    if Rover.rock_pixels > 1:  # Must be more than 1 pix to cont as rock
        Rover.see_rock = True

        dists, angles = to_polar_coords(rxpix, rypix)

        # Find the pixel with the minimum distance.  This is the most accurate measure
        # of the rock, since it will be the bottom pixel touching the ground -- all
        # ohters will "look" to be further away.  It also solves the issue of two or more rocks
        # at the same time -- we find the closest!  But if we get false postives, this could
        # send us to a noise artifiact instead of to the rock!  Doesn't seem to be an issue
        # in this fake world.

        i = np.argmin(dists)

        Rover.rock_dist = dists[i]
        Rover.rock_angle = angles[i] * 180 / np.pi
        Rover.rock_xpix = rxpix[i]
        Rover.rock_ypix = rypix[i]

        Rover.rock_xpix_world, Rover.rock_ypix_world = pix_to_world(Rover.rock_xpix, Rover.rock_ypix,
                Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, world_scale, truncate=False)

        dists = None
        angles = None

        Rover.saw_rock = True
        Rover.saw_time = time.time() # Record time we last saw a rock!
        Rover.rock_forget_time = Rover.saw_time + 4 # 4 seconds to forget

        # rock_dist and rock_angle stay valid from last time we saw a rock
        # even if we don't currently see it

    if not Rover.see_rock and Rover.saw_rock and Rover.rock_forget_time < time.time():
        # it's been many seconds and we didn't find it, forget we ever saw it.
        #print("MEMORY OF ROCK FADES, TIME IS", time.time(), " saw it at ", Rover.saw_time)
        Rover.saw_rock = False

    # Hunt for invisible rocks!

    if 0:
        # fake the smell for testing but only if the real invissarock is not in the set
        if (len(Rover.samples_pos) > 0 and Rover.samples_pos[0][0] != 70) and \
            not Rover.saw_rock and not Rover.near_sample and not Rover.picking_up:
            # Fake the smell of an invisble rock for testing invisa-rock code if we are close to the location
            if np.sqrt((Rover.pos[0]-70.0)**2 + (Rover.pos[1]-87.0)**2) < 1:
                print("FAKE ROCK SMELL")
                Rover.near_sample = True
        
    if not Rover.saw_rock and not Rover.picking_up and Rover.near_sample: # Geiger counter is buzzing like crazy!
        Rover.saw_rock = True
        Rover.rock_xpix_world = Rover.pos[0]
        Rover.rock_ypix_world = Rover.pos[1]
        Rover.saw_time = time.time() # Record time we last smelled the rock!
        Rover.rock_forget_time = Rover.saw_time + 15 # 15 seconds to turn around and get it
        Rover.worldmap[np.int_(Rover.rock_ypix_world), np.int_(Rover.rock_xpix_world)] = 1
        # The call to update_rock() will set the rest

    Rover.update_rock() # only does sommething if we are in rock_saw mode

    #
    # Status output to tty
    #

    # Rover.fps calcuated in drive_rover.py and passed here just for info`
    # if we can't keep up, things start to go to go badly.

    print("")

    if Rover.fps is not None:
        print("FPS {:3d} ".format(Rover.fps), end='')

    print("Target SV:{:6.3f} SA:{:6.3f}".format(Rover.safe_vel, Rover.safe_angle), end='')

    print(" Rocks:{}".format(Rover.rock_cnt), end='')

    if Rover.saw_rock:
        print(" ROCK", end='')
        if not Rover.see_rock:
            print(" WAS", end='')
        print(" AT {:6.2f} deg, {:6.2f} distx, {} pixels". \
                format(Rover.rock_angle, Rover.rock_dist, Rover.rock_pixels))

    print("")
    print("rocks:", Rover.samples_pos)

    #print("Rover pos", Rover.pos, " Rocks to find:", Rover.samples_pos)

    return Rover
