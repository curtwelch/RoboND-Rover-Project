## Project: Search and Sample Return
### Writeup by Curt Welch <curt@kcwc.com> for Project 1
### Jun 9th, 2017

---


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup -- Yes, here it is!

I'm runing a bit behind.  I hope to catch up.  I wasn't able to start this projet until the due date last Wednesday and I've been putting in 12 hours day for the last week having great fun with this project. I've done a bit more than the project required. (that's an understatement).

### Notebook Analysis

I'm not going to describe what I did with the notebook. At this point, I honeastly don't remember. It was 100's of hours of work in the past...  I hope that's ok. I'll skip straight to the describing my code as it exists now.

#### 1. color selection of obstacles and rock samples.

Rock selection I did by writting yellow_thresh() which checks for yellow collors.  Since Yellow is (255, 255, 0), my function is almost identical to the ground selection, except for the blue value, it checks for values LESS THAN the given parameter. I spend hours twiking the values of the threshold numbers, but the last numbers I used are (90, 90, 60).  So all pixels with R and G above 90, and B below 60 are considered to be potential Rock pixels.  I've only seen one falase positive in this virtural world using this approach -- and it's only sporadic so it doesn't cause the rover to lock onto a location looking for a rock that isn't there.  This virtual world is very yellow-free except for the rocks.

Obsticals are everything that is not ground or rock.

I tried MANY different values for the ground selection thresholds. But in the end, I just went back to the suggested (160, 160, 160) values. The simulation has different colors depending on what time of day and where the sun is in the simulation and though 160 seems too dark for many test runs, some of the "darker days" in the simualtion need the lower values.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

Ok, I played with that. But it has little to do with what's in my current code.  Here's the mp4 that I was playing with..

[output/test_mapping.mp4](output/test_mapping.mp4)


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

I wrote a LOT of code and threw away 10 times as much as I kept as I explored many different algoirthms for controlling the rover.  So much fun I've found it hard to stop playing with it and move on to the rest of the course.  I'll just descibe the version I've submitted here.

My current version is genrally able to reach over 80% accuracy on map fidelity and and percent mapped.  Though the longer it runs, the more "edge" pixels get added with ups the Mapped percent and lowers the Fidelity any time one of the pixels my code considers part of the map falls outside of the reference ground truth map used for scoring.  My code is always able to make the entire area.

My code picks up the rocks, and MOST the time, will get all 6 rocks if you let it run long enough (15 minuites or so).  But there is one very hard to see rock, that I don't think my code will find on it's own, and one invisible rock (below the surface) that I've never seen my rover find on it's own.  So for those two random starting rock positions, I don't think my rover code will find them -- but it MIGHT because if the rover gets close enough, and sets off the "near object" flag, my code will stop and grab the rock.  And the code as I finsished it tonight, will search the entire map over and over, so it might find the if allowed to run for hours.  But, both those rocks are in the edge of the path in dark dirt that my rover will probably never intentionly run over, due to it being lassified as unsafe land. To be sure the rover finds those two, I would have to add behavior to make the rover run off the path at all locations to endlessly serach for the hidden rocks and to verify reahablity of evey inch of the map.  It would not be hard to code this, but it would have to be allowed to run for hours as it searhced.

Or, I could have "cheated" and hard coded those two hard to find locations into my behavior, and forced to rover to keep exploring those two spots.  But I opted not to hard code any specific map knoweldge into my solution.

The rover runs at around 4 m/s in serahing so it's much faster than how it was configured in the examples.

So lets me overview the major features of the code...

# Perception

# Image Warp Translation

I used the provided perspect_transform() transform and the provided srouce and destination reference values.  I had as the notebook suggested entered my own values from the test data, but I deided to just use the one provided in the example code.

I did however inhance the transform to adjust for the rover pitch.  I hacked the mathimatidal solution that seems to work well by testing but did not carefully verify exactly what the virtual camara distrition really was.  I only used a "close enough" engineering solutio for this.

When making the rover run at near it's full speed of 5 m/s the rover will rock back and forth on accelerations and braking that produces very large distrutions in this image to world map transation approach.  I frist addressed the problem simply by not using any data when the rover was rocking too far forward or back (Rover.pitch over 2deg or so), but this limited how fast it could map and caused large sections of the ground to not be mapped while it was runnign fast.  Adding the pitch adjustment allowed mapping to work very well even when the rover rocked.

I think the current code still refuses to update the world map from the image data if the picth is more than 4 deg or the roll is more than 2 defgrees from flat.

To improve world map data updates, I also limited which pixels were used.  Pixels near the horizon (the middle of the screen) are VERY inacurate in their estimated world map locations. Pixels near the rover are very accurate.  So I wrote a trim_cords() to filter out the more distant pixels.  For the ground pixels (which I call "sand" in the code -- but later relaised this was probbaly "snow" and not sand in the virual world), I trimmed anything more than 50 pixels away from the "rover" in the map coordiages (which translates to anything more than 5 meters out in front of the rover).  This improves my maping accuracy.

# Navagation Data

I used all the "sand" pixels to assist in driving, not the "timmed" set I used for world map updates.

The perceetion code uses the visual data to check the path ahead, and recommend a steering angle, and maximum safe speed to the decision.  So it sets Rover.safe_angle (for the steering angle), and Rover.safe_vel (for the speed) which is then used my the decision code to drive the car.

To check the path ahead, the code creates a trapozoid shaped box in front of the rover, at the angle specified.  The box is 15 pixels wide, and the length depeonds on the confirued max driving speed, but as supmitted, it's 52 pixels ahead of teh rover (5.2 meters approximately). The centerline of the box follows the recommeneded steering angle -- so for a 10 deg steering angle, it's a box slated at 10 degs.

The "good path" sand pixels are then checked to see how many fall inside this "path ahead" box. The number of pixels inside the path, are then used to reccomend a safe driving speed down that path. The maximum number to expect in the box is tracked imperically by watching for the max evert seen. Then the driving speed is set on a linear scale where a full box is max speed, and the speed ahead is zero (or negative) of less than about 20% of the max.

For every image, multiple paths ahead are checked with this "slanted" path ahead box and the estmated "safe driving speed" is calucated for each direction forward.  The current code tries 13 different fixed angles from -30 to +30.

The basic idea is to pick the path with the highest estimated "safe driving" speed.  But more compolexity I'll explain below. But the simple idea, is that the best path forward in this way is passed to the "driving" decision code.

# Map updates

My code maintains multiple maps separate from the "worldmap" used for the accuracy grading and the screen display. I choose not to modify how the worldmap worked, since it was related to the "scoring" of this project.  So I added additional maps to help my rover.

I first update the world map with every grid that "seems" like it might be bad.  BUT,  limite the "bad" pixels to a fixed distance away from tehe frover, so th wrodl map never has "bad" (red) data more than a given distane away form where the rover has been (50 m or so??).

Then, I identifhy the "good" grid locations.  I map all sand and "not sand" pixels into the 200x200 (1m each pixels) grid, I sum up how many image pixels mapped to each grid for both "good" sand pixels and "bad" dirth pixels. Then for every world grid that had more good than bad pixels, I consider that a "good" grid cell and set those matching grids on the worldmap to 1 (erasing any marking as potential bad pixels since I trust this understnding of "good grids" far more tha the bad grids.

This turst is placed in the sand in this world beause the sand is always flat. So when we "see" sand in an image, our understanding of where it is in the 2D flat world is accurate. When we see a rock, we have no valid understnding of where it is in the 2D map world (it's shadow falls behind it, so our map understanding plaes it far way from where the "warpped" image tells us it is). So we must trust sand pixels, and distrust evetything else.  So I update the "untrusted" dirt pixels first, and overrite with the good sand pixel data I trust to get a high world map accuracy.

# The Visit and Stuck Maps

For advanced naigation use, I track two other maps as well. I track every grid the rover has DRIVEN on. This is the "visit" map.  Every image update that is processed I increment the grid location the rover is currently on (Rover.pos).  This creates a ground truth map that has been verified by the fact the rover could get to it.  And it tracks how many times the rover has been seen at each grid.

The 

# Rover Vision Screen

The rover vision data shown on the screen has multiple things being displayed on it. But it includes the spread of "safe" "sand" pixels (in green), as well as the "path ahead" box in blue.  So you can watch as the code is picking different paths ahead.  There is no implicit display of obstical pixels. Anything that is not "safe" is undersood as "not safe".

The 



















# Decision





#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

