## Project: Search and Sample Return
### Writeup for Curt Welch <curt@kcwc.com> for Project 1
### Jun 9th, 2017

---


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup -- Yes, here it is!


### Notebook Analysis

I'm not going to describe what I did with the notebook. At this point, I honeastly don't remember. It was 100 hours of work in the past...  I hope that's ok. I'll skip straight to the describing the points about the code itself.


#### 1. color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.

Rock selection I did by writting yellow_thresh() which checks for yellow collors.  Since Yellow is (255, 255, 0), my function is almost identical to the ground selection, except for the blue value, it checks for values LESS THAN the given parameter. I spend hours twiking the values of the threshold numbers, but the last numbers I used are (90, 90, 60).  So all pixels with R and G above 90, and B below 60 are considered to be potential ROck pixels.  I've only seen one flase positive in this vitural world using this approach -- and it's spuratic so it doesn't cause the rover to lock onto a location looking for a rock that isn't there.

Obsticals are everything that is not ground or rock.

I tried MANY different values for the ground selection thresholds. But in the end, I just went back to the suggested (160, 160, 160) values. The simulation has different colors depending on what time of day and where the sun is in the simulation and though 160 seems too dark for many test runs, some of the "darker days" in the simualtion need the lower values.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

Ok, I played with that. But it has little to do with what's in my current code.  Here's the mp4 that I was playing with..


![alt text][output/test_mapping.mp4]


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

OK, so I wrote a LOT of code.  And threw away 10 times as much as I kept as I explored many different algoirthms for controlling the rover.  So much fun I've found it hard to stop playing with it and move on to the rest of the course.  I'll just descibe the version I've submitted here.

My current version is genrally able to reach over 80% accuracy on map fidelity and and percent mapped.  Though the longer it runs, the more "edge" pizels get added with ups the Mapped percent and lowers the Fidelity any time one of the pixels my code considers part of the map falls outside of the reference ground truth map used for scoring.  My code is always able to make the entire area.

My code picks up the rocks, and MOST the time, will get all 6 rocks if you let it run long enough (15 minuites or so).  But there is one very hard to see rock, that I don't think my code will find on it's own, and one invisble rock (below the surface) that I've never seen my rover find on it's own.  So for those two random starting positions, I don't think my rover code will find them -- but it MIGHT because if the rover gets close enough, and sets off the "near object" flag, mu code will stop and grab the rock.  And the code as I finsished it tnoght, will search the entire map over and over, so it might find the if alowed to run for hours.

So lets me overview the major features of the code...

# Perception

# Decision





#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  



![alt text][image3]


