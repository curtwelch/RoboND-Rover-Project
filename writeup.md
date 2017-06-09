## Project: Search and Sample Return
### Writeup by Curt Welch <curt@kcwc.com> for Project 1
### June 9th, 2017

### Jerky -- what I call my rover

---


## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points 

---
### Rubric 1: Writeup / README

#### My Writeup -- Yes, here it is!

I'm running a bit behind.  I will catch up.  I wasn't able to start this project until the due date last Wednesday and I've been putting in 12 hours days for the last week having great fun with this project. I've done a bit more than the project required but I found it hard to stop the cycle of endless improvements.

My submitted project can be found in the project_1 branch of my GIT repository (not the master branch).

https://github.com/curtwelch/RoboND-Rover-Project/tree/project_1

### Rubric 2: Notebook Analysis

I don't remember much of what I did in the notebook. I hope that's ok. I'll focus on describing my code as it exists now.

I did use the notebook for various phases of testing and my heavily modified notebook is in the repository to look at but it's not clean or documented what I was experimenting with.

My notebook is here:

https://github.com/curtwelch/RoboND-Rover-Project/blob/project_1/code/Rover_Project_Test_Notebook.ipynb

#### Rubric 2a: Color selection of obstacles and rock samples.

Rock selection I did by writing yellow_thresh() which checks for yellow colors.  Since Yellow is (255, 255, 0), my function is almost identical to the ground selection, except for the blue value, it checks for values LESS THAN the given B parameter. I spent hours tweaking the values of the threshold numbers, but the last numbers I used are (90, 90, 60).  So all pixels with R and G above 90, and B below 60 are considered to be Rock pixels.  I've only seen one false positive in this virtual world using this approach -- and it's only sporadic so it doesn't cause the rover to lock onto a location looking for a rock that isn't there. This virtual world is very yellow-free except for the rocks which makes detection very easy.

Obstacles are everything that is not ground or rock.

I tried MANY different values for the ground selection thresholds. But in the end, I just went back to the suggested (160, 160, 160) values. The simulation has different colors depending on what time of day and where the sun is in the simulation and though 160 seems too dark for many test runs, some of the "darker days" in the simulation need the lower values. In fact, if  I were to go back and experiment more, I'd likely set it even lower now due to my rover's ability to cope well with driving into dangerous areas and to remember to avoid bad places in the future.

#### Rubric 2b: Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles, and rock samples into a world map.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

Yes, I did all that, and then a lot more in my actual code. Here's the last mp4 that I created in my experimenting. I think.  It was what I found left in my output directory.

https://github.com/curtwelch/RoboND-Rover-Project/blob/project_1/output/test_mapping.mp4


### Rubric 3: Autonomous Navigation and Mapping

#### Rubric 3a: Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the write-up of how and why these functions were modified as they were.

I wrote a LOT of code and threw away 10 times as much as I kept as I explored many different algorithms for controlling the rover.  It's been so much fun I've found it hard to stop playing with it and move on to the rest of the course.  I'll just describe the version I've submitted here and not the attempts I ended up throwing away.

My current version is generally able to reach over 80% accuracy on map fidelity and percent mapped.  Though the longer it runs, the more "edge" pixels get added which ups the Mapped Percent and lowers the Fidelity any time one of the pixels my code considers part of the map falls outside of the reference ground truth map used for scoring.  My code is always able to map the entire ground area.

My code picks up the rocks, and MOST the time will get all 6 rocks if you let it run long enough (10 to 15 minutes or so).  But there is one very hard to see gold rock that I don't think my code will find on its own, and one invisible rock (below the surface) that I've never seen my Rover find on its own.  So for those two random starting rock positions, I don't think my rover code will find them -- but it MIGHT because if the rover gets close enough, and sets off the "near object" flag, my rover will stop and grab the rock.   It doesn't need to see the rock in the visual data to grab it. The code as I finished it tonight, will search the entire map over and over, so it might find the two hard rocks if allowed to run for hours, or days. But, both those rocks are on the edge of the path in some dark dirt that my rover will probably never intentionally run over, due to it being classified as unsafe land visually. To be sure the rover finds those two, I would have to add behavior to make the Rover run off the path at all locations to endlessly search for the hidden rocks and to verify reachability of every inch of the map.  It would not be hard to code this, but it would have to be allowed to run for hours as it searched.

I could have "cheated" and hardcoded those two hard to find locations into my rover behavior, and forced the rover to keep exploring those two spots.  But I opted not to hardcode any specific map knowledge into my solution.

So lets me overview the major features of the code...

#### Perception

##### Image Warp Translation

I used the provided perspect_transform() transform and the provided source and destination reference values.  I had, as the notebook suggested, entered my own values from the test data, but I decided to just use the one provided in the example code.

I did however enhance the transform to adjust for the rover pitch.  I hacked the mathematical solution that seems to work well by testing but did not carefully verify exactly what the virtual camera distortion really was.  I only used a "close enough" engineering solution for this.

When making the rover run at 4 m/s the rover will rock back and forth on accelerations and braking that produces very large errors in the mapping.  I first addressed the problem simply by not using any data when the rover was rocking too far forward or back (Rover.pitch over 2deg or so), but this limited how fast it could map and caused large sections of the ground to not be mapped while it was running fast.  Adding the pitch adjustment allowed mapping to work very well even when the rover rocked.

The current code still refuses to update the world map from the image data if the pitch is more than 4 deg off flat or the roll is more than 2 degrees from flat.  This allows it to map most the time while it's moving.

To improve world map data updates, I also limited which pixels were used.  Pixels near the horizon (the middle of the screen) are VERY inaccurate in their estimated world map locations. Pixels near the rover are very accurate.  So I wrote a trim_cords() to filter out the more distant pixels.  For the ground pixels (which I call "sand" in the code -- but later relaised this was probbaly "snow" and not sand in the virual world), I trimmed anything more than 50 pixels away from the "rover" in the map coordinates (which translates to anything more than 5 meters out in front of the rover).  This improves my maping accuracy.

##### Navagation Data

I used all the "sand" pixels to assist in driving, not the "trimmed" set I used for world map updates.

The perception code uses the visual data to check the path ahead, and recommend a steering angle, and maximum safe speed to the decision.  So it sets Rover.safe_angle (for the steering angle), and Rover.safe_vel (for the speed) which is then used by the decision code to drive the car.

To check the path ahead, the code creates a trapezoid shaped box in front of the rover, at the angle specified.  The box is 15 pixels wide, and the length depeonds on the confirued max driving speed, but as submitted, it's 52 pixels ahead of teh rover (5.2 meters approximately). The centerline of the box follows the recommeneded steering angle -- so for a 10 deg steering angle, it's a box slated at 10 degs.

The rover is about 1m wide, so my path is looking at approximately 1/4 m on either side.  In theory.

The "good path" sand pixels are then checked to see how many pixels fall inside this "path ahead" box. The number of pixels inside the path, are then used to reccomend a safe driving speed down that path. The maximum number to expect in the box is tracked imperically by watching for the max ever seen (due to the warp mapping, and average ground color, the expected number of pixels is actually very. hard to calculate). Then the driving speed is set on a linear scale where a full box is max speed, and the speed ahead is zero (or negative) of less than about 20% of the max.

For every image, multiple paths ahead are checked with this "slanted" path ahead box and the estimated "safe driving speed" is calculated for each direction forward.  The current code tries 13 different fixed angles from -30 to +30.

The basic idea is to pick the path with the highest estimated "safe driving" speed.  But also includes more complexity I'll explain below. But the simple idea is that the best path forward found in this way is passed to the "driving" decision code.

##### Map updates

My code maintains multiple maps separate from the "world map" used for the Fidelity grading and the screen display. I choose not to modify how the supplied world map worked, since it was related to the "scoring" of this project.  So I added additional maps to help my rover.

I first update the world map with every grid that "seems" like it might be bad.  But,  limit the "bad" pixels to a fixed distance away from the rover, so the world map never has "bad" (red) data more than a given distance away form where the rover has been (50 m or so??).

Then, I identify the "good" grid locations.  I map all sand and "not sand" pixels into the 200x200 (1m each pixels) grid, I sum up how many image pixels mapped to each grid for both "good" sand pixels and "bad" dirth pixels. Then for every world grid that had more good than bad pixels, I consider that a "good" grid cell and set those matching grids on the worldmap to 1 (erasing any marking as potential bad pixels since I trust this understnding of "good grids" far more tha the bad grids.

This trust is placed in the sand in this world beause the sand is always flat. So when we "see" sand in an image, our understanding of where it is in the 2D flat world is accurate. When we see a rock, we have no valid understanding of where it is in the 2D map world (it's shadow falls behind it, so our map understanding places it far way from where the "warpped" image tells us it is). So we must trust sand pixels, and distrust evetything else.  So I update the "untrusted" dirt pixels first, and overrite with the good sand pixel data I trust to get a high world map accuracy.

##### The Visit and Stuck Maps

For advanced navigation use, I track two other maps as well. I track every grid the rover has DRIVEN on. This is the "visit" map.  Every image update that is processed I increment the grid location the rover is currently on (Rover.pos).  This creates a ground truth map that has been verified by the fact the rover could get to it.  And it tracks how many times the rover has been seen at each grid.

In the same sense, I also track a "stuck" map. The rover has the ability to detect when it's stuck, and perform random violent maneuvers until it gets itself unstuck. This world seems like there is no place the rover can't escape from so getting stuck seems harmless in the end.  Every time this happens, the stuck map grid is incremented so the rover learns what places to try and avoid in the future.

As currently configured, my software is printing a small section of the Visit and Stuck maps to the TTY output as it runs. It prints an 11x11 section of the large 200x200 map around the current location of the rover. Negative numbers are from the "stuck" map and positive numbers from the Vist map.  Here's some output as it was stuck on that low rock in the middle of the map (the -8).  Note that multiple grids in that rock area has been marked as "do not vist" (negative numbers).

```
Visit count MAP (near rover) used to force explore
  11  63  -4 136  28  37  36  43  66  32  51
  87  38   3  25  29  36  60  46  36  33  44
   7   .   1  13  31  47  51  51  32  30  44
   3   .  -1  32  33  41  54  45  30  46  38
  17   9  24  75  30  34  33  30  29  30  41
  33  43  47  33 127  -8  35  23  40  29  58
  26  37  30  29 -23  -1  69  38  41  30  55
  34  32  54  58   .   .  16  51  31  42  57
  41  20  17  -1   .   3  14  28  51  52  47
  84  62  57  22  38  17   6  -1  39  82  46
  15  27  80  65  49  34  56  -3  38  90  54
```

##### Collision Detection

The code detects collisions as high values of negative acceleration without the brake on.  This allows it to understand when it runs into a rock.  It will display "OUCH" on the screen when this happens, and increment the "stuck" map, to mark the fact it should avoid that grid from now on.  We see in the data bove, the rover has marked 8 grid location in "rock valley" to avoid.  Even whem marked to avoid them, it still at times will hit them.

##### Path Selection

In addition to looking for the best clear path forward, the path selection logic is also free to make higher level decisions when it sees multiple good paths forward.  It does two things with these higher level choices to help the rover reach its goal of mapping the whole world and finding all the rocks.

The first, is that when given multiple "good" paths to pick from, it picks the left most path over the rest.  Simple, but effective.  This creates a "follow the left wall" navigation approach by doing this.  Because this simple puzzle world has no loops, this sikmple appraoch helps force it to visit the etire map. And beause the gold it is looking for, is always hidden on the edges of the map. seaching the edges is not a bad approach.  So when the program first starts, you will notice a very strong follow the left wall behavior effect.

But, that alone is not enough, because the rover can tend to get stuck in long loops around the world that don't cover all the ground.  After it marks various grid locations as "avoid" in the stuck map, it becomes even easier for it to never drive though the rocks and cover all the ground.

So I use the visit map to track where it's driven, and when given multiple options as "good" to choose from, it also checks to see which path forward covers ground that is LESS visited.  It always picks the path that less visited. So the more the rover gets stuck in one loop, the higher the visit counts become, and that forces it to exit the loop and go explore a less visited part of the world.  This keeps the rover exploring the entire map over and over.

So when it starts out, it will follow the edge using the left hand rule. But with each lap you will notice "lawn mowing" like behavior where it tends to get further away from the wall each time and wander out into the middle of the larger plains.  In time, this adds random path behavior as well as guarantees that the rover is driving everywhere it thinks is safe.

The other reason for using the "left haned" rule is sort of cheating. That one rock, is hidden behidn to large bolders. And if you come across it by following the left hand wall, the rover will drive up behind the bolders and grap it. if the rover instead first spots that rock from the other side, between the two rocks, the rover will drive straight to it, and get suck and not reach it. The robver will unstick istelf, and move on, but if it were doing a right hand rule, it will just keep coming apon the rock from the wrong direction and maybe never get it.  Using the left hand rule makes it more likely the rover will quickly find, and nab, that hard to get rock -- without having to write any path planing code for the rover when it's picking up rocks.

##### Rover Vision Screen

![Vision Screen Exmaple](https://github.com/curtwelch/RoboND-Rover-Project/blob/project_1/IMG/Jerky.png)

The rover vision data shown on the screen has multiple things being displayed on it. But it includes the spread of "safe" "sand" pixels (in green), as well as the "path ahead" box in blue.  So you can watch as the code is picking different paths ahead.  There is no implicit display of obstacle pixels. Anything that is not "safe" is understood as "not safe".

When grid locations get marked as "stuck" (aka bad), you can see that in the green "sand" pixels. It shows up as an empty square in the fan display that the rover knows to try and avoid.

The background of the display is a copy of the rover image.  Having it overlaid helped me see the relation between the two easier.

When a rock is spotted, the screen flashes "ROCK", and the location on the sand pixel map (wrapped image) is shown. Not it's location on the background raw image.

![Rock Pickup](https://github.com/curtwelch/RoboND-Rover-Project/blob/project_1/IMG/RockPickup.png)

When the rover is stuck, it will display "ESCAPE" on the screen as it alternates between some random escape maneuver, and trying to drive out -- which returns to the "stuck" mode if driving isn't working.

At the top are a few bar graphs.  The wider white bar graph is the "Rover.safe_vel" value calculated by the percpetion path selection code that tells the driving code how fast it is safe to go.  The smaller blue bar inside the "safe speed" bar is the actual velocty of the rover.  This makes it easy to see what the rover "belives" is safe to drive, and how good it following that speed.

Below that is a green bar from the middle to the right. That's the throttle setting.  And a red bar, from the middle to the late. That's the brake value.  It's a simple visual indication of what the rover is doing with the throttle and brake.

The angle of the blue trapezoid box of the "best path" code, is basically what the steering is set to.  Since the there is no mehanical delay in setting the steering angle (as there would be in a real rover), there is no control problem here. The control softare can instantly set the setting to anything it wants and it does. This creates a sickening behavior to watch howevert as it jerks the wheel all over the place randomly -- very unnatural like, but perfectly valid for this simulation.

In fact, however, I set the starting to HALF of the path ahead value.  Setting the same value creates an oversteering and oscillation effect.  Setting it to half solves that.

##### Rocks

The code will remember where it saw a rock for something like 4 seconds.  So if it only gets a quick glimpse, it will stay on the screen for 4 seconds, and the rover will try to go to that location.  This could ccreate issue is the world had lots of flse postives, but sine the world has almst none, we can trust that when we see yellow, it's a rock, and we can remember where se sw it, and try to go get it.

When the code sees a rock, it trnslates to a world location and remembers that. As the rover moves, and spins, the rocks location relative to the rover, is reverse calcuated from the world map location so the driving code always has a heading and distnce vector for where it "belivbes' the rock is even when it's lost sight of it. But after 4 aseconds, if it hasn't found it and grabbed it, it will "forget" it ever saw a rock, and move on with it's seachig of the world (it wil come back and find it if it's a real rock).

Seeing multiple rocks at the same time can cause problems (endles loop of swtiching between the two). And a few of the configurations of the rocks were obvoiusly set up to test just that problem by placing rocks on either side of the path.  My code deals with that by picking the "nearest" rock pixel it sees, to focus on, and by not changing it's focus, for this 4 second "memory" span, even if the first rock is lost site of, and a second rock shows up.  This is a simple solution to the more complex prob lem of tryign to tack the lcoation of multiple rocks. My code makes no attempt to track multiple rocks at the same time. It tracks only one.

But with this 4-second focus window, the rover will likely get close to the rock it locked onto, and not be tempted to go after the other, once the 4-second window is up, and it must pick the "closest" rock again. In general, this seemed to solve the problem. Though the rover might bounce back and forth between two or three rocks for a bit, it will end up approaching one, and grab it, and not get stuck in an endless focus loop.

I have no code to catch such a loop, so if it did get stuck in such a loop looking at two rocks, my rover would not free itself from that loop.

#### Decision Code

All the above description was basically perception logic.  The perception code identifies the best paths, and speed, and where it belives rocks are.  But in its high-level choice of paths, it's creating the high-level decision behavior that guides the rover in its travel through the environment.

There is no path planning at all in my code for picking up rocks or exploring. It all works with simple logic of staying on the clear ground and trying to drive where you haven't driven before.

The decision code in decision.py implements low-level heuristics for driving.

Its implemented with 4 modes -- 'stop', 'forward', 'spin', and 'stuck'.  Rock hunting and grabbing happen as special-case code in all the states, it's not separate states.

The basic idea is to keep driving forward at the recommended speed and angle from perception.  But when a rock is spotted, the decision code slows the rover, and turns to drive towards it. If it's too far off angle, the forward code just bumps to the stop mode.  Same is true when there is no path forward, -- just switch to stop mode.  In stop mode, it will pick up a rock if there's one to pick up, if there's a way forward, it goes back to forward drving node, if not, it will try to swith to spin to find a better path forward.

Spin looks for a rock, or a good path forward, and takes it when it finds it. If it finds a rock, it goes to stop to grab it.

Forward and spin states both have logic to detect when they are stuck and switch to the stuck state. The stuck state, is not smart at all. It just tries a random move with random high throttle (up to 50 I think) that can be a spin, or forward or reverse move, in a random direction, of random steering, for a random time. Then it tries to use forward or spin to drive away. If it's still stuck, they will detect it, and return to "stuck" state to try something else random. it's not a fast escape, but it always seems to get itself free in this simple (and mostly safe) virtual world.

##### PID for throttle and brake

I went through 3 different rewrites of the throttle and break controls and ended up with a straight forward PID to control both to regulate speed to the value set by the reset of the code.

I tried to use negative throttle to regulate speed, but there's a bug (feature?) in the simulator that makes that not work well. When driving fast (over 3 m/s), even a small amount of negative throttle causes a physics warp to drop the speed down to below 2 m/s instantly. Not useful when trying to regulate the speed to 2.9 m/s and it keeps jumping down to 2, but takes a god bit of time to accelerate back up to 3.  It causes odd pumping and jerking effect that I didn't like.

Using break to regualate speed however works ok on the simulator. So the PID sets a throttle value that can go negative, but I don't send the negative throttle to the simuilatior, I interpet it to be a brake signal. So when the PID sets the throttle to -0.3, I make that apply a break of .3, and throttle of 0. When the PID throttle is postive, I take the brake off, and use that as the throttle.  This seems to have worked fine and produces resonably smooth and fast speed control and regulation.

There was an issue with the I term of the PID however. The I terms maintains a sum of the error (integral), which offsets median error of the P and D terms to reduce it.  However, this means that as the rover fails to acceleate up to the deisred speed, and spends many sedonds under speed, the PID will compensate for this "error" by driving OVER the set speed for long periods.  This is not normally seen as bad, beuase it reduces average total error.  But for this applciatioon, there was a seroius problem. when it got stuck on a rock, and the accellerato was tryig to get the rover up to speed, but the rover was going no where for a miutes at a time, this "throttle error" was "learned" by the PID controller, (high accumulated error sum), and it would then cuase it try and drive extra fast to "make up for lost time" (so to say).  The net rsult, is that the even with the target velocity for the PID set at 4 m/s the PID would floor it, and keep the rover running at 5 m/s until it ran into something else! And the more thins it ran into, the faster the PID would try to make the ROVER go when it wasn't stuck.  Not the result we needed.

One solution, was to just not use the I term (set it to zero, and must make it a PD controller).  But then it would constantly run under speed.  That could work, and I could just up the maximum speed configued in the system to compensate to get it running at the speed I wanted.  But I took a different path, and simply made it update the sum for the PID only when it was goihg over something like 2 m/s.  So only data from fast speeds were being used to adjust the I term (when I knew the rover and throttle were opperating consistantly so the PID could learn to best control the speed under the "good drivig" conditions.

##### Rock Grabbing

The logic is simple for this. Drive straight to the rock, and try to pick it up.  There is no path planning, no map use, and no use made of the perception system's "best path" information. The rover just turns to the rock and drives straight towards it, even if it's on the other side of a pile of rocks.  This means it will sometimes get stuck and not reach the rock. But the "stuck" mode will always free it.  The side effect of the random stuck mode behavior, is that either it makes the rover forget about the rock, or it sometimes puts the rover in the perfect place to pick it up, all by accident.

This works only because this world is "safe" and no "show-stopping" danger exists.  But for this environment, it seems to be all the logic that is needed to solve this problem.

#### Rubric 3b: Run the code, results, and future improvements

#### Results

Results were a Fidelity and Percent Mapped that quickly reaches about 90/90.  But the longer it runs, the map complete percent will advance to over 98% while the Fidelity falls to about 75% (after 12 hours or so at this point).  This is due to my mapping code adding edge pixels that are not on the ground truth reference map used for scoring. It's all about the edge pixels.

For most random starting positions, my rover will find and pick up all 6 rocks in about 10 to 15 minutes.

#### Future improvements

I have kept improving the code to the point that it mostly solves this challenge so there's not a lot that "needs" improving. But yet, I could easily spend anther 6 months tweaking and exploring alternative ideas.

The mapping is fairly accurate. The score is just an issue of what the ground truth considers an edge pixel so though I could work to make it match the ground truth scoring map closer, there is not much point to that -- it would not improve the basic map and retrieve mission of the "game".

There is much room to clean up my code. It's a mess from a heavy week of constant rewriting and hacking to make it all work.  Performance could easilly be improved with more optimisig. I had it running as fast as 45 FPS but the current code is a bit clareless in it's CPU use and has fallen to 15 FPS range.

The low-level driving code is tuned very nicely at least on my computer. Though I could try to push it to work well at slightly faster speeds (mine drives at around 4 m/s now, but the simulator allows up to 5). The PID needs to be able to go over the target value as it operates so you can't run the PID at 5 m/s or it will mess up the I SUM value.  So it must use a target value a bit below 5.

The current algorithm jerks the steering all over the place as it very intentionally tries to hit every low value map square.  Due to the fact the simulator works fine jerking the steering around, I didn't change this.  In a real life rover, such erratic steering would be dangerous and harmful to the robot, not to mention wasting energy (drain batteries etc). In the simulator, these real life issues are not important, but it does make me nauseous to watch the screen for too long as it jerks around, so I _could_ add simple damping to the logic to make it act more like a smooth real-life robot. That would only be an esthetic change however.

The current code is producing a highly accurate ground truth map in the Visits map of everywhere the rover drives but it's not making any sue of that map to help navigate.  After the Visits map gets filled in, it could stop using the visual data altogether (since the world is static and doesn't change), and switch to a mode of exploring all the edges.  That would fill out the truth of where the rover is acutally able to drive, as well as force the finding of thsoe two very hard to find rocks.

The Visits map could be used to perform route planing for picking up rocks as well -- so it would know to drive around an obstacle instead of through them.

There is plenty of room for upgrading the vision understnding of the envioment. This aproacoh uses a cheap trick to map pixels to a groud map that only works for flat land -- something that happens to be true in this game. But the overhangig rocks in the middle of the map that allow the low to ground camara to see under, but not drive under, tricks this approach. Turning pixel data into a better 3D understeanding would be useful and something to be explored.

In general, this problem is easy, because the simulator eliminates the need to solve the localization problem. It tells you exactly where you are, and which direction you are pointed.  This makes mapping and navigation easy. But we could choose to not use that "free" data, and try to solve the problem without it.  That becomes much harder because it requires the rover to try and solve both the mapping and localization problems.  But it would be something to explore in this project.

There is the endless number of fun ideas to play with here, but not much else "needed" since this my current code mostly solves the problem as given. It would be more of trying to explore other ways to solve it than adding something that was obviously missing.

#### Running the simulator code

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

My code seems to run fine in all resolution modes. I tend to run in the 840x524 (third from bottom) with "fantastic" graphics just so it doesn't fill my whole screen as I code.

The FPS is running about 15 (slow but works fine).  I could speed that up by cleaning up the code and throwing away wasted processing that's not needed and optimizing what is.  Reaction times would become a problem if it ran much slower on a different machine. I'm running on a new MacBook Pro and haven't tested on any other hardware.  Running it as slow as 5 FPS might well still work.  At worse it could cause the rover to crash a lot more (slow reaction times), which really isn't a problem other than slowing down how fast it will map the environment and find rocks.

My code does learn to avoid bad areas. I did not pre-populate the stuck map with learned data so each time you re-start the simulator it will run into rocks a lot. But then over time, it learns to mostly avoid the dangerous rocks.  There is no hard coded map knowledge in my code.  It should work no a different game with similar ground and rocks and walls, but a totally different map.  Loops in the topology would not be a problem for my code.

**WARNING** -- Using manual override for too long can make the rover belive it's stuck.  It will mark the location on the map as bad and avoid it in the future, even though the only "bad" thing is that a "human giant" decided to grab it and keep it from moving.  It will also go into escape mode and act confused and scarred and psychotic. This is all normal. It's not broken. :)

#### Udacity Bugs

##### Drunk Driving (fixed in Udacity repo now)

Drunk Driving -- There was a bug in the drive_rover.sp from Udacity that I fixed in my version of the code and submitted a pull request for to the Udacity repo.  When picking up rocks, the Udacity code would send both a pickup request to the simulator, AND the normal control commands. This caused the simulator to send back two new telemetry packets, not just one.  So it caused the rover code to have a cached frame and be a frame behind in the processong. But it was worse than that, beause the code send the command first. So the first telemtry data back, did not include a "pickin up" mode flag. So the rover code would send a second request to pick up.  Putting it two frames behind the simulator.

So for each rock picked up, the rover would fall 2 frames behind the simulator. This created a control delay problem where the response time of the rover got worse and worse for each rock it picked up. It's as if the rover was getting "drunk" from the rocks!  Funny thing. But it drove me CRAZY trying to understand why my rover's ability to drive kept getting worse.  I finally tracked it down and fixed, and now, it's immune to the toxic effects of the rocks. :)

##### Rock Pickup hang

I did not report this. It's rare.  But it's possible for the simulator to detect that the rover is near the rock, have the drive code tell it to pick up, and the simulator then sets the "picking up" flag. But then the rover MOES away from the rock (physics at work), and the simulator will no longer be able to reach the rock. But the simulator just hangs at this point.  It's in rock pickup mode, but not picking up rocks, and not allowing the drive code to send more commands.  I could not fix this from my side of the code.

##### Velocity Time Warp on negative throttle

I described this above.  You might call this a feature instead of a bug, but the physics is non-life-like for this.  When the rover is traveling faster than about 3 m/s, and negative throttle is applied (should create slow brake effect), the speed of the rover can take a time warp jump down from 3 m/s to 2 m/s in what I think is one update cycle (massive negative acceleration).  This messes up the PID from being able to use negative throttle to control speed.  I did not carefully explore this issue, I just gave up on using it.  I could help debug this and produce better documentation of the effect if it would help you guys. Just let me know.
