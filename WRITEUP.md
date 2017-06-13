# Search and Sample Return Project writeup

## Describe in your writeup (and identify where in your code) how you modified or added functions to add obstacle and rock sample identification.

Obstacle identification is implemented with a color thresholding in RGB color space. With the pixel values in the range (0, 90) - R, (0,255) - G, (0, 140) - B. Exact values was tuned through the interactive widgets feature in Jupyter notebook.

The relevant code in `perception.py:191-197`

```
thresh_wall = ((0,90), (0, 255), (0, 140)) # RGB

obs_color_select = np.zeros_like(image[:,:,0])
obs_thresh = (image[:,:,0] > thresh_wall[0][0]) & (image[:,:,0] < thresh_wall[0][1]) \
            & (image[:,:,1] > thresh_wall[1][0]) & (image[:,:,1] < thresh_wall[1][1]) \
            & (image[:,:,2] > thresh_wall[2][0]) & (image[:,:,2] < thresh_wall[2][1])
obs_color_select[obs_thresh] = 1
obs_warped = perspect_transform(obs_color_select, source, destination)
obs_masked = np.multiply(obs_warped, Rover.view_mask)
```

For rock sample identification I've made a conversion to the HSV color space and tuned the relevant parameters for H - (0,30), S - (130, 255), V - (130, 255)

```
thresh_diamond = ((0,30), (130,255), (130,255)) # HSV

diam_clipped = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
diam_color_select = np.zeros_like(diam_clipped[:,:,0])
diam_thresh = (diam_clipped[:,:,0] > thresh_diamond[0][0]) & (diam_clipped[:,:,0] < thresh_diamond[0][1]) \
            & (diam_clipped[:,:,1] > thresh_diamond[1][0]) & (diam_clipped[:,:,1] < thresh_diamond[1][1]) \
            & (diam_clipped[:,:,2] > thresh_diamond[2][0]) & (diam_clipped[:,:,2] < thresh_diamond[2][1])
diam_color_select[diam_thresh] = 1
diam_warped = perspect_transform(diam_color_select, source, destination)
```

For obstacle and navigable pixel identification I've also applied the `view_mask` that selects only pixels in the range of y = [85, 160] for obstacle and y = [85, 110] for navigable pixels. Mask is obtained in `get_view_mask()`:

```
def get_view_mask(image_shape = (160, 320, 3), horizon=75, bottom=0):
    white_image = np.ones(image_shape, dtype=np.uint8)*255

    source, destination = get_src_dst()

    white_image = clip_to_view_horizon(white_image, horizon, bottom)

    warped_white = perspect_transform(white_image, source, destination)

    mask = color_thresh(warped_white, (250, 250, 250))

    return mask
```

## Describe in your writeup how you modified the `process_image()` to demonstrate your analysis and how you created a worldmap. Include your video output with your submission.

`process_image()` in general follows what was suggested in the comments. Find thresholded pixels for navigable area, obstacles and rocks separately and combine them in one worldmap image in different RGB channels. So Red is obstacles, Blue - navigable area and Green - rocks. Before we can add pixels to the map we also should transform them to bird-eye view and rotate/translate/scale to the worldmap coordinates.


## `perception_step()` and `decision_step()` functions have been filled in and their functionality explained in the writeup.

In the `perception_step()` we are identifying navigable areas, obstacles and rocks and map them to the worldmap. Only pixels that was taken with `roll` and `pitch` below 2 degree considered as a good pixels because otherwise our bird-eye transformation will have a big error due to constant src and destination points. Ideally we should factor it and automatically warp picture for all possible angles of `roll` and `pitch` or we lose a lot of information from our vision.

Also `nav_angles` and `rock_angles` is storing for later use in `decision_step()` and `control_step()`

I've added additional method that removes rock from the map when rover finished picking it up, so rock disappears from the map when it picked up.

`decision_step()` default mode is `follow_wall` when rover stick to the left wall. Other modes is `rotate_right` used when there is not enough navigable pixels or rover is stuck in sand or rock or wall inclination. `forward_stop` mode is using to start rock hunting when there is visible rock pixels.

Additionally I've introduced `control_step` where a simple PD controller is implemented.

There also was a lot of experiments in PD controller for throttle and it's combination with PD controller for steering that didn't worked well for stopping. Also I've thought initially that I can build planning and navigation mostly on map, but find a lot of problem with map inaccuracy for such planning and combination of map data with actual situational vision data. So I rolled back everything to the simplest wall following solution that works reasonable well.

Hope in following parts of the course we will have more chance to build a proper controller, planner and decision step that works much better together.


## By running `drive_rover.py` and launching the simulator in autonomous mode, your rover does a reasonably good job at mapping the environment.

The result of running `drive_rover.py` in video below. 13-15 FPS during video recording, and around 22 FPS without. Simulator mode - 640x480, Fastest.

[![Rover Driving and Mapping](https://img.youtube.com/vi/rDij00NB7pA/0.jpg)](https://www.youtube.com/watch?v=rDij00NB7pA)
