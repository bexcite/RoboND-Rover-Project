import numpy as np
import cv2

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

def color_thresh_inv(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
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
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)    # Return the result
    # Return the result
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))    # Return the result
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


def get_src_dst(image_shape=(160, 320, 3)):
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image_shape[1]/2 - dst_size, image_shape[0] - bottom_offset],
                  [image_shape[1]/2 + dst_size, image_shape[0] - bottom_offset],
                  [image_shape[1]/2 + dst_size, image_shape[0] - 2*dst_size - bottom_offset],
                  [image_shape[1]/2 - dst_size, image_shape[0] - 2*dst_size - bottom_offset],
                  ])
    return source, destination


# Adds value to the map for a given channel with clip
def add_to_map(worldmap, channel, xx, yy, val=1):
    layer = worldmap[yy, xx, channel]
    m = layer < 256 - val
    layer[m] += val
    worldmap[yy, xx, channel] = layer

def clip_to_view_horizon(img, horizon=75, bottom=0):
    # print("==== clip horizon")
    masked = np.copy(img)
    # print("==== clip horizon 2, image_shape = ", masked.shape)
    ishape = img.shape
    # print("== masked = ", masked)
    masked[0:ishape[0]-horizon, 0:ishape[1]] = (0, 0, 0)
    masked[ishape[0]-bottom:ishape[0], 0:ishape[1]] = (0, 0, 0)
    # l1 = 160 - horizon
    # masked[0:2, 0:2] = (0, 0, 0)
    # print("masked in clip = ", masked)
    return masked

def get_view_mask(image_shape = (160, 320, 3), horizon=75, bottom=0):
    white_image = np.ones(image_shape, dtype=np.uint8)*255

    # print("WI = ", white_image)

    source, destination = get_src_dst()

    # print("SD = ", source, destination)

    white_image = clip_to_view_horizon(white_image, horizon, bottom)

    # print("CLIP = ", white_image)

    warped_white = perspect_transform(white_image, source, destination)

    # print("WW = ", warped_white)

    mask = color_thresh(warped_white, (250, 250, 250))

    # print("MASK = ", mask)
    return mask


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()

    # view_horizon = 70
    thresh_floor = (110, 110, 110)
    thresh_wall = (80, 80, 80)
    thresh_diamond = ((0,30), (120,255), (120,255)) # HSV
    # thresh_diamond = ((86,255), (0,255), (0,42)) # ((86,255), (0,255), (0,62))


    image = Rover.img

    x = Rover.pos[0]
    y = Rover.pos[1]
    yaw = Rover.yaw

    # Set View Mask
    if Rover.view_mask is None:
      # print("GET VIEW MASK ========================")
      Rover.view_mask = get_view_mask(horizon=75, bottom=0)
      Rover.nav_mask = get_view_mask(horizon=75, bottom=50)
      # print(Rover.view_mask)

    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform

    source, destination = get_src_dst()


    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    floor_threshed = color_thresh(image, thresh_floor)
    floor_warped = perspect_transform(floor_threshed, source, destination)
    floor_masked = np.multiply(floor_warped, Rover.nav_mask)

    obs_threshed = color_thresh_inv(image, thresh_wall)
    obs_warped = perspect_transform(obs_threshed, source, destination)
    obs_masked = np.multiply(obs_warped, Rover.view_mask)

    # diam_clipped = np.copy(image) # clip_to_view_horizon(image, 80)
    diam_clipped = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    diam_color_select = np.zeros_like(diam_clipped[:,:,0])
    diam_thresh = (diam_clipped[:,:,0] > thresh_diamond[0][0]) & (diam_clipped[:,:,0] < thresh_diamond[0][1]) \
                & (diam_clipped[:,:,1] > thresh_diamond[1][0]) & (diam_clipped[:,:,1] < thresh_diamond[1][1]) \
                & (diam_clipped[:,:,2] > thresh_diamond[2][0]) & (diam_clipped[:,:,2] < thresh_diamond[2][1])
    diam_color_select[diam_thresh] = 1
    diam_warped = perspect_transform(diam_color_select, source, destination)
    # diam_warped = np.copy(diam_color_select)



    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image



    Rover.vision_image[:, :, 2] = floor_masked * 255
    Rover.vision_image[:, :, 0] = obs_masked * 255
    Rover.vision_image[:, :, 1] = diam_warped * 255

    # 5) Convert map image pixel values to rover-centric coords

    # floor_masked_nav = np.multiply(floor_warped, Rover.nav_mask)

    xpix_nav, ypix_nav = rover_coords(floor_masked)
    xpix_obs, ypix_obs = rover_coords(obs_masked)
    xpix_diam, ypix_diam = rover_coords(diam_warped)

    # 6) Convert rover-centric pixel values to world coordinates

    xpix_nav_world, ypix_nav_world = pix_to_world(xpix_nav, ypix_nav, x, y, yaw, 200, 10)
    xpix_obs_world, ypix_obs_world = pix_to_world(xpix_obs, ypix_obs, x, y, yaw, 200, 10)
    xpix_diam_world, ypix_diam_world = pix_to_world(xpix_diam, ypix_diam, x, y, yaw, 200, 10)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    add_to_map(Rover.worldmap, 2, xpix_nav_world, ypix_nav_world, 1)
    add_to_map(Rover.worldmap, 0, xpix_obs_world, ypix_obs_world, 1)
    add_to_map(Rover.worldmap, 1, xpix_diam_world, ypix_diam_world, 1)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    dist_nav, angles_nav = to_polar_coords(xpix_nav, ypix_nav)
    Rover.nav_dists = dist_nav
    Rover.nav_angles = angles_nav

    dist_rock, angles_rock = to_polar_coords(xpix_diam, ypix_diam)
    Rover.rock_dists = dist_rock
    Rover.rock_angles = angles_rock
    if len(Rover.rock_dists) > 0:
      d = np.mean(Rover.rock_dists)
      a = np.mean(Rover.rock_angles)
      rx = d * np.cos(a)
      ry = d * np.sin(a)
      rx_w, ry_w = pix_to_world(rx, ry, x, y, yaw, 200, 10)
      Rover.rock_pos = (rx_w, ry_w)
      Rover.rock_ttl = 5
    elif Rover.rock_ttl > 0:
      Rover.rock_ttl -= 1
    else:
      Rover.rock_pos = None
      Rover.rock_dists = []
      Rover.rock_angles = []

    print('rock_pos = ', Rover.rock_pos)
    print('len(rock_angles) = ', len(Rover.rock_angles))
    print('rock_ttl = ', Rover.rock_ttl)

    print("x = ", x, ", y =", y, ", yaw = ", yaw)

    # print('rock map nearby = ', Rover.worldmap[y-5:y+5, x-5:x+5, 1])




    # print(">>> Samples pos = ", Rover.samples_pos)
    # print('len rock_dists_angles = ', len(Rover.rock_dists))

    return Rover
