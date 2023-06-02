def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
    img = img.resize((new_x, new_y), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')
    map_array =  np.transpose(map_array)
    # Get bianry image
    threshold = 127
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array

from PIL import Image, ImageDraw
import random, math

from PIL import Image
import numpy as np
from RRT import RRT
# Define the size of the map
width = 256
height = 256

import time
point = {}
area = {}

i = 0

while True:
    print(i)
    prob_map = np.zeros((height, width))
    if i>500:
        break
    # time.sleep(1)
    # Create a new image with a white background
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)

    labels = Image.new('RGB', (width, height), (255, 255, 255))
    draw2 = ImageDraw.Draw(labels)

    # Define the colors for the start and goal points
    start_color = (255, 0, 0)  # Red
    goal_color = (0, 255, 0)   # Green

    # Generate random obstacles
    num_obstacles = random.randint(5, 20)  # Adjust the range as desired
    obstacles = []

    for _ in range(num_obstacles):
        # Randomly choose the obstacle shape (rectangle or circle)
        shape = random.choice(["rectangle"])

        if shape == "rectangle":
            # Randomly generate the position and size of the rectangle
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            w = random.randint(10, 100)
            h = random.randint(10, 100)

            # Add the rectangle obstacle to the list
            obstacles.append((x, y, x + w, y + h, 'rectangle'))

            # Draw the rectangle obstacle
            draw.rectangle([(x, y), (x + w, y + h)], fill="black")

        elif shape == "circle":
            # Randomly generate the position and size of the circle
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            radius = random.randint(5, 20)

            # Add the circle obstacle to the list
            obstacles.append((x - radius, y - radius, x + radius, y + radius, 'circle'))

            # Draw the circle obstacle
            draw.ellipse([(x - radius, y - radius), (x + radius, y + radius)], fill="black")

    # Generate random start and goal points outside the obstacles
    start_x, start_y, goal_x, goal_y = None, None, None, None

    while True:
        start_x = random.randint(0, width - 1)
        start_y = random.randint(0, height - 1)
        goal_x = random.randint(0, width - 1)
        goal_y = random.randint(0, height - 1)

        # Check if the start and goal points are outside the obstacles
        if all(
            not (
                obstacle[0] <= start_x <= obstacle[2]
                and obstacle[1] <= start_y <= obstacle[3]
            )
            for obstacle in obstacles
        ) and all(
            not (
                obstacle[0] <= goal_x <= obstacle[2] and obstacle[1] <= goal_y <= obstacle[3]
            )
            for obstacle in obstacles
        ):
            # Check if a clear path exists between start and goal points
            clear_path = True

            for obstacle in obstacles:
                if (
                    obstacle[0] <= start_x <= obstacle[2]
                    or obstacle[0] <= goal_x <= obstacle[2]
                ) and (
                    obstacle[1] <= start_y <= obstacle[3]
                    or obstacle[1] <= goal_y <= obstacle[3]
                ):
                    clear_path = False
                    break

            if clear_path:
                break
    # radius = 3
    # draw.ellipse([(start_x - radius, start_y - radius), (start_x + radius, start_y + radius)], fill=start_color)
    # draw.ellipse([(goal_x - radius, goal_y - radius), (goal_x + radius, goal_y + radius)], fill=goal_color)
    # # Draw the start and goal points
    # draw.point([(start_x, start_y)], fill=start_color)
    # draw.point([(goal_x, goal_y)], fill=goal_color)
    reachable_pixels = set()

    def bresenham_line(x1, y1, x2, y2):
        """Bresenham's line algorithm."""
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        line = []
        while True:
            line.append((x1, y1))

            if x1 == x2 and y1 == y2:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy

        return line

    for x in range(width):
        for y in range(height):
            if (
                (x, y) != (goal_x, goal_y)
                and all(
                    not (
                        obstacle[0] <= x <= obstacle[2] and obstacle[1] <= y <= obstacle[3]
                    )
                    for obstacle in obstacles
                )
            ):
                line = bresenham_line(goal_x, goal_y, x, y)
                if all(
                    not (
                        (obstacle[4] == "rectangle" and obstacle[0] <= p[0] <= obstacle[2] and obstacle[1] <= p[1] <= obstacle[3]) or
                        (obstacle[4] == "circle" and math.sqrt((p[0] - (obstacle[0] + obstacle[2]) / 2) ** 2 + (p[1] - (obstacle[1] + obstacle[3]) / 2) ** 2) <= (obstacle[2] - obstacle[0]) / 2)
                    )
                    for p in line
                    for obstacle in obstacles
                ):
                    prob_map[y][x] = 1
                    reachable_pixels.add((x, y))

    # Draw the reachable pixels in red
    # for pixel in reachable_pixels:
    #     draw.point(pixel, fill='red')
    # Save the image to a file
    area[i] = reachable_pixels
    point[i] = (start_x, start_y, goal_x, goal_y)
    start = (point[i][0], point[i][1])
    goal = (point[i][2], point[i][3])
    image.save("demo/ori_map/{}.png".format(i))
    map_array = load_map("demo/ori_map/{}.png".format(i), 1)
    RRT_planner = RRT(map_array, start, goal)
    # RRT_planner.set_save_path("demo/labels/{}.png".format(i))
    if RRT_planner.RRT(n_pts=1000):
        image.save("demo/test/maps/{}.png".format(i))
        np.save('demo/test/prob_map/{}.npy'.format(i), prob_map)

        radius = 3
        draw.ellipse([(start_x - radius, start_y - radius), (start_x + radius, start_y + radius)], fill=start_color)
        draw.ellipse([(goal_x - radius, goal_y - radius), (goal_x + radius, goal_y + radius)], fill=goal_color)
        # Draw the start and goal points
        draw.point([(start_x, start_y)], fill=start_color)
        draw.point([(goal_x, goal_y)], fill=goal_color)
        image.save("demo/test/data/{}.png".format(i))
        

        for pixel in reachable_pixels:
            draw2.point(pixel, fill='black')
        labels.save("demo/test/label/{}.png".format(i))
        
        i+=1
    else:
        continue
    # for pixel in reachable_pixels:
    #     draw.point(pixel, fill='red')
    # Show the image
    # image.show()
np.save('demo/area5.npy', area)
np.save('demo/test_point.npy', point)