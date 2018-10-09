from queue import PriorityQueue
from math import inf, sqrt
import heapq

def matching_box(point, mesh):
    x, y = point
    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if y <= y2 and y >= y1 and x >= x1 and x <= x2:
            return box

def midpoint(box):
    x1, x2, y1, y2 = box
    return ((x1+x2)/2, (y1+y2)/2)

def heuristic(box1, box2):
    x1, y1 = midpoint(box1)
    x2, y2 = midpoint(box2)

    return abs(x2 - x1) + abs(y2 - y1)

def distance(box1, box2):
    x1, y1 = midpoint(box1)
    x2, y2 = midpoint(box2)

    return sqrt(((x2 - x1)**2) + ((y2 - y1)**2))

def assemble_path(source_box, destination_box, boxes):
    path = []
    current_box = destination_box
    while current_box != source_box:
        if current_box == None:
            print("No path possible!")
            return None
        segment = (midpoint(current_box), midpoint(boxes[current_box]))
        path.append(segment)
        current_box = boxes.get(current_box, None)
    return path

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """



    source_box = matching_box(source_point, mesh)
    destination_box = matching_box(destination_point, mesh)

    queue = []
    heapq.heappush(queue, (source_box, 0))
    boxes = {}
    boxes[source_box] = None
    distances = {}

    for box in mesh['boxes']:
        distances[box] = inf

    distances[source_box] = 0

    while not queue:
        current = heapq.heappop(queue)
        if current == destination_box:
            break
        for next in mesh['adj'][current]:
            new_distance = distances[current] + distance(current, next)
            if next not in distances or new_distance < distances[next]:
                distances[next] = new_distance
                priority = new_distance + heuristic(next, destination_box)
                heapq.heappush(queue, (next, priority))
                boxes[next] = current

    path = assemble_path(source_box, destination_box, boxes)

    return path, boxes.keys()

