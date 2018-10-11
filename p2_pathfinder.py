from queue import PriorityQueue
from math import inf, sqrt

def matching_box(point, mesh):
    x, y = point
    for box in mesh['boxes']:
        x1, x2, y1, y2 = box
        if y <= y2 and y >= y1 and x >= x1 and x <= x2:
            return box

def midpoint(box):
    x1, x2, y1, y2 = box
    return ((x1+x2)/2, (y1+y2)/2)

def heuristic(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    return abs(x2 - x1) + abs(y2 - y1)

def distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    return sqrt(((x2 - x1)**2) + ((y2 - y1)**2))

def create_path(box, prev, points):
    path = []
    curr_box = box

    while prev[curr_box] != None:
        prev_box = prev[curr_box]
        p1 = points[curr_box]
        p2 = points[prev_box]
        path.append((p1, p2))
        curr_box = prev_box
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

    if source_box == destination_box:
        return [(source_point, destination_point)], [ source_box ]

    queue = PriorityQueue()
    queue.put((0, source_box))

    dist = { source_box: 0 }
    prev = { source_box: None }

    detail_points = { source_box: source_point, destination_box: destination_point }

    while not queue.empty():
        _,curr_box = queue.get()

        if curr_box == destination_box:
            break

        for next_box in mesh['adj'][curr_box]:
            if next_box != source_box and next_box != destination_box:
                detail_points[next_box] = midpoint(next_box)

            new_dist = dist[curr_box] + distance(detail_points[curr_box], detail_points[next_box])

            if next_box not in dist or new_dist < dist[next_box]:
                dist[next_box] = new_dist
                priority = new_dist + distance(detail_points[next_box], detail_points[destination_box])
                queue.put((priority, next_box))
                prev[next_box] = curr_box

    path = create_path(destination_box, prev, detail_points)

    return path, dist.keys()
