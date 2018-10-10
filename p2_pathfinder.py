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

def assemble_path(source_point, destination_point, parents):
    path = []
    current = destination_point

    while current != source_point:
        path.append(((current), (parents[current])))
        current = parents[current]

    return path

def closest(point, box):

    return midpoint(box)

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
    boxes = {}
    boxes[source_box] = None
    distances = { source_box: 0 }
    detail_points = { source_box: source_point }
    parents = { source_point: None }


    while not queue.empty():
        current = (queue.get())[1]
        if current == destination_box:
            break
        for next in mesh['adj'][current]:
            if next == destination_box:
                detail_point = destination_point
            else:
                detail_point = closest(detail_points[current], next)

            detail_points[next] = detail_point

            new_distance = distances[current] + distance(detail_points[current], detail_point)

            if next not in distances or new_distance < distances[next]:
                distances[next] = new_distance
                priority = new_distance + heuristic(detail_point, destination_point)
                queue.put((priority, next))

                if current == source_box:
                    parents[detail_point] = source_point
                else:
                    parents[detail_point] = detail_points[current]

                boxes[next] = current

    path = assemble_path(source_point, destination_point, parents)

    return path, boxes.keys()
