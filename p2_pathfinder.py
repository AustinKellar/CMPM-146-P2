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
    queue.put((0, source_box, destination_box, 'fwd'))
    queue.put((0, destination_box, source_box, 'bkwd'))

    fwd_dist = { source_box: 0 }
    fwd_prev = { source_box: None }

    bkwd_dist = { destination_box: 0}
    bkwd_prev = { destination_box: None }

    fwd_detail_points = { source_box: source_point, destination_box: destination_point }
    bkwd_detail_points = { source_box: source_point, destination_box: destination_point }

    fwd_closed = []
    bkwd_closed = []

    while not queue.empty():
        weight, curr_box, curr_goal, direction = queue.get()

        if (direction == 'fwd' and curr_box in bkwd_closed) or (direction == 'bkwd' and curr_box in fwd_closed):
            break

        for next_box in mesh['adj'][curr_box]:
            if direction == 'fwd':
                if next_box != source_box and next_box != destination_box:
                    fwd_detail_points[next_box] = midpoint(next_box)

                new_dist = fwd_dist[curr_box] + distance(fwd_detail_points[curr_box], fwd_detail_points[next_box])

                if next_box not in fwd_dist or new_dist < fwd_dist[next_box]:
                    fwd_dist[next_box] = new_dist
                    priority = new_dist + distance(fwd_detail_points[next_box], fwd_detail_points[destination_box])
                    queue.put((priority, next_box, destination_box, 'fwd'))
                    fwd_prev[next_box] = curr_box
                    fwd_closed.append(curr_box)
            else:
                if next_box != source_box and next_box != destination_box:
                    bkwd_detail_points[next_box] = midpoint(next_box)
                    
                new_dist = bkwd_dist[curr_box] + distance(bkwd_detail_points[curr_box], bkwd_detail_points[next_box])

                if next_box not in bkwd_dist or new_dist < bkwd_dist[next_box]:
                    bkwd_dist[next_box] = new_dist
                    priority = new_dist + distance(bkwd_detail_points[next_box], bkwd_detail_points[source_box])
                    queue.put((priority, next_box, source_box, 'bkwd'))
                    bkwd_prev[next_box] = curr_box
                    bkwd_closed.append(curr_box)

    boxes = fwd_dist
    boxes.update(bkwd_dist)

    fwd_path = create_path(curr_box, fwd_prev, fwd_detail_points)
    bkwd_path = create_path(curr_box, bkwd_prev, bkwd_detail_points)

    path = fwd_path + bkwd_path

    return path, boxes.keys()
