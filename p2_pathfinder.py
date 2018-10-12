from queue import PriorityQueue
from math import inf, sqrt

#REMEMBER x1,y1 = top left x2,y2 = bottom right
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
    return distance(point1, point2)

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

def entrance_point(curr_point, b1, b2):
    px, py = curr_point
    b1x1, b1x2, b1y1, b1y2 = b1
    b2x1, b2x2, b2y1, b2y2 = b2

    lx = max(b1x1, b2x1) # right bound
    ux = min(b1x2, b2x2) # left bound

    ly = max(b1y1, b2y1) # top bound (lowest y value)
    uy = min(b1y2, b2y2) # bottom bound (highest y value)

    if px < lx:
        x = lx
    elif px > ux:
        x = ux
    else:
        x = px

    if py < ly:
        y = ly
    elif py > uy:
        y = uy
    else:
        y = py

    return (x, y)
    
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
                    fwd_detail_points[next_box] = entrance_point(fwd_detail_points[curr_box], curr_box, next_box)

                new_dist = fwd_dist[curr_box] + distance(midpoint(curr_box), midpoint(next_box))

                if next_box not in fwd_dist or new_dist < fwd_dist[next_box]:
                    fwd_dist[next_box] = new_dist
                    priority = new_dist + heuristic(midpoint(next_box), fwd_detail_points[destination_box])
                    queue.put((priority, next_box, destination_box, 'fwd'))
                    fwd_prev[next_box] = curr_box
                    fwd_closed.append(curr_box)
            else:
                if next_box != source_box and next_box != destination_box:
                    bkwd_detail_points[next_box] = entrance_point(bkwd_detail_points[curr_box], curr_box, next_box)
                    
                new_dist = bkwd_dist[curr_box] + distance(midpoint(curr_box), midpoint(next_box))

                if next_box not in bkwd_dist or new_dist < bkwd_dist[next_box]:
                    bkwd_dist[next_box] = new_dist
                    priority = new_dist + heuristic(midpoint(next_box), bkwd_detail_points[source_box])
                    queue.put((priority, next_box, source_box, 'bkwd'))
                    bkwd_prev[next_box] = curr_box
                    bkwd_closed.append(curr_box)

    boxes = fwd_dist
    boxes.update(bkwd_dist)

    fwd_path = create_path(curr_box, fwd_prev, fwd_detail_points)
    bkwd_path = create_path(curr_box, bkwd_prev, bkwd_detail_points)

    path = fwd_path + bkwd_path
    path.append((fwd_path[0][0], bkwd_path[0][0]))

    return (path, boxes.keys())
