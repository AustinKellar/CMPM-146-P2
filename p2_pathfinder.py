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
    #x1, y1 = point1
    #x2, y2 = point2

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


def calculate_x(cur1, cur2, next1, next2, current_point):
    #print("in calc x")
    #just a lazy way to sort the x coordinates in ascending order
    sort = PriorityQueue()
    sort.put(cur1[0])
    sort.put(cur2[0])
    sort.put(next1[0])
    sort.put(next2[0])
    #we don't want the farthest left x point and the farthest right x point but the two in the middle
    #so we pop off the first x point to get rid of it and keep the next 2 and ignore the last one
    sort.get()
    #x1 and X2 hold the values of the two coordinates we want to range check with our current_point
    x1 = sort.get()
    x2 = sort.get()
    #check to see if where current_point lies in relation to x1 and x2 (left inbetween or right)

    #if current_point is to the left of x1 return the point x1 and one of the y values (they should all be the same)
    if(x1 > current_point[0]):
        return (x1, cur1[1])
    #if the current_point lies between the two bounds we want to make a straight line so we keep the x value of current_point and get a y value
    elif(x1 <= current_point[0] <= x2):
        return(current_point[0], cur1[1])
    #if the current_point lies to the left of x2, return the point x2 and one of the y values
    else:
        return(x2, cur1[1])

def calculate_y(cur1, cur2, next1, next2, current_point):
    #like with calc x, just a lazy way to sort
    #everything in this method is the same as calculate_x just with y values instead of x, refer to comments in that as reference to what
    #i'm doing
    #print("in calc y")
    sort = PriorityQueue()
    sort.put(cur1[1])
    sort.put(cur2[1])
    sort.put(next1[1])
    sort.put(next2[1])
    sort.get()
    y1 = sort.get()
    y2 = sort.get()
    if(y1 > current_point[1]):
        return(cur1[0], y1)
    elif(y1 <= current_point[1] <= y2):
        return(cur1[0], current_point[1])
    else:
        return(cur1[0], y2)

def closest(current_box, next_box, current_point):
    #print("in closest: " + str(next_box))
    #gets the cooridnates of current box and next box
    x1, x2, y1, y2 = current_box
    a1, a2, b1, b2 = next_box
    #checks to see if the current box is beneath the next_box
    if(y1 == b2):
        new_point = calculate_x((x1,y1), (x2,y1), (a1,b2), (a2,b2), current_point)
    #checks to see if the current box is on top of the next_box
    elif(y2 == b1):
        new_point = calculate_x((x1,y2), (x2,y2), (a1, b1), (a2, b1), current_point)
    #checks to see if the current box is to the right of next_box
    elif(x1 == a2):
        new_point = calculate_y((x1,y1), (x1,y2), (a2,b1), (a2,b2), current_point)
    #checks to see if the current box is to the left of next_box
    elif(x2 == a1):
        new_point = calculate_y((x2,y1), (x2,y2), (a1,b1), (a1,b2), current_point)
    #print("returning point: " + str(new_point) + " next: " + str(next_box))
    return new_point
    
    
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

    return (path, boxes.keys())
