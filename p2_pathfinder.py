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

def assemble_path(source_point, destination_point, parents):
    print("in assemble path")
    path = []
    current = destination_point

    while current != source_point:
        #print("current point in path: " +str(current))
        path.append(((current), (parents[current])))
        current = parents[current]
    #print("returning path")
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
    queue.put((0, source_box))
    boxes = {}
    boxes[source_box] = None
    distances = { source_box: 0 }
    detail_points = { source_box: source_point }
    parents = { source_point: None }


    while not queue.empty():
        current = (queue.get())[1]
        if current == destination_box:
            #parents[destination_point] = detail_points[current]
            break
        #print(len(mesh['adj'][current]))
        print("checking adj")
        #print("current: " + str(current))
        for next in mesh['adj'][current]:
            if next == destination_box:
                detail_point = destination_point
            else:
                detail_point = closest(current, next, detail_points[current])
            #print("detail_point: " + str(detail_point)+ " next: " + str(next))

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
    print("done with queue")
    #for parent,point in parents:
        #print("current: "+str(point) + " parent: "+ str(parent))
    #for point in detail_points:
        #print(str(point))
    path = assemble_path(source_point, destination_point, parents)
    print("returning path")

    return (path, boxes.keys())
