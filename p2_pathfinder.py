from math import inf

def matching_box(point, mesh):
    x, y = point
    for box in mesh['boxes']:
        y1, y2, x1, x2 = box
        if y <= y2 and y >= y1 and x >= x1 and x <= x2:
            return box

def midpoint(box):
    y1, y2, x1, x2 = box
    return ((x1+x2)/2, (y1+y2)/2)

def assemble_path(source_box, destination_box, boxes):
    path = []
    current_box = destination_box
    while current_box != source_box:
        segment = (midpoint(current_box), midpoint(boxes[current_box]))
        path.append(segment)
        current_box = boxes[current_box]
    return path

def find_path (source_point, destination_point, mesh):
    """    Searches for a path from source_point to destination_point through the mesh    Args:        source_point: starting point of the pathfinder        destination_point: the ultimate goal the pathfinder must reach        mesh: pathway constraints the path adheres to
    Returns:
        A path (list of points) from source_point to destination_point if exists        A list of boxes explored by the algorithm    """

    source_box = matching_box(source_point, mesh)
    destination_box = matching_box(destination_point, mesh)

    queue = [ source_box ]
    boxes = {}
    boxes[source_box] = None

    while len(queue) > 0:
        current = queue.pop(0)
        if current == destination_box:
            break
        for next in mesh['adj'][current]:
            if next not in boxes:
                queue.append(next)
                boxes[next] = current

    path = assemble_path(source_box, destination_box, boxes)
    print(path)

    return path, mesh['adj'].keys()
