from pprint import pprint

map = [
    [3,3,4,9,8,4],
    [5,10,4,9,8,8],
    [6,6,20,9,9,7],
    [7,8,9,30,6,5],
    [7,8,5,3,3,3],
    [6,5,3,2,1,0]
]

class node:
    r = 0
    c = 0
    g = 0
    h = 0
    f = 0
    height = 0
    parent = None
    
    def __init__(self, r, c):
        self.row = r
        self.col = c

    def calc_g(self, parent):
        self.parent = parent
        self.g = 1 + self.parent.g

    def calc_h(self, goal):
        self.h = ((goal.row - self.row)**2)  + ((goal.col - self.col)**2)

    def calc_f(self):
        self.f = self.g + self.h + self.height

    def dijkstra(self):    
        self.f = self.h
        return self.f

def get_f(d): 
    return d.f

def is_in_list(lst, cell):
    for pos in range(len(lst)):
        if lst[pos].row == cell.row and lst[pos].col == cell.col:
            return pos
    
    return -1

def get_path(nod, start):
    if nod.row  == start.row and nod.col  == start.col:
        return [nod.row, nod.col]
    else:
        get_path(nod.parent, start)
        return [nod.row, nod.col] 

open_list = []
closed_list = []

start = node(0, 0)
goal = node(5, 5)

open_list.append(start)

neighbors_address = [[1 ,1], [1 ,-1], [-1 ,1], [-1 ,-1],
                     [0, 1], [0, -1], [1, 0], [-1, 0]] 

while open_list is not None:
    open_list.sort(key=get_f)

    current = open_list[0]
    open_list.pop(0)
    
    closed_list.append(current)
    print("Looping")
    if current.row == goal.row and current.col == goal.col:
        path = get_path(current, start)
        
        nod = current
        pprint(map)
        while nod.row != start.row and nod.row != start.row:
            map[nod.row][nod.col] = -1
            nod = nod.parent

        map[start.row][start.col] = -1
        map[goal.row][goal.col] = -1

        pprint(map)
        break

    for side in neighbors_address:
        child = node(current.row+side[0], current.col+side[1])
        r_in_range = (child.row>=0) and (child.row<len(map))
        c_in_range = (child.col>=0) and (child.col<len(map[0]))
        if c_in_range and r_in_range:
            child.height = map[child.row][child.col]
            

            if is_in_list(closed_list, child) == -1 and r_in_range and c_in_range: # and abs(current.height - child.height) < 3:

                child.calc_g(current)
                child.calc_h(goal)
                
                child.calc_f()
                # child.dijkstra()
            
                pos = is_in_list(open_list, child) 
                if pos == -1 or (pos >= 0 and open_list[pos].g > child.g):
                    open_list.append(child)
