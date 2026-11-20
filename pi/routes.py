"""Four original 4x4 routes. (0,0) start, robot faces +X."""
GRID_SIZE = 4
ROUTES = {
    1: [(0,0),(1,0),(2,0),(3,0),(3,1),(2,1),(1,1),(0,1),(0,2),(1,2),(2,2),(3,2),(3,3),(2,3),(1,3),(0,3)],
    2: [(0,0),(0,1),(0,2),(0,3),(1,3),(1,2),(1,1),(1,0),(2,0),(2,1),(2,2),(2,3),(3,3),(3,2),(3,1),(3,0)],
    3: [(0,0),(1,0),(2,0),(3,0),(3,1),(3,2),(3,3),(2,3),(1,3),(0,3),(0,2),(0,1),(1,1),(2,1),(2,2),(1,2)],
    4: [(0,0),(0,1),(1,1),(1,0),(2,0),(3,0),(3,1),(2,1),(2,2),(3,2),(3,3),(2,3),(1,3),(1,2),(0,2),(0,3)],
}

def validate_route(cells):
    if len(cells) != 16 or cells[0] != (0,0) or len(set(cells)) != 16:
        raise ValueError('Route must visit 16 unique cells starting at (0,0)')
    if any(not (0 <= x < 4 and 0 <= y < 4) for x,y in cells):
        raise ValueError('Grid cell out of bounds')
    for a,b in zip(cells,cells[1:]):
        if abs(b[0]-a[0])+abs(b[1]-a[1]) != 1:
            raise ValueError('Nonadjacent move: {} -> {}'.format(a,b))
    return True

def heading_for_move(a,b):
    direction = (b[0]-a[0], b[1]-a[1])
    return {(1,0):0,(0,1):90,(-1,0):180,(0,-1):270}[direction]

def signed_delta(desired,current):
    return (desired-current+180)%360-180

for _cells in ROUTES.values():
    validate_route(_cells)
