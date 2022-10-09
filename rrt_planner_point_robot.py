'''
Hello and welcome to the first assignment :)
Please try to enjoy implementing algorithms you learned from the course; in this regard, we also have tried
to provide a good starting point for you to develop your own code. In this script, you may find different functions
that should be defined by you to perform specific tasks. A good suggestion would be to begin with the primary
function for the RRT algo named as "rrt_search". Following this function line by line you may realize how to define
each function to make it work!
Note, you may use the following commands to run this script with the required arguments:
python3 rrt_planner_point_robot.py --arg1_name your_input1 --arg2_name your_input2 e.g.
python3 rrt_planner_point_robot.py --world="shot.png"
To see the list of arguments, take a look at utils.py
Also note:
G is a list containing two groups: the first group includes the nodes IDs (0,1,2,...), while the second holds all pairs of nodes creating an edge
Vertices is a list of nodes locations in the map; it corresponds to the nodes' IDs mentioned earlier
GOOD LUCK!
'''

from dis import dis
import random
import drawSample
import sys
import imageToRects
import utils
import math

def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")

def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )

def genPoint():
    if args.rrt_sampling_policy == "uniform":
        # Uniform distribution
        x = random.random()*XMAX
        y = random.random()*YMAX
    elif args.rrt_sampling_policy == "gaussian":
        # Gaussian with mean at the goal
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
    else:
        print ("Not yet implemented")
        quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    line = []
    llsq = 0.0 # line length squared
    for i in range(len(p1)):  # each dimension
        h = p2[i] - p1[i]
        line.append( h )
        llsq += h*h
    ll = math.sqrt(llsq)  # length
    # normalize line
    if ll <=0: return [0,0]
    for i in range(len(p1)):  # each dimension
        line[i] = line[i]/ll
    return line

def pointPointDistance(p1, p2):
    dist = ((p2[1]-p1[1])**2+(p2[0]-p1[0])**2)**0.5
    return dist

def closestPointToPoint(G,p2):
    dmin = 999999999
    for v in G[nodes]:
        p1 = vertices [ v ]
        d = pointPointDistance(p1,p2)
        if d <= dmin:
            dmin = d
            close = v
    return close


#used in intersect
def ccw(A,B,C):
    """ Determine if three points are listed in a counterclockwise order.
    For three points A, B and C. If the slope of the line AB is less than
    the slope of the line AC then the three points are in counterclockwise order.
    See:  http://compgeom.cs.uiuc.edu/~jeffe/teaching/373/notes/x06-sweepline.pdf
    """
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

#used in lineHitsRect
def intersect(A,B,C,D):
        """ do lines AB and CD intersect? """
        i = ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
        #if i:
        #    canvas.polyline(  [ A,B ], style=4  , tags = ("debug"))
        #    canvas.polyline(  [ C,D ], style=4  , tags = ("debug"))
        #else:
        #    canvas.polyline(  [ A,B ], style=1  , tags = ("debug")) # green
        #    canvas.polyline(  [ C,D ], style=1  , tags = ("debug"))
        return i


def lineHitsRect(p1,p2,r):
   rline = ( (r[0],r[1]), (r[0],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[0],r[1]), (r[2],r[1]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[0],r[3]), (r[2],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1
   rline = ( (r[2],r[1]), (r[2],r[3]) )
   if intersect( p1, p2, rline[0], rline[1] ): return 1

   return 0

def inRect(p,rect,dilation):
   """ Return 1 if p is inside rect, dilated by dilation (for edge cases). """
   if p[0]<rect[0]-dilation: return 0
   if p[1]<rect[1]-dilation: return 0
   if p[0]>rect[2]+dilation: return 0
   if p[1]>rect[3]+dilation: return 0
   return 1

def rrt_search(G, tx, ty, canvas):
    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    while 1:
        x_random = genPoint()
        if n%100 == 0: x_random = (tx,ty)
        v = closestPointToPoint(G,x_random)
        x_nearest = vertices[v]
        line_between = lineFromPoints(x_nearest,x_random)
        move_distance = [line_between[0] * SMALLSTEP, line_between[1] * SMALLSTEP]
        x_new = [x_nearest[0]+move_distance[0], x_nearest[1]+move_distance[1]]

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        in_rect = 0
        for o in obstacles :
            if inRect(x_new,o,1) or lineHitsRect(x_nearest,x_new,o):
                in_rect = 1

        if not in_rect:
            k = pointToVertex( x_new )   # is the new vertex ID
            G[nodes].append(k)
            G[edges].append( (v,k) )
            if visualize:
                canvas.polyline(  [x_nearest, x_new] )

            if pointPointDistance( x_new, [tx,ty] ) < SMALLSTEP:
                print ("Target achieved.", nsteps, "nodes in entire tree")
                if visualize:
                    t = pointToVertex([tx, ty])  # is the new vertex ID
                    G[edges].append((k, t))
                    if visualize:
                        canvas.polyline([x_new, vertices[t]], 1)
                    # while 1:
                    #     # backtrace and show the solution ...
                    #     canvas.events()
                    nsteps = 0
                    totaldist = 0
                    while 1:
                        oldp = vertices[k]  # remember point to compute distance
                        k = returnParent(k, canvas)  # follow links back to root.
                        canvas.events()
                        if k <= 1: break  # have we arrived?
                        nsteps = nsteps + 1  # count steps
                        totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                    print ("Path length", totaldist, "using", nsteps, "nodes.")

                    global prompt_before_next
                    if prompt_before_next:
                        canvas.events()
                        print("More [c,q,g,Y]>")
                        d = sys.stdin.readline().strip().lstrip()
                        print("[" + d + "]")
                        if d == "c": canvas.delete()
                        if d == "q": return
                        if d == "g": prompt_before_next = 0
                    break

def main():
    # seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(
            xmin=0, ymin=0, xmax=XMAX, ymax=YMAX, nrects=0, keepcontrol=0)  # , rescale=800/1800.)
        for o in obstacles:
            canvas.showRect(o, outline='red', fill='blue')
    while 1:
        # graph G
        redraw(canvas)
        G[edges].append((0, 1))
        G[nodes].append(1)
        if visualize:
            canvas.markit(tx, ty, r=SMALLSTEP)
        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':

    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y],
                [args.start_pos_x, args.start_pos_y + 10]]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y

    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
