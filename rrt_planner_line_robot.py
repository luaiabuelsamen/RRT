from dis import dis
import random
import drawSample
import sys
import imageToRects
import utils
import math
import matplotlib.pyplot as plt

def redraw(canvas):
    canvas.clear()
    canvas.markit(tx, ty, r=SMALLSTEP)
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")

def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
        # e.g. vertices: [[10, 270], [10, 280]]
        canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


# Use this function to generate points randomly for the RRT algo
def genPoint():
    # if args.rrt_sampling_policy == "uniform":
    #     # Uniform distribution
    #     x = random.random()*XMAX
    #     y = random.random()*YMAX
    # elif args.rrt_sampling_policy == "gaussian":
    #     # Gaussian with mean at the goal
    #     x = random.gauss(tx, sigmax_for_randgen)
    #     y = random.gauss(ty, sigmay_for_randgen)
    # else:
    #     print ("Not yet implemented")
    #     quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
            theta = random.random()*math.pi
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
            theta = random.gauss(ty, math.pi/2)
        else:
            print ("Not yet implemented")
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if theta<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1

    return [x,y,theta]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            #replaced to display a line robot
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append(p)
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    dist = pointPointDistance(p1, p2)
    line = [(p2[0] - p1[0])/dist, (p2[1] - p1[1])/dist]
    return line

def pointPointDistance(p1, p2):
    dist = ((p2[1]-p1[1])**2+(p2[0]-p1[0])**2)**0.5
    return dist

def closestPointToPoint(G, p2):
    nNear = 0
    minDist = pointPointDistance(vertices[0], p2)
    for node in G[nodes]:
        dist = pointPointDistance(vertices[node], p2)
        if dist <= minDist:
            minDist = dist
            nNear = node
    return nNear

#return true if slope of p3p1 > p2p1
def checkSlope(p1,p2,p3):
    return (p3[1]-p1[1])*(p2[0]-p1[0]) > (p2[1]-p1[1])*(p3[0]-p1[0])

# Return true if line segments p1p2 and p3p4 intersect
def intersect(p1,p2,p3,p4):
        return checkSlope(p1,p3,p4) != checkSlope(p2,p3,p4) and checkSlope(p1,p2,p3) != checkSlope(p1,p2,p4)

def lineHitsRect(p1, p2, r):
    i1 = intersect(p1,p2,(r[0],r[3]),(r[0],r[1]))
    i2 = intersect(p1,p2,(r[2],r[3]),(r[2],r[1]))
    i3 = intersect(p1,p2,(r[0],r[1]),(r[2],r[1]))
    i4 = intersect(p1,p2,(r[2],r[3]),(r[0],r[3]))
    if(i1 or i2 or i3 or i4):
        return 1
    return 0

def inRect(p,rect,dilation):
   """ Return 1 if p is inside rect, dilated by dilation (for edge cases). """
   if p[0]<rect[0]-dilation: return 0
   if p[1]<rect[1]-dilation: return 0
   if p[0]>rect[2]+dilation: return 0
   if p[1]>rect[3]+dilation: return 0
   return 1

def addNewPoint(p1, p2, stepsize, stepangle):
    x, y, theta = p2

    rot = p2[2] - p1[2]
    dist = pointPointDistance(p1,p2)
    if rot > 0:
        if rot >= stepangle:
            theta = theta + stepangle
    else:
        theta = theta - stepangle
    if dist > stepsize:
        x = p1[0] + stepsize*(p2[0]-p1[0])/dist
        y = p1[1] + stepsize*(p2[1]-p1[1])/dist
    return [x,y,theta]

def robotEnds(x):
    p1 = [x[0] + int(ROBOTLENGTH/2*math.cos(x[2])), x[1] + int(ROBOTLENGTH/2*math.sin(x[2]))]
    p2 = [x[0] - int(ROBOTLENGTH/2*math.cos(x[2])), x[1] - int(ROBOTLENGTH/2*math.sin(x[2]))]

    return p1, p2

def rrt_search(G, tx, ty, tt, canvas):
    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    niters=0
    nsteps=0
    while 1:
        randp = genPoint()
#        if n%100 == 0: randp = (tx,ty,tt)
        v = closestPointToPoint(G,randp)
        nearp = vertices[v]
        nextp = addNewPoint(nearp, randp, SMALLSTEP, SMALLANGLE)

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0
        nearRobot = robotEnds(nearp)
        nextRobot = robotEnds(nextp)

        freeSpace = 1
        for o in obstacles :
            #check if ends or line robot is in obstacle
            if lineHitsRect(nextRobot[0],nextRobot[1],o) or inRect(nextRobot[0],o,1) or inRect(nextRobot[1],o,1):
                freeSpace = 0

        if freeSpace:
            nsteps += 1
            k = pointToVertex( nextp )   # is the new vertex ID
            G[nodes].append(k)
            G[edges].append( (v,k) )
            if visualize:
                canvas.polyline(  [nextRobot[0], nextRobot[1]] )
            
            #check if ends or mid is at target
            if (pointPointDistance( nextp, [tx,ty] ) < SMALLSTEP) or (pointPointDistance( nextRobot[0], [tx,ty] ) < SMALLSTEP) or (pointPointDistance( nextRobot[1], [tx,ty] ) < SMALLSTEP):
                print ("Target achieved.", nsteps, "nodes in entire tree")
                if visualize:
                    t = pointToVertex([tx, ty])  # is the new vertex ID
                    G[edges].append((k, t))
                    if visualize:
                        canvas.polyline([nextp, vertices[t]], 1)
                        #show final line robot
                        canvas.polyline(  [nextRobot[0], nextRobot[1]],style=3  )
                    # while 1:
                    #     # backtrace and show the solution ...
                    #     canvas.events()
                    nsteps = 0
                    totaldist = 0
                    while 1:
                        oldp = vertices[k]  # remember point to compute distance
                        k = returnParent(k, canvas)  # follow links back to root.
                        #show line robot at parent
                        if visualize: canvas.polyline(  [robotEnds(vertices[k])[0], robotEnds(vertices[k])[1] ], style=3  )
                        if visualize: canvas.events()
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
        niters += 1

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
            canvas.markit(args.start_pos_x, args.start_pos_y, r=SMALLSTEP)
        drawGraph(G, canvas)
        rrt_search(G, tx, ty, tt, canvas)

    if visualize:
        canvas.mainloop()


if __name__ == '__main__':

    args = utils.get_args()
    visualize = utils.get_args()
    drawInterval = 10  # 10 is good for normal real-time drawing

    prompt_before_next = 1  # ask before re-running sonce solved
    SMALLSTEP = args.step_size  # what our "local planner" can handle.
    SMALLANGLE = math.pi/10
    ROBOTLENGTH = args.robot_length
    map_size, obstacles = imageToRects.imageToRects(args.world)
    # Note the obstacles are the two corner points of a rectangle (left-top, right-bottom)
    # Each obstacle is (x1,y1), (x2,y2), making for 4 points

    XMAX = map_size[0]
    YMAX = map_size[1]
    # The boundaries of the world are (0,0) and (XMAX,YMAX)

    G = [[0], []]  # nodes, edges
    vertices = [[args.start_pos_x, args.start_pos_y, args.start_pos_theta],
                [args.start_pos_x, args.start_pos_y + 10, args.start_pos_theta]]

    # goal/target
    tx = args.target_pos_x
    ty = args.target_pos_y
    tt = args.target_pos_theta
    # start
    sigmax_for_randgen = XMAX / 2.0
    sigmay_for_randgen = YMAX / 2.0
    nodes = 0
    edges = 1

    main()
    
    #code written to graph
    if 0:
        lens = [i for i in range(5,51,9)]
        iters = []
        for ROBOTLENGTH in lens:
            itern = 0
            for i in range(10):
                print(i)
                canvas = None
                G[edges].append((0, 1))
                G[nodes].append(1)
                itern += rrt_search(G, tx, ty, tt, canvas)
            iters.append(itern/10)

        print(lens, iters)
        plt.bar(lens, iters, color ='maroon', width = 4)
    
        plt.xlabel("Robot Size")
        plt.ylabel("Average iterations for 10 runs")
        plt.title("Varying  Robot size effect on iterations")
        plt.show()        

