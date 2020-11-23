#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import simpleguitk as simplegui
from PIL import Image

#Paths
userid = "divel2"
imagepath = "/home/" + userid + "/catkin_ws/src/teb_local_planner_tutorials/maps/Map_labo_300.png"
loadpointspath = "/home/" + userid + "/catkin_ws/src/navgraph_solver/navgraphs/points_Map_labo_300.csv"
loadadjpath = "/home/" + userid + "/catkin_ws/src/navgraph_solver/navgraphs/adjacency_Map_labo_300.csv"
savepointspath = "/home/" + userid + "/catkin_ws/src/navgraph_solver/navgraphs/points_gui.csv"
saveadjpath = "/home/" + userid + "/catkin_ws/src/navgraph_solver/navgraphs/adjacency_gui.csv"

# Constants
HEIGHT = 0
WIDTH = 0
SCALE = 0.05
NODE_SPACE_ALLOWANCE = 20
EDGE_COLOR = "Blue"
EDGE_SIZE = 2
NODE_LABEL_COLOR = "Black"
NODE_COLOR = "Red"
NODE_MARK_COLOR = "Green"

# Global variables
pub_string = "////"
start = 0
placeNodes = True
setNodesRelation = False
draw_relations = False
draw_mark_relations =  False
setGoal = False
setStart = False
displayResult = False
lock_nodes = False
nodes = []

pos1 = [0,0]
pos2 = [0,0]
pos_lock = False
indx = 0
letter_label_default = '@'
letter_pos = 1
current_node_letters_low = []
current_node_letters_up = []

image = simplegui.load_image(imagepath)

global mission_pub, input_start1, input_start2, input_start3, input_start4, input_start5

class Point:
    def __init__(self,pos,node_colour,node_mark_colour, station=False):
        self.pos = pos
        self.children = []
        self.radius = 5
        self.colour = node_colour
        self.mark_colour = node_mark_colour
        self.index = 0
        self.is_mark = False
        self.label = '@'
        self.station = station

    def draw(self,canvas):
        # if self.is_mark == False:
        canvas.draw_circle(self.pos, self.radius, 6, self.colour)
        # else:
            # canvas.draw_circle(self.pos, self.radius, 6, self.mark_colour)

def mouseclick(pos):
    global pos1, pos2, pos_lock, indx, draw_relations, draw_mark_relations, nodes, indx_mark_color
    global letter_label_default, letter_pos

    # Creates new instance of point(node) if the last position of
    # the mouseclick is not on  top of a previous node
    allow_place_node = True

    if placeNodes == True:
        if nodes: # Checks if the nodes are not empty
            for p, location in enumerate(nodes):
                if ((pos[0] >= (nodes[p].pos[0]-NODE_SPACE_ALLOWANCE) and pos[0] <= (nodes[p].pos[0]+NODE_SPACE_ALLOWANCE)) and
                    (pos[1] >= (nodes[p].pos[1]-NODE_SPACE_ALLOWANCE) and pos[1] <= (nodes[p].pos[1]+NODE_SPACE_ALLOWANCE))):
                    print "Warning: Cannot create node on top of another node!"
                    if(nodes[p].station):
                        # nodes[p].station = False
                        # nodes[p].colour = NODE_COLOR
                        print "Deleting Node"
                        del nodes[p]
                        for i in range(p, len(nodes)):
                            nodes[i]. label = chr(ord(nodes[i].label) - 1)
                        letter_pos -= 1
                    else:
                        print "Toggling Station/ Node"
                        nodes[p].station = True
                        nodes[p].colour = NODE_MARK_COLOR
                    allow_place_node = False
                    break
            # Creates new instance of Point class if no nodes detected in
            # the vicinity of mouseclick position
            if allow_place_node == True:
                nodes.append(Point(pos,NODE_COLOR,NODE_MARK_COLOR))
                nodes[-1].label = chr(ord(letter_label_default) + letter_pos)
                letter_pos += 1
        # Else creates a node for first time
        else:
            nodes.append(Point(pos,NODE_COLOR,NODE_MARK_COLOR))
            nodes[-1].label = chr(ord(letter_label_default) + letter_pos)
            letter_pos += 1

    # Sets up the edges or links
    if setNodesRelation == True:

        # If mouseclick pos is on top of a current node mark that node
        for i, position in enumerate(nodes):
            if ((pos[0] >= (nodes[i].pos[0]-NODE_SPACE_ALLOWANCE) and pos[0] <= (nodes[i].pos[0]+NODE_SPACE_ALLOWANCE)) and
                (pos[1] >= (nodes[i].pos[1]-NODE_SPACE_ALLOWANCE) and pos[1] <= (nodes[i].pos[1]+NODE_SPACE_ALLOWANCE))):
                if pos_lock == False:
                    pos1[0] = pos[0]
                    pos1[1] = pos[1]

                    indx = i
                    indx_mark_color = i
                    pos_lock = True
                    draw_mark_relations = True
                    break

                else:
                    # If it is the second node that is not the same of
                    # the first marked node, then creates a new relation/edge
                    if i != indx:
                        pos2[0] = pos[0]
                        pos2[1] = pos[1]
                        nodes[indx].children.append(i)
                        nodes[i].children.append(indx)

                        pos_lock = False
                        draw_relations = True
                        draw_mark_relations = False
                        break
                    else:
                        print "Warning: Recursion or self loop is not allowed."
                        print "Toggling Station"
                        pos_lock = False
                        draw_mark_relations = False
                        if(nodes[i].station):
                            nodes[i].station = False
                            nodes[i].colour = NODE_COLOR
                        else:
                            nodes[i].station = True
                            nodes[i].colour = NODE_MARK_COLOR
                        allow_place_node = False

def button_refresh_new_relation():
    global pos_lock, pos1, pos2, nodes, draw_relations, draw_mark_relations

    if lock_nodes == False and setNodesRelation == True:
        pos_lock = False
        draw_mark_relations = False
        draw_relations = False
        pos1[0] = 0
        pos1[1] = 0
        pos2[0] = 0
        pos2[1] = 0

        # This empties the list of children attribute of Point class
        for i, child in enumerate(nodes):
            del nodes[i].children[:]
    else:
        print "Warning: This action is not allowed."

def button_lock_nodes():
    global placeNodes, setNodesRelation, current_node_letters_up, nodes, current_node_letters_low

    # Can only lock nodes if the set-up is right
    # Prevents locking nodes later in the program
    if placeNodes == True and setNodesRelation == False and setStart == False and setGoal == False:
        placeNodes = False
        setNodesRelation = True

        # Fills two new lists of all node labels(letters)
        # for later use in input start and goal
        if nodes:
            for n, obj in enumerate(nodes):
                current_node_letters_up.append(nodes[n].label)

            for let in current_node_letters_up:
                current_node_letters_low.append(let.lower())

        print "The nodes are now locked in!"
    else:
        print "Warning: This action is not allowed."

def button_lock_graph():
    global placeNodes, setNodesRelation, nodes, lock_nodes

    if setNodesRelation is True:
        placeNodes = False
        setNodesRelation = False
        lock_nodes = True

        # Sets the index of nodes list and apply it to each index attribute of Point class
        # for index/element reference only, for later use in BFS and DFS functions
        g = open(saveadjpath,"w+")
        f = open(savepointspath,"w+")
        for d, dot in enumerate(nodes):
            station = 3
            if(dot.station):
                station = 4
            nodes[d].index = d
            print "node"+str(d+1)+":", nodes[d].label
            f.write("%f,%f,0,0,0,0,1,0,%d\r\n" % (float(dot.pos[0])*SCALE, (HEIGHT - float(dot.pos[1]))*SCALE, station))
            for k in nodes[d].children[:-1]:
                g.write("%d," % k)
            if nodes[d].children:
                g.write("%d\r\n" % nodes[d].children[-1])
            # This is important
            # This sorts the elements of children attribute list in ascending order
            nodes[d].children.sort()
        g.close()
        f.close()

        print "Graph is now set!"
    else:
        print "Warning: This action is not allowed."

def input_start_handler(start_string, station_id):
    global start, nodes, setStart, pub_string
    station_list = pub_string.split('/')
    setStart = False
    station_params = start_string.split(' ')
    pub_string = ""
    yaw = 90.0

    if(len(station_params) == 3 or len(station_params) == 4):
        if(station_params[1] == '0' or station_params[1] == '1'):
            if(station_params[2].isdigit()):
                if station_params[0].isdigit():
                    # Allows number as input for starting node
                    # 1 for A, 2 for B and so on and so forth
                    temp_start = int(station_params[0]) - 1
                    for element, num in enumerate(nodes):
                        if temp_start == element:

                            # Minus one because node label starts at 1 not zero(index)
                            start = temp_start
                            print "Station:", chr(start+65)
                            setStart = True
                            break
                    if setStart == False:
                        print "Warning: This number is outside of the nodes!"
                else:
                    # Allows letter as input for starting node
                    if station_params[0] in current_node_letters_up:
                        start = ord(station_params[0]) - 65
                        setStart = True
                        print "Station:", chr(start+65)
                    else:
                        if station_params[0] in current_node_letters_low:
                            start = ord(station_params[0]) - 97
                            setStart = True
                            print "Station:", chr(start+65)
                        else:
                            print "Warning: Unknown input. Enter a number or the node letter."
                if(len(station_params) == 4):
                    try:
                        yaw = float(station_params[3])
                        if(yaw > 180 or yaw < -180):
                            print "Enter yaw value between 0-360"
                            yaw = 90.0
                    except ValueError:
                        print "Enter Float Yaw value"
                        yaw = 90.0

            else:
                print "Enter integer task ID"
        else:
            print "Invalid Stay boolean, enter 0 or 1"
    else:
        print "Input format: 'Station<char/int> Stay<boolean> Task<int> Yaw<float>"

    if(setStart):
        data = str(start) + " " + station_params[1] + " " + station_params[2] + " " + str(yaw)
        station_list[station_id - 1] = data
        print "Recorded Data " + data
    else:
        station_list[station_id - 1] = ""
    pub_string = station_list[0] + '/' + station_list[1] + '/' + station_list[2] + '/' + station_list[3] + '/' + station_list[4]

def button_breadth_first_search():
    global nodes, displayResult, result_string, queue_string, pointer_string
    displayResult = False
    pointer_string = ""

    # Resets all nodes markings (color)
    for d, marking_obj in enumerate(nodes):
        nodes[d].is_mark = False

    in_queue_result = False

    if placeNodes == False and setNodesRelation == False and setStart == True and setGoal == True:
        print " "
        print "BFS starts here:"

        # Checks queue if defined,
        # if it is, then go to else and empty the list; otherwise create a new list
        try:
            queue
        except:
            queue = []
        else:
            del queue[:]

        queue.append(nodes[start])
        queue[0].is_mark = True

        try:
            result
        except:
            result = []
        else:
            del result[:]

        while queue:
            pointer = queue[0]
            queue.pop(0)

            pointer.is_mark = True
            print " "
            print "Pointer:", pointer.label

            if pointer.index == goal:
                pointer_string =  "Pointer: " + pointer.label
                result_string = "Result: "
                queue_string = "Queue: "

                for obj in result:
                    result_string += str(obj.label)
                    result_string += " "
                for objt in queue:
                    queue_string += str(objt.label)
                    queue_string += " "

                displayResult = True
                print "SUCCESS!"
                break
            else:
                result.append(pointer)

                for neighbor in pointer.children:
                    in_queue_result = False
                    for i in queue:
                        #print "neighbor:", neighbor+1, "queue:", i.index+1
                        if neighbor == i.index:
                            in_queue_result = True

                    for j in result:
                        #print "neighbor:", neighbor+1, "result:", j.index+1
                        if neighbor == j.index:
                            in_queue_result = True

                    if in_queue_result == False:
                        for objct in nodes:
                            if objct.index == neighbor:
                                queue.append(nodes[objct.index])
            result_string = "Result: "
            queue_string = "Queue: "
            for obj in result:
                result_string += str(obj.label)
                result_string += " "
            print result_string

            for objt in queue:
                queue_string += str(objt.label)
                queue_string += " "
            print queue_string

def draw_handler(canvas):
    global result_string, queue_string, pointer_string
    global placeNodes, setNodesRelation, setStart, setGoal, pos1

    canvas.draw_image(image, (WIDTH/2,HEIGHT/2), (WIDTH,HEIGHT), (WIDTH/2,HEIGHT/2), (WIDTH,HEIGHT))
    # Draws nodes
    if draw_mark_relations == True and setNodesRelation == True:
        canvas.draw_circle(nodes[indx_mark_color].pos, 15, 3, "Yellow", "Black")

    if nodes:
        for i, vertex in enumerate(nodes):
            nodes[i].draw(canvas)
            canvas.draw_text(nodes[i].label, (nodes[i].pos[0]-30, nodes[i].pos[1]), 20, NODE_LABEL_COLOR)

    # Draws edges
    if draw_relations == True:
        for n, point in enumerate(nodes):
            if nodes[n].children:
                for child in nodes[n].children:
                    canvas.draw_line(nodes[n].pos, nodes[child].pos, EDGE_SIZE, EDGE_COLOR)

    # Display results
    if displayResult == True:
        canvas.draw_text(pointer_string, (30, 345), 15, NODE_LABEL_COLOR)
        canvas.draw_text(result_string, (30, 370), 15, NODE_LABEL_COLOR)
        canvas.draw_text(queue_string, (30, 395), 15, NODE_LABEL_COLOR)

def input1(start_string):
    input_start_handler(start_string, 1)

def input2(start_string):
    input_start_handler(start_string, 2)

def input3(start_string):
    input_start_handler(start_string, 3)

def input4(start_string):
    input_start_handler(start_string, 4)

def input5(start_string):
    input_start_handler(start_string, 5)

def publisher():
    global pub_string, mission_pub
    if pub_string:
        mission_pub.publish(pub_string)

def load_nodes():
    global nodes
    global letter_label_default, letter_pos
    f = open(loadpointspath, "r")
    for line in f:
        data = line.rstrip().split(',')
        x = int(float(data[0])/SCALE)
        y = HEIGHT - int(float(data[1])/SCALE)
        station = int(data[8])
        if(station == 4):
            nodes.append(Point([x, y],NODE_MARK_COLOR,NODE_MARK_COLOR, True))
        else:
            nodes.append(Point([x, y],NODE_COLOR,NODE_MARK_COLOR))
        nodes[-1].label = chr(ord(letter_label_default) + letter_pos)
        letter_pos += 1

def load_adjacency():
    global nodes, draw_relations
    f = open(loadadjpath, "r")
    k = 0
    draw_relations = True
    for line in f:
        data = line.rstrip().split(',')
        for child in data:
            if(child):
                nodes[k].children.append(int(child))
        k += 1

def create_gui():
    global mission_pub, input_start1, input_start2, input_start3, input_start4, input_start5
    global HEIGHT, WIDTH

    # Creates the frame window
    im = Image.open(imagepath)
    WIDTH, HEIGHT = im.size

    frame = simplegui.create_frame("Graph Designer and Publisher",WIDTH,HEIGHT)

    frame.set_mouseclick_handler(mouseclick)
    frame.set_draw_handler(draw_handler)

    # Button, input and label controls for the frame window
    button4 = frame.add_button('Load nodes', load_nodes)
    button5 = frame.add_button('Load adjacency', load_adjacency)
    button1 = frame.add_button('Lock in the nodes', button_lock_nodes)
    button2 = frame.add_button('Lock in the graph', button_lock_graph)
    button3 = frame.add_button('Reset edge drawing', button_refresh_new_relation)
    label0 = frame.add_label(' ')
    label1 = frame.add_label('Task 1: Lift Block\nTask 2: Charge Bot\nTask 3: Drop Block')
    label12= frame.add_label('Input format: \'Station<char/int> Stay<boolean> Task<int> Yaw<float>\'')
    input_start1 = frame.add_input('Station 1', input1, 50)
    input_start2 = frame.add_input('Station 2', input2, 50)
    input_start3 = frame.add_input('Station 3', input3, 50)
    input_start4 = frame.add_input('Station 4', input4, 50)
    input_start5 = frame.add_input('Station 5', input5, 50)

    button4 = frame.add_button('Publish Stations', publisher)

    mission_pub = rospy.Publisher('mission', String, queue_size=10)
    rospy.init_node('gui', anonymous=True)

    # Program starts here
    frame.start()

if __name__ == '__main__':
    try:
        create_gui()
    except rospy.ROSInterruptException:
        pass
