##---------------------------------------------------------------------------##
# THIS FILE CONTAINS THE ENTIRE PROGRAM
##---------------------------------------------------------------------------##
# All operations on graph object are done assuming the graph is a dictionary 
# of dictionaries containing node names and connections to each node
#
# If you encounter set() or get() functions, these are GUI variable class methods
#used for getting and setting inputs and outputs respectively.
#
# Back-end Functions are defined first followed by the main GUI program.
#
#
#
#-----------------------------------------------------------------------------#

#----------------------------IMPORT REQUIRED MODULES--------------------------#

from tkinter import *           # For GUI
from tkinter import filedialog  # for File Browser
import numpy as np              # Array operations
import pandas as pd             # Data Frames
import string                   # String Operations
import networkx as nx           # Visualization
import matplotlib.pyplot as plt # Plotting
# Matplotlib interface with tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure
import copy

#-----------------------------------------------------------------------------#

#----------------------------DEFINE GLOBAL VARIABLES--------------------------#

# Any global variables are accessible throughout the program
global distances,parent,routes
global node_name
global start_node,end_node,node_weight
global src,dest


#-----------------------------------------------------------------------------#

#----------------------------CREATE MAIN WINDOW-------------------------------#

root = Tk()
root.title("CS542_04_16F:Project: Network Simulator")



#-----------------------------------------------------------------------------#

#---------------------------BROWSE THE NETWORK FILE--------------------------#

def open_file():
    global file
    file = filedialog.askopenfilename()
    file_name_var.set(file)
    
    
#-----------------------------------------------------------------------------#

#-----------------------READ THE NETWORK FILE AND DISPLAY---------------------#

def read_data(file_name):
	file_name =file_name_var.get()
	global data
	global adj_mat
    # Use a pandas function to read the text file
    # This is automatically saved as a data frame
	data = pd.read_table(file_name,sep=" ",header=None)
    # Convert data frame into numpy array
	adj_mat = np.array(data)
	mat2dict(data,adj_mat)
    
    
#-----------------------------------------------------------------------------#

#--------------------CONVERT ADJACENCY MATRIX TO DICTIONARY-------------------#

def mat2dict(data,adj_mat):
    # The Columns are given alphabetical names starting from A
    data.columns=list(map(chr,range(65,66+data.columns[-1])))
    data.columns.names = ["Routers"]
    data.index.names = ["Connections"]
    # The Data frame is saved to a dictionary with keys as the router names
    # The values are dictionaries themselves with keys as router names and
    # values as connection strengths
    global graph,recover_graph
    graph={}
    for i,j in zip(data.columns,range(0,adj_mat.shape[0])):
        x = adj_mat[j,:]
        subg={}
        for k,l in zip(data.columns,range(0,adj_mat.shape[0])):
            subg[k] = x[l]
        graph[i]=subg
    # This graph has entries with self connections and no connections as well
    # We delete the entries with either 0 or -1
    for k in list(graph.keys()):
        for x,y in zip(list(graph[k].keys()),list(graph[k].values())):
            if y==0 or y==-1:
                del graph[k][x]
    
    # Create a recovery copy of graph            
    recover_graph = copy.deepcopy(graph)
    # Display data read.
    matrixDispLabel.configure(text='%s'%(data))
    
    # Update Text Fields in GUI (See Spinbox Definitions)
    nodeEntry.configure(values=tuple(sorted(graph.keys())))
    sourceEntry.configure(values=tuple(sorted(graph.keys())))
    destinationEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry1.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry1.configure(values=tuple(sorted(graph.keys())))
    deleteNodeEntry.configure(values=tuple(sorted(graph.keys())))
    
    
#-----------------------------------------------------------------------------#

#------------------------------DRAW TOPOLOGY GRAPH----------------------------#                

# Uses Networkx to visualize graph on screen
def visualize(graph):
    # Convert graph to networkx compatible format
    MG = nx.MultiDiGraph()
    for keys in graph.keys():           
        for subkeys in graph[keys].keys():
            MG.add_edge(keys,subkeys,w=graph[keys][subkeys])
    edge_weight=dict([((u,v,),int(d['w'])) for u,v,d in MG.edges(data=True)])
    # 'a' is the figure name on which to plot
    a.clear()
    # draw graph
    pos=nx.spring_layout(MG)
    nx.draw_networkx_edge_labels(MG,pos,edge_labels=edge_weight,ax=a)
    nx.draw_networkx_nodes(MG,pos,ax=a)
    nx.draw_networkx_edges(MG,pos,ax=a)
    nx.draw_networkx_labels(MG,pos,ax=a)

    
#-----------------------------------------------------------------------------#

#-------------------------------ADD/MODIFY WEIGHT-----------------------------#

#Modifies as well as adds directed connections
def mod_weight(graph):
    start_node = start_node_var.get()
    end_node = end_node_var.get()
    node_weight = weight_var.get()
    graph[start_node][end_node]=node_weight
    recover_graph[start_node][end_node]=node_weight
#-----------------------------------------------------------------------------#

#------------------------------DELETE WEIGHT/EDGE-----------------------------#

# Deletes a directed edge between two nodes
def del_edge(graph):
    start_node = start_node_var1.get()
    end_node = end_node_var1.get()
    del graph[start_node][end_node]
    del recover_graph[start_node][end_node]
	


#-----------------------------------------------------------------------------#

#--------------------------------ADD NEW NODE---------------------------------#

def add_node(graph):
    node_name = chr(ord(sorted(recover_graph.keys())[-1])+1)
    graph[node_name]={}
    recover_graph[node_name]={}
    
    # Following label will display which node will be added to graph
    addNodeLabel.configure(text='%s node added.'%(node_name))
    
    # Update Text Fields in GUI (See Spinbox Definitions)
    nodeEntry.configure(values=tuple(sorted(graph.keys())))
    sourceEntry.configure(values=tuple(sorted(graph.keys())))
    destinationEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry1.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry1.configure(values=tuple(sorted(graph.keys())))  
    deleteNodeEntry.configure(values=tuple(sorted(graph.keys())))
    

#-----------------------------------------------------------------------------#

#-------------------------------REMOVE THE NODE-------------------------------#
  
def remove_node(graph):
    del_node = delete_node_var.get()
    for keys in list(graph.keys()):
        for subkeys in list(graph[keys].keys()):
            if subkeys==del_node:
                del graph[keys][subkeys]
    del graph[del_node]
    
    # Update Text Fields in GUI (See Spinbox Definitions)
    nodeEntry.configure(values=tuple(sorted(graph.keys())))
    sourceEntry.configure(values=tuple(sorted(graph.keys())))
    destinationEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry1.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry1.configure(values=tuple(sorted(graph.keys())))  
    deleteNodeEntry.configure(values=tuple(sorted(graph.keys())))
    
    
#-----------------------------------------------------------------------------#

#--------------------------------RECOVER THE NODE-----------------------------#

def recover_node():
    for keys in recover_graph.keys():
        if keys not in graph:
            graph[keys]=recover_graph[keys]

    for keys in recover_graph.keys():
        for subkeys in recover_graph[keys].keys():
            if subkeys not in graph[keys]:
                graph[keys][subkeys]=recover_graph[keys][subkeys]

    # Update Text Fields in GUI (See Spinbox Definitions)
    nodeEntry.configure(values=tuple(sorted(graph.keys())))
    sourceEntry.configure(values=tuple(sorted(graph.keys())))
    destinationEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry.configure(values=tuple(sorted(graph.keys())))
    startNodeEntry1.configure(values=tuple(sorted(graph.keys())))
    endNodeEntry1.configure(values=tuple(sorted(graph.keys())))  
    deleteNodeEntry.configure(values=tuple(sorted(graph.keys())))
    
    
#-----------------------------------------------------------------------------#




#-----------------------------------------------------------------------------#

#-------------------------------FIND INTERFACE--------------------------------#

# Assigns interface number to each outgoing link of each node.
def node_interface(graph):
    global interface
    interface = copy.deepcopy(graph)
    for keys in interface.keys():
        for i,j in zip(interface[keys],range(0,len(interface[keys]))):
            interface[keys][i]=j
    process_dijkstra()
    

#-----------------------------------------------------------------------------#

#--------------------------PATH FINDING FUNCTION------------------------------#

# Run's the Dijkstra's algorithm for entire graph
def process_dijkstra():
    global routes,all_distances
    global all_path
    routes={}
    all_distances={}
    all_path={}
    for nodes in graph.keys():
        # check for disjoint node
        if len(graph[nodes])==0:
            continue
        # run Dijkstra's algorithm
        distances,parent = dijkstra(graph,nodes)
        all_distances[nodes]=distances
        # find shortest path
        path = shortest_path(nodes,parent,distances)
        # create routing table
        routes[nodes]= routing(path,nodes,interface)
        all_path[nodes]=path
    # Created label for Djikstra's run Successfull
    DJKLabel.configure(text="Djikstra's Successful")
    

#-----------------------------------------------------------------------------#

#----------------------------DIJIKSTRA'S FUNCTION-----------------------------#

# Finds shortest path of start node to all other nodes
def dijkstra(graph,start):
    # Create a list of nodes
    nodes = sorted(graph.keys())
    
    # Dictionary to store name and distance of nodes which haven't been visited yet
    unvisited_set = {} 
    
    for node in graph.keys(): 
        # Initialize distances of all nodes to infinity
        unvisited_set[node]=np.inf
    
    # Dictionary to store all nodes which have been visited along with distances
    visited_set = {} 
    
    # Start node
    current_node = start 
    
    # Distance to start node is zero
    current_distance = 0
    
    # Update distance of start
    unvisited_set[current_node] = current_distance
    
    # Stores parent information
    parent = {}
    for keys in graph.keys():
        parent[keys]=None

    # Iterate through all unvisited sets till it's empty
    while unvisited_set:
        
        # Extract adjacent nodes and distances
        # The neighbour and distance will be an iterable list
        for neighbour, distance in graph[current_node].items():
            # Check whether neighbour is already visited
            if neighbour not in unvisited_set: 
                continue
            update = current_distance + distance
            if unvisited_set[neighbour] > update:
                #Update to lower distance
                unvisited_set[neighbour] = update
                parent[neighbour]=current_node
                
                
        # Add node and distance to visited set
        visited_set[current_node] = current_distance 
        
        # Pop node (key) from unvisited set
        unvisited_set.pop(current_node)
        # Exit if unvisited set is empty
        if not unvisited_set: break
            
        # Store unvisited set in a list for sorting
        adjacent = []
        for node in unvisited_set.items():
            adjacent.append(node)
        #print(adjacent)
        # Update current node and its distance
        current_node, current_distance = sorted(adjacent,
                                                key = lambda x: x[1])[0]
        # lambda is an anonymous function which can be defined inline
        # lambda passes x as the argument
        # the syntax is labda x:<do stuff with x>
        # Above, lambda selects distance from adjacent as the key for sorting
        # So sorting is performed on the basis of distance
        # Then the first element is selected (minimum)
        # This is assigned to the current node and distance
        #print(current_node,current_distance)
    # Return the final distance dictionary as well as parents of each node
    return visited_set,parent


#-----------------------------------------------------------------------------#

#---------------------------SHORTEST PATH FUNCTION----------------------------#

# Finds shortest path from start node to end node by iterating through the
# parent variable to link source and end. Finds paths for all nodes.
def shortest_path(src,parent,distances):
    Path_cumu=[]
    for keys in parent:
        dest = keys
        Path = []
        while 1:
            Path.append(dest)
            if dest == src:
                break
            if parent[keys]==None:
                break
            dest = parent[dest]
        Path.reverse()
        Path_cumu.append(Path)
    # return a list of lists containing paths to each node
    return Path_cumu


#-----------------------------------------------------------------------------#

#------------------------FUNCTION FOR ROUTING TABLE---------------------------#

# Returns routing table of node using path returned by dijkstra's algorithm
def routing(Path,src,interface):
    route = []
    for path in Path:
        dest = path[-1]
        if dest==src:
            continue
        # check if node is disjoint
        elif bool(interface[path[-1]])==False:
            continue
        else:
            route.append([dest,path[1],interface[src][path[1]]])
    return route


#-----------------------------------------------------------------------------#

#-----------------------SINGLE SHORTEST PATH FUNCTION-------------------------#

# Finds shortest path from start node to end node as well as the distance
# Simply extracts data and prints
def single_path(all_path,all_distances):
    src = source_var.get()
    dest = destination_var.get()
    dist = all_distances[src][dest]
    path_list = all_path[src]
    path = []
    for i in path_list:
        if i[-1] == dest:
            path = i
            
    # Following label will display the shortest path from source to destination
    pathLabel.configure(text='Shortest path is: %s'%(path),background="white")
    
    # Following label will display the total distance from source to destination
    distLabel.configure(text='Total Distance is: %s'%(dist),background="white")
    

#-----------------------------------------------------------------------------#

#------------------------------ALL SHORTEST PATH------------------------------#

# Find all shortest paths in terms of nodes traversed and distances from a 
# start node to the end node
def find_all_paths(graph):
    src = source_var.get()
    dest = destination_var.get()
    MG = nx.MultiDiGraph()                          
    for keys in graph.keys():           
        for subkeys in graph[keys].keys():
            MG.add_edge(keys,subkeys,weight=graph[keys][subkeys])
    paths = list(nx.all_shortest_paths(MG,src,dest))
    dist =[]
    for a_path,index in zip(paths,range(0,len(paths))):
        d=0
        for i in range(0,len(a_path)):
            if i<(len(a_path)-1):
                d=d+graph[a_path[i]][a_path[i+1]]
        dist.append(d)

    # Following label will display the all shortest path from source to destination
    pathLabel1.configure(text='All Shortest paths are: \n%s'%(paths),background="white")
    
    # Following label will display the total distance from source to destination for all path
    distLabel1.configure(text='Total Distances are: %s'%(dist),background="white")


#-----------------------------------------------------------------------------#

#--------------------FUNCTION TO DISPLAY ROUTING TABLE -----------------------#
    
# Creates Routing Table of all nodes
def create_routing_table(routes):
    global route_node
    route_node = node_var.get()
    global routing_table
    routing_table ={}
    columns = ["Destination","Send To","Interface"]
    for keys in routes.keys():
        routing_table[keys]=pd.DataFrame(routes[keys],columns=columns)
        
    # Following label will display routing table
    if route_node in routing_table:
        routingTableLabel.configure(text="%s"%(routing_table[route_node]),background="white")
    else:
        routingTableLabel.configure(text="%s has no routing table as it is disjoint"%(route_node),background="white")






#-----------------------------------------------------------------------------#

#----------DEFINE INPUT VARIABLE CLASSES FOR FRONTEND/BACKEND INTERFACE-------#

#created classes for GUI variables
file_name_var = StringVar()
start_node_var = StringVar()
end_node_var = StringVar()
weight_var = IntVar()
start_node_var1 = StringVar()
end_node_var1 = StringVar()
source_var = StringVar()
destination_var = StringVar()
node_var = StringVar()
delete_node_var = StringVar()


#-----------------------------------------------------------------------------#

#------------------CREATE MAIN FRAME INSIDE ROOT WINDOW-----------------------#

frame = Frame(root,width="1000",height="600")
frame.grid()

#-----------------------------------------------------------------------------#

#---------------------WIDGETS USED IN MAIN FRAME ROOT-------------------------#

#-----------------------------------------------------------------------------#

#----------------CREATE SUBFRAME IN ROOT FOR ALL PROCESSES--------------------#

frame1 = LabelFrame(frame,width=800,height=600)
frame1.grid(row=0,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#--------------------WIDGETS CREATED IN SUB FRAME FRAME1----------------------#

#-----------------------------------------------------------------------------#

#--------------------CREATE SUBFRAME IN FRAME1 FOR FILE-----------------------#

dataFileSubFrame =Frame(frame1)
dataFileSubFrame.grid(row=0,column=0)

#-----------------------------------------------------------------------------#

#---------------WIDGETS CREATED IN SUB FRAME dataFileSubFrame-----------------#

#-----------------------------------------------------------------------------#

#-----------------------CREATE LABEL FOR BROWSE FILE--------------------------#

fileNameLabel = Label(dataFileSubFrame,text="Browse Data File",font=("Arial", "10", "bold"))
fileNameLabel.grid(row=0,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#-------------------CREATE ENTRY FOR GETTING PATH OF FILE---------------------#

fileNameEntry = Entry(dataFileSubFrame,bd=2,width=75,textvariable=file_name_var)
fileNameEntry.grid(row=0,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#-----------------------CREATE BUTTON FOR SELECT PATH-------------------------#

browseButton=Button(dataFileSubFrame, text ="Browse",relief=RAISED,command=open_file)
browseButton.grid(row=0,column=2,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------------CREATE BUTTON FOR DISPLAY FILE CONTENT---------------------#

okButton = Button(dataFileSubFrame, text ="Ok",relief=RAISED,command=lambda: read_data(file))
okButton.grid(row=0,column=3,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------END OF WIDGETS CREATED IN dataFileSubFrame-------------------#

#-----------------------------------------------------------------------------#

#------------------CREATE SUB FRAME IN FRAME1 FOR PROCESS---------------------#

processSubFrame = Frame(frame1)
processSubFrame.grid(row=1,column=0)

#-----------------------------------------------------------------------------#

#---------------WIDGETS CREATED IN SUB FRAME processSubFrame------------------#

#-----------------------------------------------------------------------------#

#--------------------CREATE LABEL FOR ADJACENCY MATRIX------------------------#

matrixLabel = Label(processSubFrame,text="Adjacency Matrix:",font=("Arial", "10", "bold"))
matrixLabel.grid(row=0,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------------------CREATE LABEL FOR PROCESSES---------------------------#

processLabel = Label(processSubFrame,text="Processes:",font=("Arial", "10", "bold"))
processLabel.grid(row=1,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#--------------------CREATE SUB FRAME IN processSubFrame----------------------#

matrixDispFrame = LabelFrame(processSubFrame)
matrixDispFrame.grid(row=0,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE LABEL FOR MATRIX DISPLAY------------------------#

matrixDispLabel =Label(matrixDispFrame)
matrixDispLabel.grid(row=0,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#--------------------CREATE BUTTON FOR ROUTING ALGORITHM----------------------#

DJKButton = Button(processSubFrame, text ="Run Routing",relief=RAISED,command=lambda: node_interface(graph))
DJKButton.grid(row=1,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------------CREATE LABEL FOR OUTPUT FOR ALGORITHM----------------------#

DJKLabel = Label(processSubFrame)
DJKLabel.grid(row=1,column=2,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#---------------------CREATE BUTTON FOR TOPOLOGY GRAPH------------------------#

visualizeButton = Button(processSubFrame,text="Visualize Graph",relief=RAISED,command=lambda: visualize(graph))
visualizeButton.grid(row=1,column=3,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE LABEL FOR ROUTING TABLE-------------------------#

routingTableLabel = Label(processSubFrame, text ="Routing table:",font=("Arial", "10", "bold"))
routingTableLabel.grid(row=2,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------CREATE LABEL AND SPINBOX FOR INPUT OF ROUTING TABLE--------------#

nodeLabel = Label(processSubFrame, text ="Enter Node:")
nodeLabel.grid(row=3,column=0,padx=10,pady=10)

nodeEntry = Spinbox(processSubFrame,textvariable=node_var)
nodeEntry.grid(row=3,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE BUTTON FOR ROUTING TABLE------------------------#

routingTableButton = Button(processSubFrame, text ="Submit",relief=RAISED,command=lambda: create_routing_table(routes))
routingTableButton.grid(row=3,column=2,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE LABEL FOR SHORTEST PATH-------------------------#

shortestPathLabel = Label(processSubFrame, text ="Shortest Path:",font=("Arial", "10", "bold"))
shortestPathLabel.grid(row=4,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------CREATE LABEL AND SPINBOX FOR INPUT OF SHORTEST PATH--------------#

# Created label for "Enter Source" for finding short
sourceLabel=Label(processSubFrame,text="Enter Source:")
sourceLabel.grid(row=5,column=0,padx=10,pady=10)

# Following entry will take name of source node from user for finding shortest path
sourceEntry = Spinbox(processSubFrame,textvariable=source_var)
sourceEntry.grid(row=5,column=1,padx=10,pady=10)

# Created label for "Enter Destination" for finding short path
destinationLabel=Label(processSubFrame,text="Enter Destination:")
destinationLabel.grid(row=5,column=2,padx=10,pady=10)

# Following entry will take name of Destination node from user for finding shortest path
destinationEntry = Spinbox(processSubFrame,textvariable=destination_var)
destinationEntry.grid(row=5,column=3,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------------CREATE BUTTON FOR SINGLE SHORTEST PATH---------------------#

singleShortPathButton = Button(processSubFrame, text ="Single Shortest Path",relief=RAISED,command=lambda: single_path(all_path,all_distances))
singleShortPathButton.grid(row=5,column=4,padx=10,pady=10)
   
#-----------------------------------------------------------------------------#

#--------------------CREATE BUTTON FOR ALL SHORTEST PATH----------------------#   

allShortPathButton = Button(processSubFrame, text ="All Shortest Paths",relief=RAISED,command=lambda: find_all_paths(graph))
allShortPathButton.grid(row=5,column=5,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE LABEL ADD/MODIFY WEIGHT-------------------------#

modifyLabel = Label(processSubFrame,text="Add/Modify Weight:",font=("Arial", "10", "bold"))
modifyLabel.grid(row=6,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#---------CREATE LABEL AND SPINBOX FOR INPUT OF ADD/MODIFY WEIGHT-------------#

# Created label for "Enter Start Node" for adding/modifing the weight
startNodeLabel=Label(processSubFrame,text="Enter Start Node:")
startNodeLabel.grid(row=7,column=0,padx=10,pady=10)

# Following entry will take name of starting node from user for adding/modifing the weight
startNodeEntry = Spinbox(processSubFrame,textvariable=start_node_var)
startNodeEntry.grid(row=7,column=1,padx=10,pady=10)

# Created label for "Enter End Node" for adding/modifing the weight
endNodeLabel=Label(processSubFrame,text="Enter End Node:")
endNodeLabel.grid(row=7,column=2,padx=10,pady=10)

# Following entry will take name of end node from user for adding/modifing the weight
endNodeEntry = Spinbox(processSubFrame,textvariable=end_node_var)
endNodeEntry.grid(row=7,column=3,padx=10,pady=10)

# Created label for "Enter Weight" for adding/modifing the weight
weightLabel=Label(processSubFrame,text="Enter Weight:")
weightLabel.grid(row=7,column=4,padx=10,pady=10)

# Following entry will take name of weight to be added from user for adding/modifing the weight
WeightEntry = Entry(processSubFrame,textvariable=weight_var)
WeightEntry.grid(row=7,column=5,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#--------------------CREATE BUTTON FOR ADD/MODIFY WEIGHT----------------------#

ModifyButton = Button(processSubFrame, text ="Submit",relief=RAISED,command=lambda: mod_weight(graph))
ModifyButton.grid(row=7,column=6,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE LABEL FOR DELETE WEIGHT-------------------------#

deleteLabel = Label(processSubFrame,text="Delete Weight:",font=("Arial", "10", "bold"))
deleteLabel.grid(row=8,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------CREATE LABEL AND SPINBOX FOR INPUT OF DELETE WEIGHT--------------#

# Created label for "Enter Start Node" for deleting the weight
startNodeLabel1=Label(processSubFrame,text="Enter Start Node:")
startNodeLabel1.grid(row=9,column=0,padx=10,pady=10)

# Following entry will take name of starting node from user for deleting the weight
startNodeEntry1 = Spinbox(processSubFrame,textvariable=start_node_var1)
startNodeEntry1.grid(row=9,column=1,padx=10,pady=10)

# Created label for "Enter End Node" for deleting the weight
endNodeLabel1=Label(processSubFrame,text="Enter End Node:")
endNodeLabel1.grid(row=9,column=2,padx=10,pady=10)

# Following entry will take name of end node from user for deleting the weight
endNodeEntry1 = Spinbox(processSubFrame,textvariable=end_node_var1)
endNodeEntry1.grid(row=9,column=3,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE BUTTON FOR DELETE WEIGHT------------------------#

deleteButton = Button(processSubFrame, text ="Submit",relief=RAISED,command=lambda: del_edge(graph))
deleteButton.grid(row=9,column=4,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE LABEL FOR DELETE NODE-------------------------#

deleteNodeLabel = Label(processSubFrame,text="Delete Node:",font=("Arial", "10", "bold"))
deleteNodeLabel.grid(row=10,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#-------------CREATE LABEL AND SPINBOX FOR INPUT OF DELETE NODE---------------#

# Created label for "Enter Node" for deleting the node
deleteNodeLabel1=Label(processSubFrame,text="Enter Node:")
deleteNodeLabel1.grid(row=11,column=0,padx=10,pady=10)

# Following entry will take name of end node to be deleted from user
deleteNodeEntry = Spinbox(processSubFrame,textvariable=delete_node_var)
deleteNodeEntry.grid(row=11,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE BUTTON FOR DELETE NODE--------------------------#

deleteNodeButton = Button(processSubFrame, text ="Submit",relief=RAISED,command=lambda: remove_node(graph))
deleteNodeButton.grid(row=11,column=2,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------------------CREATE BUTTON FOE ADD NODE---------------------------#

addNodeButton = Button(processSubFrame, text ="Add Node",relief=RAISED,command=lambda: add_node(graph))
addNodeButton.grid(row=12,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#-------------------CREATE LABEL FOR OUTPUT FOR ADD NODE----------------------#

addNodeLabel = Label(processSubFrame)
addNodeLabel.grid(row=12,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------------CREATE BUTTON FOR RECOVER GRAPH------------------------#

recoverNodeButton = Button(processSubFrame, text ="Recover Graph",relief=RAISED,command=lambda: recover_node())
recoverNodeButton.grid(row=12,column=2,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#-----------------END OF WIDGETS CREATED IN processSubFrame-------------------#

#-----------------------------------------------------------------------------#

#---------------------END OF WIDGETS CREATED IN FRAME1------------------------#


#-----------------------------------------------------------------------------#

#-------------------CREATE SUB FRAME IN ROOT FOR ALL OUTPUTS------------------#

frame2 = Frame(frame,width=800,height=600)
frame2.grid(row=0,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#--------------------WIDGETS CREATED IN SUB FRAME FRAME2----------------------#

#-----------------------------------------------------------------------------#

#-------------------CREATE SUB FRAME IN FRAME2 FOR CANVAS---------------------#

canvasSubFrame = Frame(frame2)
canvasSubFrame.grid(row=0,column=0)

#-----------------------------------------------------------------------------#

#----------------WIDGETS CREATED IN SUB FRAME canvasSubFrame------------------#

# REFERENCE: http://matplotlib.org/examples/user_interfaces/embedding_in_tk.html
f = Figure(figsize=(5, 4), dpi=100)
a = f.add_subplot(111)
canvas = FigureCanvasTkAgg(f, master=canvasSubFrame)
canvas.show()
canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

toolbar = NavigationToolbar2TkAgg(canvas, canvasSubFrame)
toolbar.update()
canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)
#-------------------------------------------------------------#

def on_key_event(event):
    print('you pressed %s' % event.key)
    key_press_handler(event, canvas, toolbar)

canvas.mpl_connect('key_press_event', on_key_event)

#-----------------------------------------------------------------------------#

#-----------------------------QUITTING APPLICATION----------------------------#

def _quit():
    root.quit()  
    root.destroy()



#-----------------------------------------------------------------------------#

#----------------CREATE BUTTON FOR QUITTING APPLICATION-----------------------#

button = Button(master=canvasSubFrame, text='Quit', command=_quit)
button.pack(side=BOTTOM)

#-----------------------------------------------------------------------------#

#----------------END OF WIDGETS CREATED IN canvasSubFrame---------------------#

#-----------------------------------------------------------------------------#

#-------------------CREATE SUB FRAME IN FRAME2 FOR OUTPUT---------------------#

routeSubFrame = Frame(frame2)
routeSubFrame.grid(row=1,column=0)

#-----------------------------------------------------------------------------#

#---------------------WIDGETS CREATED IN routeSubFrame------------------------#

#-----------------------------------------------------------------------------#

#------------------CREATE LABEL FOR ROUTING TABLE OUTPUT----------------------#

routingTableLabel = Label(routeSubFrame)
routingTableLabel.grid(row=0,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#----------------CREATE LABEL FOR SINGLE SHORT PATH OUTPUT--------------------#

pathLabel = Label(routeSubFrame)
pathLabel.grid(row=1,column=0,padx=10,pady=10)
    
#-----------------------------------------------------------------------------#

#-------------CREATE LABEL FOR DISTANCE FOR SINGLE SHORT PATH-----------------#

distLabel = Label(routeSubFrame)
distLabel.grid(row=1,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#-----------------CREATE LABEL FOR ALL SHORT PATHS OUTPUT---------------------#

pathLabel1 = Label(routeSubFrame)
pathLabel1.grid(row=2,column=0,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#--------------CREATE LABEL FOR DISTANCES FOR ALL SHORT PATHS-----------------#

distLabel1 = Label(routeSubFrame)
distLabel1.grid(row=2,column=1,padx=10,pady=10)

#-----------------------------------------------------------------------------#

#------------END OF WIDGETS CREATED IN SUB FRAME routeSubFrame----------------#
        
#-----------------------------------------------------------------------------#

#----------------END OF WIDGETS CREATED IN SUB FRAME FRAME2-------------------#

#-----------------------------------------------------------------------------#

#----------------END OF WIDGETS CREATED IN MAIN FRAME ROOT--------------------#


root.mainloop()
#-----------------------------------------------------------------------------#

#---------------------------END OF MAIN WINDOW--------------------------------#



