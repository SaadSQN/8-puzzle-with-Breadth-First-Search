#Program to solve an 8 puzzle problem by inputting initial state as an array (row-wise , left to right) and inputting final ,desired state
#in the same format.Program will display the path of the optimal solution found using Breadth-First Search




import numpy as np
depthVariable = 0
visited = []
costVariable = 0
allNodes = []

class Node:
    def __init__(self, state, parent, operator, depth, cost):
        self.state = state
        self.parent = parent
        self.operator = operator
        self.depth = depth
        self.cost = cost

def create_node(state, parent, operator, depth, cost):
    return Node(state, parent, operator, depth, cost)

def move_left(state):
    global costVariable
    costVariable = costVariable + 1
    
    ZeroIndex = 0
    #Checking position of 0
    for x in state:
        if state[x] == 0:
            ZeroIndex = x
            break

    #Boundary conditions
    if x==0 or x==3 or x==6:
        return None

    #Switching Zero
    temp = state[x-1]
    state[x-1] = 0
    state[x] = temp    

    

    return state

def move_right(state):
    global costVariable
    costVariable = costVariable + 1
    
    ZeroIndex = 0
    #Checking position of 0
    for x in state:
        if state[x] == 0:
            ZeroIndex = x
            break

    #Boundary conditions
    if x==2 or x==5 or x==8:
        return None

    #Switching Zero
    temp = state[x+1]
    state[x+1] = 0
    state[x] = temp    

    

    return state    

def move_up(state):
    global costVariable
    costVariable = costVariable + 1
    
    ZeroIndex = 0
    #Checking position of 0
    for x in state:
        if state[x] == 0:
            ZeroIndex = x
            break

    #Boundary conditions
    if x==0 or x==1 or x==2:
        return None

    #Switching Zero
    temp = state[x-3]
    state[x-3] = 0
    state[x] = temp    

    

    return state


def move_down(state):
    global costVariable
    costVariable = costVariable + 1
   
    ZeroIndex = 0
    #Checking position of 0
    for x in state:
        if state[x] == 0:
            ZeroIndex = x
            break

    #Boundary conditions
    if x==6 or x==7 or x==8:
        return None

    #Switching Zero
    temp = state[x+3]
    state[x+3] = 0
    state[x] = temp    

    

    return state

def arrayChecker(s1 , s2):
    equal = True
    for x in s1:
        if s1[x] != s2[x]:
            equal = False
    return equal        

def checkVisited(a1):
    found = False
    for x in visited:
        if arrayChecker(a1,x.state) == True:
            found = True
    return found     

def expand_node(node):

    global depthVariable
    depthVariable = depthVariable + 1
    expanded_nodes = []
    # ----- code here
   
    ZeroIndex = 0
    #Checking position of 0
    for x in node.state:
        if node.state[x] == 0:
            ZeroIndex = x
            break
    
    #Check for move up 
    a1 = node.state.copy()
    if x!=0 and x!=1 and x!=2:
        a1 = move_up(a1)
        #check if its in visited
        if checkVisited(a1) == False:
            sub = create_node(a1,node.state,"up",depthVariable,depthVariable)
            expanded_nodes.append(sub)
            

     #Check for move down
    a2 = node.state.copy()
    if x!=6 and x!=7 and x!=8:
        a2 = move_down(a2)
        #check if its in visited
        if checkVisited(a2) == False:
            sub = create_node(a2,node.state,"down",depthVariable,depthVariable)
            expanded_nodes.append(sub)
            

     #Check for move right
    a3 = node.state.copy()
    if x!=2 and x!=5 and x!=8:
        a3 = move_right(a3)
        #check if its in visited
        if checkVisited(a3) == False:
            sub = create_node(a3,node.state,"right",depthVariable,depthVariable)
            expanded_nodes.append(sub)
            

     #Check for move left
    a4 = node.state.copy()
    if x!=0 and x!=3 and x!=6:
        a4 = move_left(a4)
        #check if its in visited
        if checkVisited(a4) == False:
            sub = create_node(a4,node.state,"left",depthVariable,depthVariable)
            expanded_nodes.append(sub)
            

    return expanded_nodes

"""def backTracker(answerNode , startNode):
    final = answerNode.state
    stack = []
    stack.append(final)
    print(stack)
    while (arrayChecker(final,startNode.state) != True):
        final = answerNode.state

        if answerNode.operator == "Up" or answerNode.operator == "up":
            final = move_down(final)
            stack.append(final)
            for x in reversed (visited):
                if(arrayChecker(x.state,answerNode.parent) and x.depth == answerNode.depth-1):
                    answerNode = x

        if answerNode.operator == "Down" or answerNode.operator == "down":
            final = move_up(final)
            stack.append(final)
            for x in reversed (visited):
                if(arrayChecker(x.state,answerNode.parent) and x.depth == answerNode.depth-1):
                    answerNode = x

        if answerNode.operator == "Right" or answerNode.operator == "right":
            final = move_left(final)
            stack.append(final)
            for x in reversed (visited):
                if(arrayChecker(x.state,answerNode.parent) and x.depth == answerNode.depth-1):
                    answerNode = x

        if answerNode.operator == "left" or answerNode.operator == "Left":
            final = move_right(final)
            stack.append(final)
            for x in reversed (visited):
                if(arrayChecker(x.state,answerNode.parent) and x.depth == (answerNode.depth)-1):
                    answerNode = x

        print (answerNode.state)
        print ("\n")
    
    #print (stack)

"""
def backTrackerVisited(answerNode , startNode):
    final = answerNode.state
    for x in reversed (visited):
        if(arrayChecker(answerNode.parent,x.state)):
             print(final)
             answerNode = x
             final = answerNode.state

def bfs(start, goal):
     #---- code here -----
    c = (np.array(start) == np.array(goal))
    finalAnswer = None
    checked = True
   #Check whether start is Goal
    for x in np.nditer(c):
        if x == False:
            checked = False
    
    if checked == True:
        finalAnswer = start
        return None
    
    visited.append(start)
    queue = []
    queue.append(start)
    answerFound = False
    allNodes.append(start)

    while answerFound == False:
        #if it isn't , expand the node
        #print (queue)
        nextNode = queue.pop(0)
        visited.append(nextNode)
        nextToCheck = expand_node(nextNode).copy()
        #for x in nextToCheck:
        #    print (x.state)
        for x in nextToCheck:
            allNodes.append(x)

        #Check if any of the new nodes are the answer
        for x in nextToCheck:
            if arrayChecker(x.state,goal.state) == True:
                #Break the Loop
                answerFound = True
                finalAnswer = x

        if answerFound == True:
            break

        #Check if new Nodes have been visited before
        for x in nextToCheck:
            found = False
            if checkVisited(x.state) == True:
                found = True
            if found == False:
                queue.append(x)
                
    #print("Cost of calculation : ")
    #print(costVariable)
    print("\n")
    print("Path to Solution : \n ")
    backTrackerVisited(finalAnswer , start)
    #print("\nNo of nodes expanded : ")
    counter = 0
    for x in visited:
        counter = counter + 1
    #print(counter)

    return None

def main():
    #Please Input final state in first array space of 'tester' Node
    #Please Input initial state in first array space of 'tester2' Node

    tester = create_node([1,2,3,4,5,6,7,8,0],[1,2,3,4,5,6,7,8,0],"None",0,0);
    tester2 = create_node([1,2,0,4,5,6,7,3,8],[1,0,2,4,5,6,7,3,8],"Right",0,0);
    returned = expand_node(tester).copy()
    #for x in returned:
    #    print (x.state)
    bfs(tester , tester2)

if __name__ == "__main__":
    main()