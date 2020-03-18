# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """
    #global  var
    moves_path=[]

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

# successors.append( ( nextState, action, cost) )
# successors include nextState, action, cost

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    open_set  = util.Stack()
    visited = []
    actions = []

    open_set.push((problem.getStartState(), actions))
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    while not open_set.isEmpty():

        current_state, actions = open_set.pop()
       # print("current_state:", current_state)
       # print("actions:", actions)

        if problem.isGoalState(current_state):
            return actions

        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, move, cost in successors:
                if (state not in visited):
                    open_set.push((state, actions + [move]))

    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    open_set  = util.Queue()
    visited = []
    actions = []
    open_set.push((problem.getStartState(), actions))
    while not open_set.isEmpty():
        current_state, actions = open_set.pop()
        print("current_state:", current_state)
        print("actions:", actions)
        if problem.isGoalState(current_state):
            return actions    
        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, move, cost in successors:
                if (state not in visited):
                    open_set.push((state, actions + [move]))                


    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    open_set  = util.PriorityQueue()
    visited = []
    actions = []
    temp_actions = []
    open_set.push((problem.getStartState(), actions),0)
    while not open_set.isEmpty():
        current_state, actions = open_set.pop()
        print("current_state:", current_state)
        print("actions:", actions)
        if problem.isGoalState(current_state):
            return actions    
        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, move, cost in successors:
                temp_actions = actions+[move]
                temp_cost = problem.getCostOfActions(temp_actions)
                if (state not in visited):
                    open_set.push((state, temp_actions),temp_cost)    



    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    open_set  = util.PriorityQueue()
    visited = []
    actions = []
    temp_actions = []
    open_set.push((problem.getStartState(), actions),0)
    while not open_set.isEmpty():
        current_state, actions = open_set.pop()
        print("current_state:", current_state)
        print("actions:", actions)
        if problem.isGoalState(current_state):
            return actions    
        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, move, cost in successors:
                temp_actions = actions+[move]
                temp_cost = problem.getCostOfActions(temp_actions)+ heuristic(state,problem)
                if (state not in visited):
                    open_set.push((state, temp_actions),temp_cost)    


    util.raiseNotDefined()

def iterativeDeepeningSearch(problem):
    """Search the deepest node in an iterative manner."""
    "*** YOUR CODE HERE FOR TASK 1 ***"
    max_num = 100

    for i in range(max_num):
        #print ("depth = ",i)
        result = depthLimitedSearch(problem,i)
        if result != 0:
            return result
    util.raiseNotDefined()


def depthLimitedSearch(problem,depth):

    if depth == 0:
        if problem.isGoalState(problem.getStartState()):
            return actions
        return 0

    open_set  = util.Stack()
    visited = []
    close_set = []
    actions = []
    see_list = []
    open_set.push((problem.getStartState(), actions,0))
    close_set.append(problem.getStartState())
    see_list.append(("start:",problem.getStartState()))

    while not open_set.isEmpty():
        #print ("see_list  = ",see_list)
        current_state, actions,current_depth = open_set.pop()
        #print ("current_depth  = ",current_depth)
        #print ("actions  = ",actions)
        see_list.pop()

 
        if problem.isGoalState(current_state):
            #print ("maxdep goal = ",max_dep)

            return actions

        if (current_state not in visited) and (current_depth < depth):
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)

            for state, move, cost in successors:
                if (state not in close_set):
                    close_set.append(state)
                    open_set.push((state, actions + [move],current_depth+1))
                    see_list.append(("non-start:",state))
    return 0                
    #util.raiseNotDefined()


def waStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has has the weighted (x 2) lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 2 ***"
    open_set  = util.PriorityQueue()
    visited = []
    actions = []
    temp_actions = []
    open_set.push((problem.getStartState(), actions),0)
    while not open_set.isEmpty():
        current_state, actions = open_set.pop()
        #print("current_state:", current_state)
        #print("actions:", actions)
        if problem.isGoalState(current_state):
            return actions    
        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, move, cost in successors:
                temp_actions = actions+[move]
                temp_cost = 2*problem.getCostOfActions(temp_actions)+ heuristic(state,problem)
                if (state not in visited):
                    open_set.push((state, temp_actions),temp_cost)  


    #util.raiseNotDefined()

def waStarSearch2(problem, heuristic=nullHeuristic):
    """Search the node that has has the weighted (x 2) lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 2 ***"
    open_set  = util.PriorityQueue()
    visited = []
    actions = []
    temp_actions = []
    open_set.push((problem.getStartState(), actions),0)
    while not open_set.isEmpty():
        current_state , actions = open_set.pop()
        #print("current_state:", current_state)
        #print("actions:", actions)
        if problem.isGoalState(current_state)  :
            return actions    
        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, next_food, next_capsule, move in successors:
                temp_actions = actions+[move]
                temp_cost = 2*problem.getCostOfActions(temp_actions)+ heuristic(state,problem)
                if (state not in visited):
                    open_set.push(((state, next_food,next_capsule), temp_actions),temp_cost)  


def depthFirstSearch2(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    open_set  = util.Stack()
    visited = []
    actions = []

    open_set.push((problem.getStartState(), actions))
    print("Start:", problem.getStartState())
    print("walls:", problem.getwalls())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    cap = problem.getCapsules()


    while not open_set.isEmpty():

        current_state, actions = open_set.pop()
       # print("current_state:", current_state)
       # print("actions:", actions)
        #print("current_state0000:",current_state[0])
        #print("current_state2222:",current_state[2])

        if current_state[2][0][0]==current_state[0][0]  and current_state[2][0][1]==current_state[0][1] :
            print("find it :",actions)  
            open_set.push((current_state,[]))
            break


        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, next_food,next_capsule, move , cost in successors:
                flag =  problem.hasFood(state[0],state[1])

                if (state not in visited) and flag == False:
                    open_set.push(((state,next_food, next_capsule), actions + [move]))
                    print("actions:",actions)    


    after_action = []                
    while not open_set.isEmpty():

        current_state, after_action = open_set.pop()
       # print("current_state:", current_state)
       # print("actions:", actions)

        if problem.isGoalState(current_state):
            return actions+after_action

        if current_state not in visited:
            visited.append(current_state)
            successors = problem.getSuccessors(current_state)
            for state, next_food,next_capsule, move , cost in successors:

                if (state not in visited):
                    open_set.push(((state,next_food, next_capsule), after_action + [move]))

    util.raiseNotDefined()















def depthFirstSearch3(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"

    moves = []
    visited_state = []

    dfstest(problem, problem.getStartState(),moves,visited_state)

    result = moves_path
    print("result:", moves_path)

    return result

    util.raiseNotDefined()




def dfstest(problem, current_state, moves , visited_state ):
    if problem.isGoalState(current_state):
        print ("moves:",moves)
        global moves_path 
        moves_path = moves
        return 

    if  current_state  not in visited_state:
        visited_state.append(current_state)
        successors = problem.getSuccessors(current_state)
        for next_state, move, cost in successors:
                if (next_state not in visited_state):
                    dfstest(problem,next_state,moves+[move],visited_state)
    else:     
        return      



moves_path=[]


def iterativeDeepeningSearch2(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"

    moves = []
    visited_state = []

    for i in range(70):
        print(i)
        moves = []
        visited_state = []
        moves = dls(problem, problem.getStartState(),moves,visited_state,i)
        #print("moves_path:",moves_path)
        #print("moves:",moves)
        result = moves_path
        if not len(moves_path)==0 :
            return result

    util.raiseNotDefined()


def dls(problem, current_state, moves , visited_state , depth):
    #print (moves)
    if depth == 0 :
        if problem.isGoalState(current_state):
            return moves
    elif depth > 0 :
        #if  current_state  not in visited_state:
            #visited_state.append(current_state)
        successors = problem.getSuccessors(current_state)
        for next_state, move, cost in successors:
            #if (next_state not in visited_state):
            found = dls(problem,next_state,moves+[move],visited_state,depth-1)
            if found != []:
                return found
    return []            





# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
dfs2 = depthFirstSearch2
dfs3 = depthFirstSearch3

astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
ids2 = iterativeDeepeningSearch2


wastar = waStarSearch
wastar2 = waStarSearch2