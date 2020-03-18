# myTeam.py
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


from captureAgents import CaptureAgent
import random, time, util
from game import Directions,Actions
import game
from util import nearestPoint
import pickle
#sys.path.append('teams/PacManGo/')

##################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'DummyAgent', second = 'DummyAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """




  def registerInitialState(self, gameState):


    global actionType
    global isUsingAstar
    self.faraway = False
    actionType=['Offensive','Offensive','Offensive','Offensive']
    isUsingAstar=[False,False,False,False]
    self.debug = False

    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''
    #TF wall matrix
    self.TFwalls = gameState.getWalls()
    #A list of the wall
    self.wallsList = gameState.getWalls().asList()
    #The middle coordinate X
    self.mapcentralLine = int(MapFeature.getMapcentralLine(self,gameState,self.wallsList))
    #Coordinates of the intermediate path
    self.borderPort = MapFeature.getBoderPort(self,gameState,self.mapcentralLine,self.TFwalls)
    #Initialize the food I need to defend
    self.myDefFood = self.getFoodYouAreDefending(gameState).asList()
    #Initialize my location information
    self.nowMyPosition =  gameState.getAgentState(self.index).getPosition()
    self.nextMyPosition = gameState.getAgentState(self.index).getPosition()
    #Coordinates of our possible position

    self.gameState=gameState
    self.mySidePositionList = MapFeature.mySidePositionList(self,gameState,self.mapcentralLine)
    self.allActFoodList =self.getFood(gameState).asList() 

    self.cornerFoodList = MapFeature.getcornerFood(self,gameState,self.wallsList,1)
    self.nowmyTeamPosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()

    self.enemyScaredStatus = False
    self.mytime = time.time()

    self.begintime = time.time()
    self.findGhost=(0,0)
    self.cornerList = MapFeature.getcornerList(self,gameState,self.wallsList,40)

    self.cornerList.sort()
    self.cornerAeraList = MapFeature.getcornerAera(self,gameState,self.cornerList)
    self.cornerPortList, self.cornerList = MapFeature.getcornerPort1(self,gameState,self.cornerAeraList,self.cornerList,self.wallsList)
    # print("######################################")
    # print("self.cornerList",self.cornerList)
    # print("------------------------------------------")
    # for i in self.cornerPortList:
    #   print(i)

    random.seed(time.time())
    self.choseBoderPort = random.choice(self.borderPort)

    self.initToCentral=True
    #The target coordinates
    self.target=()
    self.isTargetToFood = False

    self.changestatus = False

    self.allActCapsule =  MapFeature.myActCapsules(self,gameState)

    self.usingAstar = False     
    self.actionList = []          
    self.normalize = self.wallsList[-1][0] * self.wallsList[-1][1]
    self.myTeamMatePosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()

    if self.index < self.getOtherTeamIndex(gameState):
      self.initPosition = self.borderPort[0]
    else:
      self.initPosition = self.borderPort[-1]

    self.init = True

    self.backHomedict = dict()
    self.goToCapsuledict = dict()

    self.backHome = False
    self.goToCapsule = False


    self.offweights = util.Counter()
    self.offweights ={'successorScore': 30.3743673481738, 'distanceToCapsule': 10.480721589274932,
    'dangeroDistance': 30.0742197075443904, 'mindistanceToFood': -10.428393957981967, 'backMyside': 20.550308409506943}
    self.defweights = util.Counter()                
    self.defweights = { 'onDefense': 100, 'distanceToPacman': -10, 'distanceToGhost':-10, 
              'distanceToTarget': -20,'nearPacman':-1000, }

  #attack and defense transition
  def choseActionType(self, gameState):

    nowmyactfood =  self.getFood(gameState).asList()
    #print("nowmyactfood",nowmyactfood)
    if len(nowmyactfood) > 2:
      if not gameState.getAgentState(self.index).isPacman:
        actionType[self.index]="Offensive" 


    if gameState.data.timeleft <= 300:
      if not gameState.getAgentState(self.index).isPacman:
        actionType[self.index]="Defensive"


    if len(nowmyactfood) < 3:
      if not gameState.getAgentState(self.index).isPacman:
        actionType[self.index]="Defensive"      


    if len(nowmyactfood) > 2:
      if gameState.getAgentState(self.index).scaredTimer > 3:
        actionType[self.index]="Offensive"



  #get other agent index
  def getOtherTeamIndex(self, gameState):# find the index of each agent
    if self.index == self.getTeam(gameState)[0]:
      other_index = self.getTeam(gameState)[1]
    else:
      other_index = self.getTeam(gameState)[0]
    return other_index
      
  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getdefFeatures(gameState, action)
    weights = self.getdefWeights(gameState, action)

    return features * weights



  #Astar  
  def aStarSearch(self, gameState, nowPos, goalPos,startime):

    #this mathod is like wa* 
    #create a Priority queue (Take out the higher priority position first)  Stored node to wait access    
    open_set = util.PriorityQueue()
    #find which node has been visited    
    visited = []
    #The array of paths from the initial position to the current node
    actions = []
    #Put the initial state into the queue,Since it is the initial state and needs to be accessed first, the priority is 0  
    open_set.push((nowPos,actions),0)

    cx , cy = nowPos
    cx=int(cx)
    cy=int(cy)
    gx,gy = goalPos
    gx=int(gx)
    gy=int(gy)

    if (cx,cy)==(gx,gy):
      return ['Stop']


    #Continue operation when the queue is not empty
    while not open_set.isEmpty():
      test_action = []
      #Take the state stored in the queue and start accessing the node
      #The current_state is the coordinates of the node
      #The actions is The array of paths from the initial position to the current node
      current_state, actions = open_set.pop()

      endTime = time.time()
      #Return the path if the target node is found

      cx , cy = current_state
      cx=int(cx)
      cy=int(cy)
      gx,gy = goalPos
      gx=int(gx)
      gy=int(gy)

      if (cx,cy) == (gx,gy) :
        return actions

      #If the current state is not accessed, access is started
      if current_state not in visited and current_state in self.mySidePositionList:
        #Adds the node to the visited array
        visited.append(current_state)
        #Gets all Successor nodes of the access node
        #_, _, testout = MapFeature.getNearPosition(self, current_state, self.TFwalls)  
        stateList = MapFeature.getNearSidePosition(self, current_state, self.TFwalls)  
          #successors = problem.getSuccessors(current_state)
          #successor is (successor, action, stepCost)
        for nextPosition, nextAction  in stateList:
          #nextPos = testout[action]
          #Add the path which have traveled to plus the path you have moved to the successor node
          temp_actions = actions+[nextAction]
          #fx = gx + hx
          #gx is the cost of the path that have been moved
          #hx is the Estimate the cost of reaching the destination node
          temp_cost = len(temp_actions)+ self.getMazeDistance(current_state, goalPos)
          if (nextPosition not in visited):
            #is the statw is not visited, Put the next state and actions on the queue, 
            #The priority is determined by cost
            open_set.push((nextPosition, temp_actions),temp_cost)    
    if self.debug:
      print("cant find path")
    return []  


  #Astar  
  def aStarSearch2(self, gameState, nowPos, goalPos,startime):

    #this mathod is like wa* 
    #create a Priority queue (Take out the higher priority position first)  Stored node to wait access    
    open_set = util.PriorityQueue()
    #find which node has been visited    
    visited = []
    #The array of paths from the initial position to the current node
    actions = []
    #Put the initial state into the queue,Since it is the initial state and needs to be accessed first, the priority is 0  
    open_set.push((nowPos,actions),0)

    cx , cy = nowPos
    cx=int(cx)
    cy=int(cy)
    gx,gy = goalPos
    gx=int(gx)
    gy=int(gy)

    if (cx,cy)==(gx,gy):
      return ['Stop']

    #Continue operation when the queue is not empty
    while not open_set.isEmpty():
      test_action = []
      #Take the state stored in the queue and start accessing the node
      #The current_state is the coordinates of the node
      #The actions is The array of paths from the initial position to the current node
      current_state, actions = open_set.pop()

      endTime = time.time()
      #Return the path if the target node is found



      cx , cy = current_state
      cx=int(cx)
      cy=int(cy)
      gx,gy = goalPos
      gx=int(gx)
      gy=int(gy)

      if (cx,cy) == (gx,gy) :
        return actions
       
      #If the current state is not accessed, access is started
      if current_state not in visited:
        #Adds the node to the visited array
        visited.append(current_state)
        #Gets all Successor nodes of the access node
        #_, _, testout = MapFeature.getNearPosition(self, current_state, self.TFwalls)  
        stateList = MapFeature.getNearSidePosition(self, current_state, self.TFwalls)  


          #successors = problem.getSuccessors(current_state)
          #successor is (successor, action, stepCost)
        for nextPosition, nextAction  in stateList:
          #nextPos = testout[action]
          #Add the path which have traveled to plus the path you have moved to the successor node
          temp_actions = actions+[nextAction]
          #fx = gx + hx
          #gx is the cost of the path that have been moved
          #hx is the Estimate the cost of reaching the destination node
          temp_cost = len(temp_actions)+ self.getMazeDistance(current_state, goalPos)
          if (nextPosition not in visited):
            #is the statw is not visited, Put the next state and actions on the queue, 
            #The priority is determined by cost
            open_set.push((nextPosition, temp_actions),temp_cost)    
    return []  


  #from baselineTeam
  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor  





  #Select the action to perform
  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    #Get legal action
    actions = gameState.getLegalActions(self.index)
    #print("actions",actions)
    #Gets when the code started
    Timer = time.time()

    #My current position
    self.nowMyPosition =  gameState.getAgentState(self.index).getPosition()
    #print("myposition",self.nowMyPosition)
    self.myTeamMatePosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()

    self.choseActionType(gameState)
    #print(actionType)
    nearGhost = MapFeature.getEnemyGhost(self, gameState)
    nearGhostPosition = MapFeature.getEnemyGhostPosition(self, gameState)
    nearPacmanPosition = MapFeature.getEnemyPacmanPosition(self, gameState)
    nearPacman = MapFeature.getEnemyPacman(self, gameState)
    myNextDefFood = self.getFoodYouAreDefending(gameState).asList()

    #If they eat my food
    if len(self.myDefFood) > len(myNextDefFood):
      self.target=list(set(self.myDefFood)-set(myNextDefFood))[0]
      self.isTargetToFood = True
      self.myDefFood=myNextDefFood




    if actionType[self.index]=="Offensive":
      self.mytime = time.time()
      distanceToGhost = 9999
      enemyScared = False


      #If the teammate is not using the search algorithm
      if not isUsingAstar[self.getOtherTeamIndex(gameState)]:
        #print("my self.index",self.index)
        
        #If I find my opponent, and I'm a ghost, and I have to go to the other side because I'm on the offensive,
        if nearGhostPosition and not gameState.getAgentState(self.index).isPacman:
          distanceToGhost = min([self.getMazeDistance(self.nowMyPosition, ghostPos) for ghostPos in nearGhostPosition])
          for scaredEnemy in nearGhost:
            #If your opponent is in a state of fear, go straight to the center
            if scaredEnemy.scaredTimer>0:
              enemyScared = True

          #Change the exit when you find my opponent
          #If the opponent is less than 5 away from me, the opponent is not afraid
          if distanceToGhost < 2 and not enemyScared :
            #Use Astar to find the path
            #Choose the exit with oneself coordinate different, need to change an exit
            if self.nowMyPosition in self.borderPort:
              #print("im in border")
              nowPort = self.borderPort.copy()
              nowPort.remove(self.nowMyPosition)
              while self.choseBoderPort == self.nowMyPosition or self.choseBoderPort == self.myTeamMatePosition:
                self.choseBoderPort = random.choice(self.borderPort)
              self.usingAstar = True
              isUsingAstar[self.index]=True
              self.actionList=[]

      #If my teammates use the search algorithm, I don't
      else:
        self.usingAstar = False
        isUsingAstar[self.index]=False
        self.actionList=[]    



      if self.init:
        self.init = False
        #isUsingAstar[self.index]=True
        self.usingAstar = True
        self.choseBoderPort = self.initPosition

#######################
      if self.nowMyPosition in self.borderPort and not nearGhostPosition:
        self.usingAstar = False
        isUsingAstar[self.index]=False
        self.actionList=[]          

#######################

      #If I have not reached the selected border entry
      if self.nowMyPosition != self.choseBoderPort:
        #If you need to use Astar
        if self.usingAstar:
          #If the previously selected action is not empty
          if self.actionList:
            findaction = self.actionList[0]

            self.actionList.remove(findaction)
            if findaction in actions:

              nowtime = time.time()
              if ((nowtime - Timer)*2) > 1:
                print("time:",nowtime - Timer)
              return findaction  

          #If the previous action is empty               
          else:
            x , y = self.choseBoderPort
            x = int(x)
            y = int(y)
            self.actionList = self.aStarSearch(gameState,self.nowMyPosition,(x,y),Timer) 
            #print("actionList",self.actionList)

            if self.actionList:
              findaction = self.actionList[0]
              self.actionList.remove(findaction)
              if findaction in actions:


                nowtime = time.time()
                if ((nowtime - Timer)*2) > 1:
                  print("time:",nowtime - Timer)
                return findaction
            else:
              self.usingAstar = False
              isUsingAstar[self.index]=False

      else:
        #No further execution is required when the boundary is reached
        self.usingAstar = False
        isUsingAstar[self.index]=False

      action = self.getPolicy(gameState)
      self.begintime = time.time()
      strr = self.begintime - self. mytime

      nowtime = time.time()
      if ((nowtime - Timer)*2) > 1:
        print("time:",nowtime - Timer)
      return action






    if actionType[self.index]=="Defensive":
      #Gets information about the opponent found in the current state


      #Defense doesn't need to stop
      #remove the stop action
      actions.remove(Directions.STOP)

      #You don't have to go to the other side when you're defending
      #remove the pacman action under some situations
      #if self.isTargetToFood == False:
      neardistance = 9999
      if nearPacmanPosition:
        nearPacmanPosition.sort(key=self.getDistance)
        neardistance = self.getDistance(nearPacmanPosition[0]) 
      #If I'm a ghost I need to defend
      if not gameState.getAgentState(self.index).isPacman:
        actions = self.removeChangeStateAction(gameState,actions)

      #会丢失视野
      #If you need to go after food, and there's no food nearby
      if self.isTargetToFood == True and neardistance > 5:
          #print("去追食物",self.target)
          action = self.aStarSearch(gameState, self.nowMyPosition , self.target, Timer)
          #action = action[0]
          if action:

            nowtime = time.time()
            if ((nowtime - Timer)*2) > 1:
              print("time:",nowtime - Timer)
            return action[0]

      values = [self.evaluate(gameState, a) for a in actions]      
      maxValue = max(values)
      

      bestActions = [a for a, v in zip(actions, values) if v == maxValue]

      nowtime = time.time()
      if ((nowtime - Timer)*2) > 1:
        print("time:",nowtime - Timer) 
      return random.choice(bestActions)








#########################################################################################################
#########################################################################################################
#######################              Next Part is Deffensive           ##################################
#########################################################################################################
#########################################################################################################

  def getdefFeatures(self, gameState, action):  
    features = util.Counter()  
    successor = self.getSuccessor(gameState,action)
    #The state of the successor node
    nextAgentState = successor.getAgentState(self.index)
    #Current my position
    nowMyPosition = gameState.getAgentState(self.index).getPosition()
    #My place next time
    nextMyPosition = successor.getAgentState(self.index).getPosition()
    #I need food to defend at present
    nowmydefFoodList=self.getFoodYouAreDefending(gameState).asList()
    #Randomly select the boundary coordinates to go
    choseBoderPort = random.choice(self.borderPort)
    #Food that I need to defend in the next state
    myNextDefFood = self.getFoodYouAreDefending(successor).asList()
    #Gets information about the opponent found in the next state
    nearGhost = MapFeature.getEnemyGhostPosition(self, successor)
    nearPacman = MapFeature.getEnemyPacmanPosition(self, successor)
    features['nearPacman'] = len(nearPacman)

    #Am I afraid
    myScaredStatus = gameState.getAgentState(self.index).scaredTimer > 0

    #Start setting to center
    if self.initToCentral:
      self.initToCentral=False
      self.target=choseBoderPort
    

    #If they eat my food
    if len(self.myDefFood) > len(myNextDefFood):
      #Get the coordinates of the food
      self.target=list(set(self.myDefFood)-set(myNextDefFood))[0]
      #print("对方吃掉了我的食物",self.target)
      self.isTargetToFood = True
      self.myDefFood=myNextDefFood
      

    #If the other person is eaten, spit out the food
    if len(self.myDefFood) < len(myNextDefFood):
      #Update food information
      self.myDefFood = myNextDefFood



    #如果附近有敌人
    #If there's an opponent nearby and it's pacman
    if nearPacman:

      distanceToPacman = [self.getMazeDistance(nextMyPosition, enemy) for enemy in nearPacman]
      lowestdistanceToPacman= min(distanceToPacman)
      #The shortest distance to the enemy
      features['distanceToPacman'] = lowestdistanceToPacman/self.normalize

      #The goal is set as the center point, and there is no need to track the food
      self.target=choseBoderPort
      self.isTargetToFood=False


    else:
      #Find nearby ghosts, and don't need to go to the final food
      if nearGhost and self.isTargetToFood==False:
        distanceToGhost = [self.getMazeDistance(nextMyPosition, enemy) for enemy in nearGhost]
        features['distanceToGhost'] = min(distanceToGhost)/self.normalize

      else:

        features['distanceToTarget'] = self.getMazeDistance(nextMyPosition,self.target)/self.normalize

    #If I'm pacman I need to go back
    if gameState.getAgentState(self.index).isPacman:
      features['distanceToTarget'] = self.getMazeDistance(nextMyPosition,self.target)/self.normalize
      features['nearPacman'] = 0
      self.nextMyPosition = nextMyPosition
      if nearGhost:
        nearGhost.sort(key=self.getNextDistance)
        features['distanceToGhost'] = self.getMazeDistance(nextMyPosition,nearGhost[0])/self.normalize

    return features


  def getdefWeights(self, gameState, action):
    return self.defweights





#########################################################################################################
#########################################################################################################
#######################              Next Part is Offensive            ##################################
#########################################################################################################
#########################################################################################################

  def getQVal(self,gameState,action):

    features = self.getOffFeatures(gameState,action)
    weights = self.getoffWeight(gameState,action)
    if self.debug:
      print("action",action)
      print("features",features)
      #print("weights",weights)
    return features*weights



  def getNextMaxQValueAction(self, gameState):
    capsules = MapFeature.myActCapsules(self, gameState)
    qValues=[]
    self.nowMyPosition = gameState.getAgentPosition(self.index)
    actions = gameState.getLegalActions(self.index)
    temp_actions = actions.copy()
    if actionType[self.index]=="Offensive" and gameState.getAgentState(self.index).isPacman :
      #Gets information about the opponent found in the next state
      nearGhostPositionList = MapFeature.getEnemyGhostPosition(self, gameState)
      nearPacmanPositionList = MapFeature.getEnemyPacmanPosition(self, gameState)    
      nearGhostList = MapFeature.getEnemyGhost(self, gameState)
      nearGhostPositionList.sort(key=self.getDistance)
      #If I'm not in the corner
      if self.nowMyPosition not in self.cornerList:
        if nearGhostList:
          minearGhost = [i for i in nearGhostList if i.getPosition() == nearGhostPositionList[0] ]
          mindistanceToGhost = self.getMazeDistance(self.nextMyPosition, nearGhostPositionList[0]) 
          if minearGhost[0].scaredTimer < 3 and mindistanceToGhost < 4:
            #print("remove action")
            for action in actions.copy():
              x, y = gameState.getAgentPosition(self.index)
              dx, dy = Actions.directionToVector(action)
              nextX, nextY = int(x + dx), int(y + dy)
              nextPosition = (nextX,nextY)
              #The next step is the corner, get rid of that one
              if nextPosition in self.cornerList:
                #print("Don't go to the corner",nextPosition)
                hascapsule = False
                insubfalg = False
                sublist = []
                subport = (0,0)
                for co in self.cornerPortList:
                  sublist,subport = co
                  if nextPosition in sublist:
                    insubfalg = True
                    break
                if insubfalg:
                  for cap in capsules:
                    if cap in sublist:
                      hascapsule = True
                if action in temp_actions and  not hascapsule:
                  temp_actions.remove(action)


      #If there is an enemy nearby
      if nearGhostList:
        minearGhost = [i for i in nearGhostList if i.getPosition() == nearGhostPositionList[0] ]
        mindistanceToGhost = self.getMazeDistance(self.nextMyPosition, nearGhostPositionList[0]) 

        for action in actions.copy():
          x, y = gameState.getAgentPosition(self.index)
          dx, dy = Actions.directionToVector(action)
          nextX, nextY = int(x + dx), int(y + dy)
          nextPosition = (nextX,nextY)

          #The next step has an enemy, remove that step
          if nextPosition == minearGhost[0].getPosition() and  minearGhost[0].scaredTimer < 1:

            if action in temp_actions:
              temp_actions.remove(action)


        if  mindistanceToGhost > 10:
          if 'Stop' in temp_actions:
            temp_actions.remove("Stop")
      else:
        if 'Stop' in temp_actions:
          temp_actions.remove("Stop")   




    if temp_actions:
      actions = temp_actions
    for action in actions:
        qValues.append((self.getQVal(gameState,action),action))


    maxQ = max(qValues)

    if self.backHomedict[maxQ[1]]:
      self.backHome=True
    if self.goToCapsuledict[maxQ[1]]:
      self.goToCapsule=True

    if self.debug:
      print("qValues######",qValues)    
      print("maxQ#######",maxQ)

    return maxQ



  def getPolicy(self,gameState):
    if self.debug:
      print("----------------------------------------------------------",self.index)

    #print("----------------------------------------------------------",self.index)

    action = self.getNextMaxQValueAction(gameState)
    if self.debug:
      print("self.backHome",self.backHome)
      print("self.goToCapsule",self.goToCapsule)
      print("##############################################################")
    #print("##############################################################")

    return action[1]    




  def removeChangeStateAction(self,gameState,action):
    for i in action:
      successorState = self.getSuccessor(gameState, i).getAgentState(self.index)
      if successorState.isPacman:
        action.remove(i)
    return action



  #Attack  feature 
  def getOffFeatures(self, gameState, action):
    self.enemyScaredStatus= False
    self.findGhost = (0,0)
    self.gameState = gameState
    features = util.Counter()  
    #The subsequent nodes
    successor = self.getSuccessor(gameState,action)
    #The state of the successor node
    nextAgentState = successor.getAgentState(self.index)
    #Current my position
    nowMyPosition = gameState.getAgentState(self.index).getPosition()
    #My place next time
    nextMyPosition = successor.getAgentState(self.index).getPosition()

    self.nextMyPosition= successor.getAgentState(self.index).getPosition()

    nowmyTeamPosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()
    #I need food to defend at present
    nowmydefFoodList=self.getFoodYouAreDefending(gameState).asList()
    #Food that I need to defend in the next state
    myNextDefFood = self.getFoodYouAreDefending(successor).asList()

    #Next state food list
    nextactFoodList = self.getFood(successor).asList()
    #Food list of current status  
    nowactFoodList = self.getFood(gameState).asList() 

    self.faraway = False
    #My food
    nowMyCarry = gameState.getAgentState(self.index).numCarrying
    #print("nowMyCarry",nowMyCarry)
    #Gets information about the opponent found in the next state
    nearGhost = MapFeature.getEnemyGhostPosition(self, successor)
    nearPacman = MapFeature.getEnemyPacmanPosition(self, successor)  
    #Update my location information
    self.nowMyPosition =  gameState.getAgentState(self.index).getPosition()

    self.backHomedict[action] = False
    self.goToCapsuledict[action] = False

    #self.backHome = False
    #self.goToCapsule = False



    #Find food in the corner
    cornerFoodList = MapFeature.getcornerFood(self,gameState,self.wallsList,1)
    #Locate the attack capsule
    actCapsules =  MapFeature.myActCapsules(self,gameState)
    #The next step
    nextactCapsules =  MapFeature.myActCapsules(self,successor)


    #The food list in the corner has a depth of 1
    self.cornerFoodList = MapFeature.getcornerFood(self,gameState,self.wallsList,1)

    if len(nextactFoodList) < 2:

      features['successorScore'] = -float(len(nowactFoodList)+(len(actCapsules)))/(len(self.allActFoodList)+len(self.allActCapsule))

    else:
      features['successorScore'] = -float(len(nextactFoodList)+(len(nextactCapsules)))/(len(self.allActFoodList)+len(self.allActCapsule))

    #features['teamDis'] = (self.getMazeDistance(self.nextMyPosition,nowmyTeamPosition))/(self.normalize )/10

    #Gets information about the opponent found in the next state
    nearGhostPositionList = MapFeature.getEnemyGhostPosition(self, successor)
    nearPacmanPositionList = MapFeature.getEnemyPacmanPosition(self, successor)    
    nearGhostList = MapFeature.getEnemyGhost(self, successor)
    mindistanceToGhost = 9999

    #If there are enemies nearby ghost
    if nearGhostPositionList:
      nearGhostPositionList.sort(key=self.getNextDistance) 
      minearGhost = [i for i in nearGhostList if i.getPosition() == nearGhostPositionList[0] ]
      mindistanceToGhost = self.getMazeDistance(self.nextMyPosition, nearGhostPositionList[0])  
      nowmindistancetosule = self.getMazeDistance(self.nowMyPosition, nearGhostPositionList[0]) 
      if minearGhost[0].scaredTimer > 0:
        self.enemyScaredStatus= True
      if mindistanceToGhost >10:
        self.enemyScaredStatus= True
        
      self.findGhost = nearGhostPositionList[0]

    #Choose the nearest exit
    self.borderPort.sort(key=self.myMazeDis)
    


    mindistanceToBorder = self.myMazeDis(self.borderPort[0])
    #print("self.borderPort",mindistanceToBorder)  


    #Rank the distance from the food to me
    #nextactFoodList.sort(key=self.getDistance)
    nextactFoodList.sort(key=self.myMazeDis)

    startime = time.time()
    #If there's food nearby
    if nextactFoodList:
      #If one is attacking and one is defending then the distance from the food is the direct distance
      if actionType[self.index] != actionType[self.getOtherTeamIndex(gameState)]:      

      
        if nearGhostPositionList:

          distance =  self.myMazeDis(nextactFoodList[0])
          features['mindistanceToFood'] = distance/(self.normalize )

        else:
          features['mindistanceToFood'] = (self.getMazeDistance(self.nextMyPosition,nextactFoodList[0]))/(self.normalize )


      else:
        #Otherwise, the two attackers were given different food distances
        if self.index < self.getOtherTeamIndex(gameState):

          ###myteamToFoodDis = self.getMazeDistance(self.nowmyTeamPosition,nextactFoodList[0])

          #mydisToFood = self.getMazeDistance(self.nextMyPosition,nextactFoodList[0])

          mydistance =  self.myMazeDis(nextactFoodList[0])
          myteamdistance =  self.myTeamMazeDis(nextactFoodList[0])

          features['mindistanceToFood'] = (mydistance/myteamdistance)/self.normalize
        else:
          distance =  self.myMazeDis(nextactFoodList[0])
          features['mindistanceToFood'] = (distance)/(self.normalize )

    #If there is no food
    else:
      features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
      features['mindistanceToFood'] = -1
    actCapsules.sort(key=self.myMazeDis)
    mindistancetosule=9999
    nowmindistancetosule=9999

    aflag = False 
    #If I were in our position
    if not gameState.getAgentState(self.index).isPacman:
      
      if nearGhostPositionList:
        if mindistanceToGhost < 2:
          features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
          notcornerFoodList = list(set(nextactFoodList)-set(self.cornerFoodList))
          notcornerFoodList.sort(key=self.getDistance)

        if actionType[self.index] != actionType[self.getOtherTeamIndex(gameState)]:      

          mydistance =  self.myMazeDis(nextactFoodList[0])
          features['mindistanceToFood'] = mydistance/(self.normalize )


        else:
          if self.index < self.getOtherTeamIndex(gameState):
            if nextactFoodList:

              mydistance =  self.myMazeDis(nextactFoodList[0])
              
              myteamdistance =  self.myTeamMazeDis(nextactFoodList[0])

              features['mindistanceToFood'] = (mydistance/myteamdistance)/self.normalize
          else:
            if nextactFoodList:
              mydistance =  self.myMazeDis(nextactFoodList[0])
              features['mindistanceToFood'] = (mydistance)/(self.normalize )

    #I'm in enemy position
    else:
      #If there is an enemy nearby
      if nearGhostPositionList:

        if self.nowMyPosition in self.cornerList and minearGhost[0].scaredTimer < 1 and mindistanceToGhost < 3:
          cornerPort = ()
          sublist = []
          inflag = False
          for find_aera in self.cornerPortList:
            sublist, cornerPort = find_aera
            if self.nowMyPosition in sublist:
              inflag=True
              break
          if inflag:
            distanceToPort = self.getNextDistance(cornerPort)
            ghostdistanceToPort =  self.getMazeDistance(minearGhost[0].getPosition(),cornerPort)
            ghostdistanceTocapsule = 9999
            if actCapsules:
              ghostdistanceTocapsule = self.getMazeDistance(minearGhost[0].getPosition(),cornerPort)

              mydistance =  self.myMazeDis(actCapsules[0])

              if mydistance == 0:
                mydistance = 0.01
              mindistancetosule = mydistance


            aflag=False
            if (ghostdistanceToPort  - distanceToPort) > 1:
              aflag=True
              features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)
              features['mindistanceToFood'] = 0
              features['successorScore'] = 0
            else:
              hascapsule = False
              for cap in actCapsules:
                if cap in sublist:
                  hascapsule = True
              if actCapsules and hascapsule:
                if actCapsules[0]  in sublist and (mindistancetosule - ghostdistanceTocapsule)>2 :
                  features['distanceToCapsule'] = (self.normalize)/float(mindistancetosule)*100
                features['mindistanceToFood'] = 0




        #print("mindistanceToGhost",mindistanceToGhost)
        #print("nowMyCarry",nowMyCarry)
        if mindistanceToGhost < 3  and minearGhost[0].scaredTimer < 3:

          features['dangeroDistance'] =   -(self.normalize)/(self.getMazeDistance(self.nextMyPosition,minearGhost[0].getPosition())*100)         

          if nowMyCarry > 6:
            features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)
            features['mindistanceToFood'] = 0
            features['distanceToCapsule'] = 0 
            self.backHomedict[action]=True

          else:
            if actCapsules:
              mindistancetosule = self.myMazeDis(actCapsules[0])
              if mindistancetosule == 0:
                mindistancetosule = 0.01
              features['distanceToCapsule'] = (self.normalize)/(float(mindistancetosule)*100)
              features['mindistanceToFood'] = 0
              features['backMyside'] = 0
              self.goToCapsuledict[action]=True
            else:
              if nowMyCarry > 0:
                features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)
                features['mindistanceToFood'] = 0
                features['distanceToCapsule'] = 0 
                self.goToCapsuledict[action]=True


        if mindistanceToGhost > 10 or minearGhost[0].scaredTimer > 3:
          self.faraway = True





      else:
        self.faraway = True


      if self.faraway:
        self.goToCapsule = False
        self.backHome = False


    if not gameState.getAgentState(self.index).isPacman:
      self.backHome = False
      self.goToCapsule = False

    if gameState.getAgentState(self.index).isPacman:
      if self.goToCapsule:
        if actCapsules:
          mindistancetosule = self.myMazeDis(actCapsules[0])
          if mindistancetosule == 0:
            mindistancetosule = 0.01
          features['distanceToCapsule'] = (self.normalize)/(float(mindistancetosule)*100)
          features['mindistanceToFood'] = 0
          features['backMyside'] = 0
        else:
          features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)
          features['mindistanceToFood'] = 0
          features['distanceToCapsule'] = 0 



      if self.backHome:
        features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)
        features['mindistanceToFood'] = 0
        features['distanceToCapsule'] = 0 





    if gameState.data.timeleft <= 200:
      features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
      features['mindistanceToFood'] = 0


    if len(nowactFoodList) < 3 :
      features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
      features['mindistanceToFood'] = 0

    weights = self.offweights
    self.findGhost = (0,0)

    if self.debug:
      print("now carry",nowMyCarry)
    return features




  def myMazeDis(self,goalPos):
    mypos = self.nextMyPosition
    startime = time.time()
    ghostPos = self.findGhost

    flag = False
    if ghostPos == (0,0):
      return self.getMazeDistance(mypos,goalPos)

    if self.enemyScaredStatus:
      return self.getMazeDistance(mypos,goalPos)



    if ghostPos:
      actions = self.aStarSearch2( self.gameState,mypos,goalPos,startime)
      if actions:
        flag=True
      len_actions = len(actions)
      i=0
      x,y = mypos
      gx,gy = ghostPos
      gx = int(gx)
      gy = int(gy)
      while actions:
        action = actions[0]
        actions.remove(action)
        dx, dy = Actions.directionToVector(action)
        Newx, Newy = int(x + dx), int(y + dy)

        if (Newx, Newy) == (gx,gy):
          i=20
        x,y = (Newx, Newy)

      if flag:
        return (len_actions+i)
      else:
        return self.getMazeDistance(mypos,goalPos)

    else:
      return self.getMazeDistance(mypos,goalPos)





  def myTeamMazeDis(self,goalPos):

    mypos = self.myTeamMatePosition
    startime = time.time()
    ghostPos = self.findGhost
    flag = False

    if self.enemyScaredStatus:
      return self.getMazeDistance(mypos,goalPos)

    if ghostPos == (0,0):
      return self.getMazeDistance(mypos,goalPos)


    if ghostPos:
      actions = self.aStarSearch2( self.gameState,mypos,goalPos,startime)
      if actions:
        flag=True
      len_actions = len(actions)
      i=0
      x,y = mypos

      while actions:
        action = actions[0]
        actions.remove(action)
        dx, dy = Actions.directionToVector(action)
        Newx, Newy = int(x + dx), int(y + dy)
        if (Newx, Newy) == ghostPos:
          i=20
        x,y = (Newx, Newy)

      if flag:
        return (len_actions+i)
      else:
        return self.getMazeDistance(mypos,goalPos)

    else:
      return self.getMazeDistance(mypos,goalPos)






  def getoffWeight(self, gameState, action):
    return self.offweights




  def getDistance(self, pos1):
    return self.getMazeDistance(self.nowMyPosition, pos1)

  def getNextDistance(self, pos1):
    return self.getMazeDistance(self.nextMyPosition, pos1)



class MapFeature:

  def getcornerFood(self,gameState,wallsList,num):
    foodList = self.getFood(gameState).asList()
    cornerFoodList = []
    actionList = ['North', 'South', 'East', 'West','Stop']
    otherFoodList1 = foodList.copy()
    for i in range(num):
      otherFoodList=[]
      for coor in otherFoodList1:
        x,y = coor
        haswallnum = 0
        for action in actionList:
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)
          if (Newx, Newy) in wallsList:
            haswallnum+=1
          if (Newx, Newy) in cornerFoodList:
            haswallnum+=1
        if haswallnum > 2:
          cornerFoodList.append(coor)
        else:
          otherFoodList.append(coor)
      otherFoodList1 =otherFoodList

    return cornerFoodList

  def getcornerList(self,gameState,wallsList,num):
    allLegaPosition = gameState.getWalls().asList(False)
    cornerList = []
    actionList = ['North', 'South', 'East', 'West','Stop']
    otherList1 = allLegaPosition.copy()
    for i in range(num):
      otherList=[]
      for coor in otherList1:
        x,y = coor
        haswallnum = 0
        hascoornum = 0 
        for action in actionList:
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)
          if (Newx, Newy) in wallsList:
            haswallnum+=1
          if (Newx, Newy) in cornerList:
            hascoornum+=1
        if ((haswallnum+hascoornum) > 2  and haswallnum > 1) or haswallnum > 2:
          cornerList.append(coor)
        else:
          otherList.append(coor)
      otherList1 =otherList

    return cornerList

  #Gets the middle position of the map to get the boundary between the red team and the blue team.The red team and the blue team are different boundaries
  def getMapcentralLine(self,gameState,wallsList):
    if self.red:
      centralLine = gameState.data.layout.width/2-1
    else:
      centralLine = gameState.data.layout.width/2
    return centralLine





  def getBoderPort(self,gameState,centralLine,TFwalls):
    height = gameState.data.layout.height
    borderPort = []
    for coorY in range(height):
      if not TFwalls[centralLine][coorY]:
        borderPort.append((centralLine, coorY))
    return borderPort



  def mySidePositionList(self,gameState,centralLine):
    sidePositionList = []
    for x,y in gameState.getWalls().asList(False):

      if self.red and x <= centralLine:
        sidePositionList.append((x,y))
      if not self.red and x >=  centralLine:
        sidePositionList.append((x,y))
    return sidePositionList



  def getEnemyGhost(self,gameState):

    #Gets a list of enemies[<game.AgentState object at 0x000001D90C97C550>, <game.AgentState object at 0x000001D90C97C4A8>]
    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    #From the list of enemies, get the list of enemies in ghost state
    nearGhostList = [enemy for enemy in enemyList if not enemy.isPacman and enemy.getPosition() != None]
    return nearGhostList




  def getEnemyGhostPosition(self,gameState):

    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    nearGhostList = [enemy for enemy in enemyList if not enemy.isPacman and enemy.getPosition() != None]
    nearGhostPositionList = [nearGhost.getPosition() for nearGhost in nearGhostList]
    return nearGhostPositionList

  def getEnemyPacman(self,gameState):

    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    nearPacmanList = [enemy for enemy in enemyList if enemy.isPacman and enemy.getPosition() != None]
    return nearPacmanList


  def getEnemyPacmanPosition(self,gameState):

    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    nearPacmanList = [enemy for enemy in enemyList if enemy.isPacman and enemy.getPosition() != None]
    nearPacmanPositionList = [nearPacman.getPosition() for nearPacman in nearPacmanList]
    return nearPacmanPositionList

  #Gets the valid coordinates and actions near the current coordinates
  def getNearSidePosition(self,position,TFwalls):
    actionList = ['North', 'South', 'East', 'West','Stop']
    x,y = position
    nearPositionList = []
    for action in actionList:
      dx, dy = Actions.directionToVector(action)
      Newx, Newy = int(x + dx), int(y + dy)
      if not TFwalls[Newx][Newy]:
        nearPositionList.append(((Newx, Newy),action))

    return nearPositionList

  def myActCapsules(self,gameState):
    returnList = []
    capsules = gameState.getCapsules()
    defCapsules = self.getCapsulesYouAreDefending(gameState)
    for capsule in capsules:
      if capsule in defCapsules:
        continue
      returnList.append(capsule)
    return  returnList 


  def getcornerPort(self,gameState,cornerAera,cornerList,wallsList):

    new_list = []
    for corner in cornerAera:
      sub_aera = corner
      port = ()
      for sub in corner:
        actionList = ['North', 'South', 'East', 'West']
        for action in actionList:
          x,y=sub
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)
          if ((Newx, Newy) not in cornerList) and ((Newx, Newy) not in wallsList):
            port = sub
      new_list.append((sub_aera,port))

    return new_list


  def getcornerPort1(self,gameState,cornerAera,cornerList,wallsList):
    #The first step is to calculate the initial corner
    port_coor = (0,0)
    new_list = []
    for corner in cornerAera:
      sub_aera = corner
      port = ()
      for sub in corner:
        actionList = ['North', 'South', 'East', 'West']
        for action in actionList:
          x,y=sub
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)
          if ((Newx, Newy) not in cornerList) and ((Newx, Newy) not in wallsList):
            port = sub
            port_coor = (Newx, Newy)
      new_list.append((list(set(sub_aera)),port,port_coor))

    #The second step is to determine if there is a wall, if there is a corner, if there is a corner,
    lennewlist = len(new_list)
    new_area_list=[]
    for i in range(lennewlist):
      new_area=[]
      newsub, newport, safecoor = new_list[i]
      new_area=newsub
      for j in range(lennewlist):
        if i == j:
          continue
        a,b,c =new_list[j] 

        if c == safecoor:
          haswallnum = 0
          actionList = ['North', 'South', 'East', 'West']
          for action in actionList:
            x,y=safecoor
            dx, dy = Actions.directionToVector(action)
            Newx, Newy = int(x + dx), int(y + dy)
            if (Newx, Newy) in wallsList:
              haswallnum+=1
          if haswallnum > 0:
            new_area = newsub+a+[c]

      new_area.sort()
      new_area = list(set(new_area))
      if new_area not in new_area_list:
        new_area_list.append(new_area)


    cornerList = []
    for i in new_area_list:
      cornerList=cornerList+i


    port_coor = (0,0)
    new_list = []
    for corner in new_area_list:
      sub_aera = corner
      port = ()
      for sub in corner:
        actionList = ['North', 'South', 'East', 'West']
        for action in actionList:
          x,y=sub
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)

          if ((Newx, Newy) not in cornerList) and ((Newx, Newy) not in wallsList):
            port = sub
            port_coor = (Newx, Newy)
      new_list.append((list(set(sub_aera)),port_coor))



    for number in range(1):
      new_sub_aera=[]
      for sub_aera, safe_port in new_list:
        actionList = ['North', 'South', 'East', 'West']
        haswallnum = 0
        for action in actionList:
          x,y=safe_port
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)
          if (Newx, Newy) in wallsList:
            haswallnum+=1
        if haswallnum>1:
          sub_aera.sort()
          new_sub_aera.append (sub_aera + [safe_port])
          cornerList.append(safe_port)
        else:
          sub_aera.sort()
          new_sub_aera.append(sub_aera) 

      port_coor = (0,0)
      new_list = []
      for corner in new_sub_aera:
        sub_aera = corner
        port = ()
        for sub in corner:
          actionList = ['North', 'South', 'East', 'West']
          for action in actionList:
            x,y=sub
            dx, dy = Actions.directionToVector(action)
            Newx, Newy = int(x + dx), int(y + dy)

            if ((Newx, Newy) not in cornerList) and ((Newx, Newy) not in wallsList):
              port = sub
              port_coor = (Newx, Newy)
        sub_aera.sort()      
        new_list.append((list(set(sub_aera)),port_coor))


    return (new_list,cornerList)




  def getcornerAera(self,gameState,cornerList):
    aera_List =[]
    copy_cornerList = cornerList.copy()

    open_set = util.Stack()
    while copy_cornerList:
      open_set.push(copy_cornerList[0])
      #coor =open_set.pop()
      copy_cornerList.remove(copy_cornerList[0])
      sub_aera = [] 
      #sub_aera.append(copy_cornerList[0])

      while not open_set.isEmpty():
        coor = open_set.pop()
        sub_aera.append(coor)
        actionList = ['North', 'South', 'East', 'West']      
        for action in actionList:
          x,y=coor
          dx, dy = Actions.directionToVector(action)
          Newx, Newy = int(x + dx), int(y + dy)
          if (Newx, Newy) in copy_cornerList:
            temp_coor = (Newx, Newy)
            sub_aera.append(temp_coor)
            open_set.push(temp_coor)
            copy_cornerList.remove((Newx, Newy))

      aera_List.append(sub_aera)  

    return aera_List




