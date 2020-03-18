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

#################
# Team creation #
#################
DEBUG = True
interestingValues = {}
def createTeam(firstIndex, secondIndex, isRed,
               first = 'DummyAgent', second = 'DummyAgent',**args):
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
  if 'numTraining' in args:
    interestingValues['numTraining'] = args['numTraining']
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
    global mytargetFood
    global isUsingAstar

    actionType=['Offensive','Offensive','Offensive','Offensive']
    isUsingAstar=[False,False,False,False]
    mytargetFood=[(),(),(),()]

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

    self.mapcentralLine = int(MapFeature.getMapcentralLine(self,gameState,self.wallsList))

    self.borderPort = MapFeature.getBoderPort(self,gameState,self.mapcentralLine,self.TFwalls)

    self.myDefFood = self.getFoodYouAreDefending(gameState).asList()
    self.nowMyPosition =  gameState.getAgentState(self.index).getPosition()
    self.nextMyPosition = gameState.getAgentState(self.index).getPosition()


    self.mySidePositionList = MapFeature.mySidePositionList(self,gameState,self.mapcentralLine)
    self.allActFoodList =self.getFood(gameState).asList() 
    self.cornerFoodList = MapFeature.getcornerFood(self,gameState,self.wallsList,1)
    self.nowmyTeamPosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()
    self.AstaractionList = []

    self.cornerList = MapFeature.getcornerList(self,gameState,self.wallsList,40)

    self.cornerList.sort()


    self.cornerAeraList = MapFeature.getcornerAera(self,gameState,self.cornerList)



    self.cornerPortList = MapFeature.getcornerPort(self,gameState,self.cornerAeraList,self.cornerList,self.wallsList)

    random.seed(time.time())
    self.choseBoderPort = random.choice(self.borderPort)

    self.initToCentral=True
    #The target coordinates
    self.target=()
    self.isTargetToFood = False

    self.changestatus = False

    self.alpha = 0.5
    self.discount = 0.8

    #self.epsilon = 0.05
    #self.episodesSoFar = 0

    #pkl_file = open('offweight.pkl', 'rb')

    #data1 = pickle.load(pkl_file)

    self.runAway = False

    self.offweights = util.Counter()

    self.offweights ={'successorScore': 50.3743673481738, 'distanceToCapsule': -10.480721589274932, 'successorCapsuleScore':50.3743673481738,
    'dangeroDistance': 21.0742197075443904, 'mindistanceToFood': -10.428393957981967, 'backMyside': 20.550308409506943}
    self.defweights = util.Counter()                
    self.defweights = { 'onDefense': 100, 'distanceToPacman': -10, 'distanceToGhost':-10,'distanceToTarget': -20,'nearPacman':-1000, }
    self.usingAstar = False     
    self.actionList = []          
    self.normalize = self.wallsList[-1][0] * self.wallsList[-1][1]
    self.myTeamMatePosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()
    if self.index < self.getOtherTeamIndex(gameState):
      self.initPosition = self.borderPort[0]
    else:
      self.initPosition = self.borderPort[-1]

    self.init = True



  #attack and defense transition
  def choseActionType(self, gameState):
   
    #Update my current position
    self.nowMyPosition = gameState.getAgentState(self.index).getPosition()
    #If I and my teammates are on offense
    #print("self.index",gameState.getAgentState(self.index).scaredTimer)
    if gameState.getAgentState(self.index).scaredTimer <= 3:
      if actionType[self.index]=="Offensive" and actionType[self.getOtherTeamIndex(gameState)]=="Offensive":
        myAgentCarrying = gameState.getAgentState(self.index).numCarrying
        myTeamAgentCarrying = gameState.getAgentState(self.getOtherTeamIndex(gameState)).numCarrying
        if myAgentCarrying > myTeamAgentCarrying:
          self.changestatus = True
          actionType[self.index]="Defensive"
          #Randomly select the boundary coordinates to go
          choseBoderPort = random.choice(self.borderPort)
          self.target=choseBoderPort
        #else:
          #actionType[self.getOtherTeamIndex(gameState)]="Defensive"

    if actionType[self.index]=="Defensive":
      if gameState.getAgentState(self.index).scaredTimer > 3:
        actionType[self.index]="Offensive"  
    
    myTeamMatePosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()
    enemyPacman= MapFeature.getEnemyPacman(self,gameState)


    if len(enemyPacman) > 0:
      myenemyDistClose = min([self.getMazeDistance(self.nowMyPosition, a.getPosition()) for a in enemyPacman])
      teamEnemyDistClose = min([self.getMazeDistance(myTeamMatePosition, a.getPosition()) for a in enemyPacman])
      if myenemyDistClose < teamEnemyDistClose:
        actionType[self.index]="Defensive"
        actionType[self.getOtherTeamIndex(gameState)] = "Offensive"

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

    if nowPos==goalPos:
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

      if current_state == goalPos :
        return actions
       

      #If the time is longer than 0.7 seconds, you should quit the search. The calculation time specified by the teacher is less than 1 second
      if (endTime - startime)>0.7:
        #print("计算超时")
        return []     

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
    #print("cant find path")
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




  #Gets the Features of the defense
  def getdefFeatures(self, gameState, action):  
    features = util.Counter()  
    successor = self.getSuccessor(gameState,action)
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

    #Start setting to center
    if self.initToCentral:
      self.initToCentral=False
      self.target=choseBoderPort
    

    #If they eat my food
    if len(self.myDefFood) > len(myNextDefFood):
      #Get the coordinates of the food
      self.target=list(set(self.myDefFood)-set(myNextDefFood))[0]
      self.isTargetToFood = True
      self.myDefFood=myNextDefFood
      

    #If the other person is eaten, spit out the food
    if len(self.myDefFood) < len(myNextDefFood):
      #Update food information
      self.myDefFood = myNextDefFood




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

  #Get defensive weight
  def getdefWeights(self, gameState, action):
    return self.defweights

  def getQVal(self,gameState,action):

    features = self.getOffFeatures(gameState,action)
    weights = self.getoffWeight(gameState,action)
    return features*weights

  #Gets the maximum value for the next action
  def getNextMaxQValueAction(self, gameState):
    qValues=[]
    actions = gameState.getLegalActions(self.index)
    temp_actions = actions.copy()
    if actionType[self.index]=="Offensive" and gameState.getAgentState(self.index).isPacman :
      #Gets information about the opponent found in the next state
      nearGhostPositionList = MapFeature.getEnemyGhostPosition(self, gameState)
      nearPacmanPositionList = MapFeature.getEnemyPacmanPosition(self, gameState)    
      nearGhostList = MapFeature.getEnemyGhost(self, gameState)

      if self.nowMyPosition not in self.cornerList:
        if nearGhostList:
          minearGhost = [i for i in nearGhostList if i.getPosition() == nearGhostPositionList[0] ]
          mindistanceToGhost = self.getMazeDistance(self.nextMyPosition, nearGhostPositionList[0]) 
          if minearGhost[0].scaredTimer < 2 and mindistanceToGhost < 5:
            
            for action in actions.copy():
              x, y = gameState.getAgentPosition(self.index)
              dx, dy = Actions.directionToVector(action)
              nextX, nextY = int(x + dx), int(y + dy)
              nextPosition = (nextX,nextY)
              if nextPosition in self.cornerList:
                if action in temp_actions:
                  temp_actions.remove(action)

      if nearGhostList:
        minearGhost = [i for i in nearGhostList if i.getPosition() == nearGhostPositionList[0] ]
        mindistanceToGhost = self.getMazeDistance(self.nextMyPosition, nearGhostPositionList[0]) 
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
    return maxQ


  def getPolicy(self,gameState):
    action = self.getNextMaxQValueAction(gameState)
    self.offweights = self.updateQ(gameState,action[1])
    return action[1]    

  #Update my weight information
  def updateQ(self,gameState,action):
    weights = self.offweights
    successor = self.getSuccessor(gameState, action)
    #Get the subsequent state
    nextState = successor.getAgentState(self.index)
    nowState = gameState.getAgentState(self.index)
    myPos =  nowState.getPosition()

    #Get the next food
    foodList = self.getFood(successor).asList()
    #My current coordinates
    myNextPos =  nextState.getPosition()
    #Subsequent enemy positions
    enemyGhostPositionList = MapFeature.getEnemyGhostPosition(self,gameState)
    #Initialization reward
    rewards = 0 
    if enemyGhostPositionList:
      distanceToEnemyGhost = (min([self.getMazeDistance(myPos, ghostPos) for ghostPos in enemyGhostPositionList]))
    else:
      distanceToEnemyGhost = 9999

    x, y = gameState.getAgentPosition(self.index)
    dx, dy = Actions.directionToVector(action)
    nextX, nextY = int(x + dx), int(y + dy)
    nextPosition = (nextX,nextY)

    flag = False
    enemyGhost = MapFeature.getEnemyGhost(self,gameState)
    for i in enemyGhost:
      if i.scaredTimer == 0:
        flag=True

    #If there is food next, reward it
    if gameState.hasFood(nextX,nextY):
      if nextPosition in self.cornerFoodList and distanceToEnemyGhost<3 and flag:
        rewards -= 1
      else:
        rewards+=1
    else:
      rewards -= 1
    capsuleList = MapFeature.myActCapsules(self,gameState)

    #If the next position is a capsule, a reward is given
    if (nextX,nextY) in capsuleList:
      rewards+=1

    if self.getScore(successor) - self.getScore(gameState)<=0:
      rewards -= 1
    else:
      rewards +=1  

    features = self.getOffFeatures(gameState,action)  
    NextMaxQ = self.getNextMaxQValueAction(successor)
    currentQ = self.getQVal(gameState,action)

    for feature in features: 
      weights[feature] = weights[feature]+ (self.alpha*(rewards+self.discount*NextMaxQ[0]-currentQ)*features[feature])

    return weights     


  #Select the next action
  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """

    actions = gameState.getLegalActions(self.index)
    #print("actions",actions)
    #Gets when the code started
    Timer = time.time()
    #My current position
    self.nowMyPosition =  gameState.getAgentState(self.index).getPosition()
    self.myTeamMatePosition = gameState.getAgentState(self.getOtherTeamIndex(gameState)).getPosition()
    self.choseActionType(gameState)

    nearGhost = MapFeature.getEnemyGhost(self, gameState)
    nearPacman = MapFeature.getEnemyPacman(self, gameState)
    nearGhostPosition = MapFeature.getEnemyGhostPosition(self, gameState)
    nearPacmanPosition = MapFeature.getEnemyPacmanPosition(self, gameState)




    if actionType[self.index]=="Offensive":
      distanceToGhost = 9999
      enemyScared = False

      if not isUsingAstar[self.getOtherTeamIndex(gameState)]:
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
              nowPort = self.borderPort.copy()
              nowPort.remove(self.nowMyPosition)
              while self.choseBoderPort!=self.nowMyPosition and self.choseBoderPort!= self.myTeamMatePosition:
                self.choseBoderPort = random.choice(self.borderPort)
                self.usingAstar = True
                isUsingAstar[self.index]=True
                self.actionList=[]

      else:
        self.usingAstar = False
        isUsingAstar[self.index]=False
        self.actionList=[]    

      if self.init:
        self.init = False
        #isUsingAstar[self.index]=True
        self.usingAstar = True
        self.choseBoderPort = self.initPosition

      #If I have not reached the selected border entry
      if self.nowMyPosition != self.choseBoderPort:
        #If you need to use Astar
        if self.usingAstar:
          #If the previously selected action is not empty
          if self.actionList:
            findaction = self.actionList[0]
            self.actionList.remove(findaction)
            if findaction in actions:
              return findaction  
          #If the previous action is empty               
          else:
            x , y = self.choseBoderPort
            x = int(x)
            y = int(y)
            self.actionList = self.aStarSearch(gameState,self.nowMyPosition,(x,y),Timer) 

            #print("self.actionList:",self.actionList) 
            if self.actionList:
              findaction = self.actionList[0]
              self.actionList.remove(findaction)
              if findaction in actions:
                return findaction
            else:
              self.usingAstar = False
              isUsingAstar[self.index]=False

      else:
        #No further execution is required when the boundary is reached
        self.usingAstar = False
        isUsingAstar[self.index]=False

      action = self.getPolicy(gameState)
      return action

    if actionType[self.index]=="Defensive":
      #remove the stop action
      actions.remove(Directions.STOP)

      #You don't have to go to the other side when you're defending

      if not gameState.getAgentState(self.index).isPacman:
        actions = self.removeChangeStateAction(gameState,actions)

      #From baseline      
      values = [self.evaluate(gameState, a) for a in actions]      
      maxValue = max(values)
      bestActions = [a for a, v in zip(actions, values) if v == maxValue]
      foodLeft = len(self.getFood(gameState).asList())
      if foodLeft <= 2:
        bestDist = 9999
        for action in actions:
          successor = self.getSuccessor(gameState, action)
          pos2 = successor.getAgentPosition(self.index)
          dist = self.getMazeDistance(self.nowMyPosition,pos2)
          if dist < bestDist:
            bestAction = action
            bestDist = dist
        return bestAction

      #If you need to chase food, and there are no ghosts near
      if self.isTargetToFood == True and len(nearPacman)==0:
          action = self.aStarSearch(gameState, self.nowMyPosition , self.target, Timer)
          #action = action[0]
          if action:
            return action[0]
           
                      
      return random.choice(bestActions)


  def removeChangeStateAction(self,gameState,action):
    for i in action:
      successorState = self.getSuccessor(gameState, i).getAgentState(self.index)
      if successorState.isPacman:
        action.remove(i)
    return action

  #Attack Features
  def getOffFeatures(self, gameState, action):
    features = util.Counter()  
    #successor node
    successor = self.getSuccessor(gameState,action)
    #The state of the successor node
    nextAgentState = successor.getAgentState(self.index)
    #Current my position
    nowMyPosition = gameState.getAgentState(self.index).getPosition()
    #My place next time
    nextMyPosition = successor.getAgentState(self.index).getPosition()

    self.nextMyPosition= successor.getAgentState(self.index).getPosition()
    #I need food to defend at present
    nowmydefFoodList=self.getFoodYouAreDefending(gameState).asList()
    #Food that I need to defend in the next state
    myNextDefFood = self.getFoodYouAreDefending(successor).asList()

    nextactFoodList = self.getFood(successor).asList()    
    nowactFoodList = self.getFood(gameState).asList() 


    #My Carry  food 
    nowMyCarry = gameState.getAgentState(self.index).numCarrying
    #Gets information about the opponent found in the next state
    nearGhost = MapFeature.getEnemyGhostPosition(self, successor)
    nearPacman = MapFeature.getEnemyPacmanPosition(self, successor)  
    #Update my location information
    self.nowMyPosition =  gameState.getAgentState(self.index).getPosition()
    #Choose the nearest exit
    self.borderPort.sort(key=self.getDistance)

    mindistanceToBorder = self.getNextDistance(self.borderPort[0])
    #Find food in the corner
    cornerFoodList = MapFeature.getcornerFood(self,gameState,self.wallsList,1)
    #Locate the attack capsule
    actCapsules =  MapFeature.myActCapsules(self,gameState)
    #next step Capsules
    nextactCapsules =  MapFeature.myActCapsules(self,successor)

    nextactFoodList.sort(key=self.getDistance)
    self.cornerFoodList = MapFeature.getcornerFood(self,gameState,self.wallsList,1)
    features['successorScore'] = -float(len(nextactFoodList))/len(self.allActFoodList)
    features['successorCapsuleScore'] = -float(len(nextactFoodList))/len(self.allActFoodList)


    #Gets information about the opponent found in the next state
    nearGhostPositionList = MapFeature.getEnemyGhostPosition(self, successor)
    nearPacmanPositionList = MapFeature.getEnemyPacmanPosition(self, successor)    
    nearGhostList = MapFeature.getEnemyGhost(self, successor)
    mindistanceToGhost = 9999
    if nextactFoodList:
      if actionType[self.index] != actionType[self.getOtherTeamIndex(gameState)]:      
        features['mindistanceToFood'] = (self.getMazeDistance(self.nextMyPosition,nextactFoodList[0]))/(self.normalize )
      else:

        if self.index < self.getOtherTeamIndex(gameState):
          myteamToFoodDis = self.getMazeDistance(self.nowmyTeamPosition,nextactFoodList[0])
          mydisToFood = self.getMazeDistance(self.nextMyPosition,nextactFoodList[0])
          features['mindistanceToFood'] = (mydisToFood/myteamToFoodDis)/self.normalize
        else:
          features['mindistanceToFood'] = (self.getMazeDistance(self.nextMyPosition,nextactFoodList[0]))/(self.normalize )


    else:
      features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
      features['mindistanceToFood'] = -1



    if not nowactFoodList:
      features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )

    self.nextMyPosition = nextMyPosition
    toDot=True
    toCapsule=True
    actCapsules.sort(key=self.getNextDistance)
    mindistancetosule=9999
    nowmindistancetosule=9999
    if nearGhostPositionList:
      nearGhostPositionList.sort(key=self.getNextDistance) 
      minearGhost = [i for i in nearGhostList if i.getPosition() == nearGhostPositionList[0] ]
      mindistanceToGhost = self.getMazeDistance(self.nextMyPosition, nearGhostPositionList[0])  
      nowmindistancetosule = self.getMazeDistance(self.nowMyPosition, nearGhostPositionList[0])  



    #If I were in our position
    if not gameState.getAgentState(self.index).isPacman:
      
      if nearGhostPositionList:
        if mindistanceToGhost < 2:
          features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
          notcornerFoodList = list(set(nextactFoodList)-set(self.cornerFoodList))
          notcornerFoodList.sort(key=self.getDistance)
          if notcornerFoodList:
            features['mindistanceToFood'] = (self.getMazeDistance(self.nextMyPosition,notcornerFoodList[0]))/(self.normalize)
          else:
            if nextactFoodList:
              features['mindistanceToFood'] = (self.getMazeDistance(self.nextMyPosition,nextactFoodList[0]))/(self.normalize )
   


    #I'm in enemy position
    else:
      if nowMyCarry > (len(self.allActFoodList)/2):
        #print("biger back ")
        features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)

      if nearGhostPositionList:
        #In the corner
        if self.nowMyPosition in self.cornerList and minearGhost[0].scaredTimer < 2:
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
              mindistancetosule = self.getNextDistance(actCapsules[0])

            if (distanceToPort - ghostdistanceToPort) > 1:
              features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)
            else:
              if actCapsules:
                if actCapsules[0]  in sublist and (mindistancetosule - ghostdistanceTocapsule)>2 :
                  features['distanceToCapsule'] = float(mindistancetosule)/(self.normalize)




        #If the enemy is close to me
        if mindistanceToGhost < 3  and minearGhost[0].scaredTimer < 2:
          features['successorScore'] = 0
          if actCapsules:
            mindistancetosule = self.getNextDistance(actCapsules[0])
            features['distanceToCapsule'] = float(mindistancetosule)/(self.normalize)
          if nowMyCarry:
            features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)

          if mindistanceToGhost < 2 or nowmindistancetosule:
            features['mindistanceToFood'] = 0
            features['distanceToCapsule'] = 0

            features['dangeroDistance'] = (self.getMazeDistance(self.nextMyPosition,minearGhost[0].getPosition())/(self.normalize))          
          if nowMyCarry > 6:
            features['backMyside'] = -float(mindistanceToBorder)/(self.normalize)


    if len(nowactFoodList) < 3 :
      features['backMyside'] = -float(mindistanceToBorder)/(self.normalize )
      features['mindistanceToFood'] = 0

    return features


  def getoffWeight(self, gameState, action):
    return self.offweights

  def getDistance(self, pos1):
    return self.getMazeDistance(self.nowMyPosition, pos1)

  def getNextDistance(self, pos1):
    return self.getMazeDistance(self.nextMyPosition, pos1)


#Extract map features
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
        if ((haswallnum+hascoornum) > 2 and haswallnum > 1) or haswallnum > 2:
          cornerList.append(coor)
        else:
          otherList.append(coor)
      otherList1 =otherList

    return cornerList

  #Gets the middle position of the map to get the boundary between the red team and the blue team.The red team and the blue team are different boundaries
  def getMapcentralLine(self,gameState,wallsList):
    if self.red:
      #The red team is on the left of the center line
      centralLine = gameState.data.layout.width/2-1
    else:
      #The blue team is to the right of the center line
      centralLine = gameState.data.layout.width/2
    #Returns the value of the midline
    return centralLine





  #Gets the coordinates of the middle boundary, excluding the position of the wall
  def getBoderPort(self,gameState,centralLine,TFwalls):
    height = gameState.data.layout.height
    borderPort = []
    #Traverse the height of the map, determine if there is a wall according to the TF wall matrix, and if there is no wall, put it into the list of acceptable boundaries
    for coorY in range(height):
      if not TFwalls[centralLine][coorY]:
        borderPort.append((centralLine, coorY))
    return borderPort



  def mySidePositionList(self,gameState,centralLine):
    #Get our possible position coordinates
    sidePositionList = []
    for x,y in gameState.getWalls().asList(False):

      if self.red and x <= centralLine:
        sidePositionList.append((x,y))
      if not self.red and x >=  centralLine:
        sidePositionList.append((x,y))
    return sidePositionList



  def getEnemyGhost(self,gameState):

    #Gets a list of enemies [<game.AgentState object at 0x000001D90C97C550>, <game.AgentState object at 0x000001D90C97C4A8>]
    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    #From the list of enemies, get the list of enemies in ghost state
    nearGhostList = [enemy for enemy in enemyList if not enemy.isPacman and enemy.getPosition() != None]
    return nearGhostList




  def getEnemyGhostPosition(self,gameState):

    #Gets a list of enemies[<game.AgentState object at 0x000001D90C97C550>, <game.AgentState object at 0x000001D90C97C4A8>]
    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    #From the list of enemies, get the list of enemies in ghost state
    nearGhostList = [enemy for enemy in enemyList if not enemy.isPacman and enemy.getPosition() != None]
    #Gets the location of the ghost
    nearGhostPositionList = [nearGhost.getPosition() for nearGhost in nearGhostList]
    return nearGhostPositionList

  def getEnemyPacman(self,gameState):

    #Gets a list of enemies[<game.AgentState object at 0x000001D90C97C550>, <game.AgentState object at 0x000001D90C97C4A8>]
    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    #From the list of enemies, get the list of enemies in ghost state
    nearPacmanList = [enemy for enemy in enemyList if enemy.isPacman and enemy.getPosition() != None]
    return nearPacmanList


  def getEnemyPacmanPosition(self,gameState):

    #Gets a list of enemies [<game.AgentState object at 0x000001D90C97C550>, <game.AgentState object at 0x000001D90C97C4A8>]
    enemyList = [gameState.getAgentState(opponent) for opponent in self.getOpponents(gameState)]
    #From the list of enemies, get the list of enemies in ghost state
    nearPacmanList = [enemy for enemy in enemyList if enemy.isPacman and enemy.getPosition() != None]
    #Gets the location of the ghost
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

  #Get my capsule coordinates
  def myActCapsules(self,gameState):
    returnList = []
    capsules = gameState.getCapsules()
    defCapsules = self.getCapsulesYouAreDefending(gameState)
    for capsule in capsules:
      if capsule in defCapsules:
        continue
      returnList.append(capsule)
    return  returnList 


  #Get the exit of each subcorner list
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




  #Calculate the coordinate list for each corner
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

