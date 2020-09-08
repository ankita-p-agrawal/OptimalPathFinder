from heapq import heapify, heappop


class MarsPathFinder:
    visitedNodeSet = set()
    nodeToParentMap = {}
    priorityQueue = []

    def __init__(self, inputFilePath):
        self.columns, self.rows = [int(num) for num in inputFilePath.readline().split()]
        self.landing_y, self.landing_x = [int(num) for num in inputFilePath.readline().split()]
        self.maxElevation = int(inputFilePath.readline())
        self.numOfTargetSites = int(inputFilePath.readline())

        self.targetList = []
        for eachTarget in range(0, self.numOfTargetSites):
            self.targetList.append([int(num) for num in inputFilePath.readline().split()])
        self.elevationMap = []
        for eachRow in range(0, self.rows):
            self.elevationMap.append([int(num) for num in inputFilePath.readline().split()])

        self.currentTarget_x = 0
        self.currentTarget_y = 0

    def cleanup(self):
        self.nodeToParentMap.clear()
        self.visitedNodeSet.clear()
        if hasattr(self, 'priorityQueue'):
            self.priorityQueue.clear()

    def goalTest(self, x, y):
        if x == self.currentTarget_x and y == self.currentTarget_y:
            return True
        else:
            return False

    def generatePathToRootNode(self, x, y, outputFilePath):
        pathString = ""
        while (x, y) != (None, None):
            pathString = "{a},{b} ".format(a=y, b=x) + pathString
            x, y = self.nodeToParentMap[(x, y)]
        pathString = pathString.strip()
        outputFilePath.write(pathString+"\n")

    def findPath(self, executeAlgorithm):
        outputFilePath = open("output.txt", "w")
        for target_y, target_x in self.targetList:
            self.currentTarget_x = target_x
            self.currentTarget_y = target_y
            print("*"*30)
            print("Target : (", self.currentTarget_x, self.currentTarget_y,")")

            if executeAlgorithm.lower() == "BFS".lower():
                result = self.bfs_algorithm()
            elif executeAlgorithm.lower() == "UCS".lower():
                result = self.ucs_algorithm()
            elif executeAlgorithm.lower() == "A*".lower():
                result = self.astar_algorithm()

            print(result)
            if result:
                self.generatePathToRootNode(self.currentTarget_x, self.currentTarget_y, outputFilePath)
            else:
                outputFilePath.writelines("FAIL\n")
            self.cleanup()

    def bfs_algorithm(self):
        landingNode = (self.landing_x, self.landing_y)
        bfsQueue = [landingNode]
        self.visitedNodeSet.add(landingNode)
        self.nodeToParentMap[landingNode] = (None, None)
        while len(bfsQueue) != 0:
            currentNode = bfsQueue.pop(0)

            if self.goalTest(currentNode[0], currentNode[1]):
                return True
            else:
                eligibleChildren = self.discoverChildrenForBFS(currentNode)
                for eachChild in eligibleChildren:
                    bfsQueue.append(eachChild)
        return False

    def discoverChildrenForBFS(self, currentNode):
        childrenList = []
        currentElevation = self.elevationMap[currentNode[0]][currentNode[1]]

        # Limits to check for boundary nodes
        lowest_x = currentNode[0] - 1
        lowest_y = currentNode[1] - 1
        highest_x = currentNode[0] + 1
        highest_y = currentNode[1] + 1
        north = (lowest_x, currentNode[1])
        south = (highest_x, currentNode[1])
        west = (currentNode[0], lowest_y)
        east = (currentNode[0], highest_y)
        northwest = (lowest_x, lowest_y)
        southeast = (highest_x, highest_y)
        southwest = (highest_x, lowest_y)
        northeast = (lowest_x, highest_y)

        if lowest_x > -1 and lowest_y > -1 and northwest not in self.visitedNodeSet:
            self.visitedNodeSet.add(northwest)
            if abs(currentElevation - self.elevationMap[northwest[0]][northwest[1]]) <= self.maxElevation:
                self.nodeToParentMap[northwest] = currentNode
                childrenList.append(northwest)

        if lowest_x > -1 and north not in self.visitedNodeSet:
            self.visitedNodeSet.add(north)
            if abs(currentElevation - self.elevationMap[north[0]][north[1]]) <= self.maxElevation:
                self.nodeToParentMap[north] = currentNode
                childrenList.append(north)

        if lowest_x > -1 and highest_y < self.columns and northeast not in self.visitedNodeSet:
            self.visitedNodeSet.add(northeast)
            if abs(currentElevation - self.elevationMap[northeast[0]][northeast[1]]) <= self.maxElevation:
                self.nodeToParentMap[northeast] = currentNode
                childrenList.append(northeast)

        if highest_y < self.columns and east not in self.visitedNodeSet:
            self.visitedNodeSet.add(east)
            if abs(currentElevation - self.elevationMap[east[0]][east[1]]) <= self.maxElevation:
                self.nodeToParentMap[east] = currentNode
                childrenList.append(east)

        if highest_x < self.rows and highest_y < self.columns and southeast not in self.visitedNodeSet:
            self.visitedNodeSet.add(southeast)
            if abs(currentElevation - self.elevationMap[southeast[0]][southeast[1]]) <= self.maxElevation:
                self.nodeToParentMap[southeast] = currentNode
                childrenList.append(southeast)

        if highest_x < self.rows and south not in self.visitedNodeSet:
            self.visitedNodeSet.add(south)
            if abs(currentElevation - self.elevationMap[south[0]][south[1]]) <= self.maxElevation:
                self.nodeToParentMap[south] = currentNode
                childrenList.append(south)

        if highest_x < self.rows and lowest_y > -1 and southwest not in self.visitedNodeSet:
            self.visitedNodeSet.add(southwest)
            if abs(currentElevation - self.elevationMap[southwest[0]][southwest[1]]) <= self.maxElevation:
                self.nodeToParentMap[southwest] = currentNode
                childrenList.append(southwest)

        if lowest_y > -1 and west not in self.visitedNodeSet:
            self.visitedNodeSet.add(west)
            if abs(currentElevation - self.elevationMap[west[0]][west[1]]) <= self.maxElevation:
                self.nodeToParentMap[west] = currentNode
                childrenList.append(west)

        return childrenList

    def ucs_algorithm(self):
        landingNode = (self.landing_x, self.landing_y)
        self.priorityQueue = [(0, landingNode)]
        self.visitedNodeSet.add(landingNode)
        self.nodeToParentMap[landingNode] = (None, None)

        while len(self.priorityQueue) != 0:
            currentNodeCost, currentNode = heappop(self.priorityQueue)

            if self.goalTest(currentNode[0], currentNode[1]):
                return True
            else:
                self.discoverChildren(currentNodeCost, currentNode, isUCS=True)
                self.visitedNodeSet.add(currentNode)
                heapify(self.priorityQueue)
                print(self.priorityQueue)
                print("-"*20)
        return False

    def childEvaluationForUCS(self, childCoords, currentElevation, currentNodeCost, currentNode, costFactor = 10):
        if abs(currentElevation - self.elevationMap[childCoords[0]][childCoords[1]]) <= self.maxElevation:
            latestCost = currentNodeCost + costFactor
            childExistInQueue = False
            for costInQueue, eachChild in self.priorityQueue:
                if childCoords == eachChild:
                    childExistInQueue = True
                    if latestCost < costInQueue:
                        self.priorityQueue.remove((costInQueue, childCoords))
                        self.priorityQueue.append((latestCost, childCoords))
                        self.nodeToParentMap[childCoords] = currentNode
                    break
            if not childExistInQueue:
                self.priorityQueue.append((latestCost, childCoords))
                self.nodeToParentMap[childCoords] = currentNode

    def astar_algorithm(self):
        landingNode = (self.landing_x, self.landing_y)
        self.priorityQueue = [(0, landingNode)]
        self.visitedNodeSet.add(landingNode)
        self.nodeToParentMap[landingNode] = (None, None)

        while len(self.priorityQueue) != 0:
            currentNodeCost, currentNode = heappop(self.priorityQueue)
            if self.goalTest(currentNode[0], currentNode[1]):
                return True
            else:
                self.discoverChildren(currentNodeCost, currentNode, isAStar=True)
                self.visitedNodeSet.add(currentNode)
                heapify(self.priorityQueue)
        return False

    def childEvaluationForAStar(self, childCoords, currentElevation, currentNodeCost, currentNode, costFactor = 10):
        childElevation = self.elevationMap[childCoords[0]][childCoords[1]]
        if abs(currentElevation - childElevation) <= self.maxElevation:
            costToReachChild = (currentNodeCost + costFactor + abs(currentElevation - childElevation))
            delta_x = abs(childCoords[0] - self.currentTarget_x)
            delta_y = abs(childCoords[1] - self.currentTarget_y)
            estimatedCostToReachGoalNode = (14 * min(delta_x, delta_y)) + (10 * abs(delta_x - delta_y))
            latestCost = costToReachChild + estimatedCostToReachGoalNode
            childExistInQueue = False
            for costInQueue, eachChild in self.priorityQueue:
                if childCoords == eachChild:
                    childExistInQueue = True
                    if latestCost < costInQueue:
                        self.priorityQueue.remove((costInQueue, childCoords))
                        self.priorityQueue.append((costToReachChild, childCoords))
                        self.nodeToParentMap[childCoords] = currentNode
                    break
            if not childExistInQueue:
                self.priorityQueue.append((costToReachChild, childCoords))
                self.nodeToParentMap[childCoords] = currentNode

    def discoverChildren(self, currentNodeCost, currentNode, isUCS=False, isAStar=False):
        currentElevation = self.elevationMap[currentNode[0]][currentNode[1]]

        lowest_x = currentNode[0] - 1
        lowest_y = currentNode[1] - 1
        highest_x = currentNode[0] + 1
        highest_y = currentNode[1] + 1

        north = (lowest_x, currentNode[1])
        south = (highest_x, currentNode[1])
        west = (currentNode[0], lowest_y)
        east = (currentNode[0], highest_y)
        northeast = (lowest_x, highest_y)
        northwest = (lowest_x, lowest_y)
        southeast = (highest_x, highest_y)
        southwest = (highest_x, lowest_y)

        if lowest_x > -1 and north not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(north, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                self.childEvaluationForAStar(north, currentElevation, currentNodeCost, currentNode, costFactor=10)

        if highest_x < self.rows and south not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(south, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                self.childEvaluationForAStar(south, currentElevation, currentNodeCost, currentNode, costFactor=10)

        if lowest_y > -1 and west not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(west, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                self.childEvaluationForAStar(west, currentElevation, currentNodeCost, currentNode, costFactor=10)

        if highest_y < self.columns and east not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(east, currentElevation, currentNodeCost, currentNode, costFactor=10)
            elif isAStar:
                self.childEvaluationForAStar(east, currentElevation, currentNodeCost, currentNode, costFactor=10)

        if lowest_x > -1 and lowest_y > -1 and northwest not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(northwest, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                self.childEvaluationForAStar(northwest, currentElevation, currentNodeCost, currentNode, costFactor=14)

        if lowest_x > -1 and highest_y < self.columns and northeast not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(northeast, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                self.childEvaluationForAStar(northeast, currentElevation, currentNodeCost, currentNode, costFactor=14)

        if highest_x < self.rows and highest_y < self.columns and southeast not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(southeast, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                self.childEvaluationForAStar(southeast, currentElevation, currentNodeCost, currentNode, costFactor=14)

        if highest_x < self.rows and lowest_y > -1 and southwest not in self.visitedNodeSet:
            if isUCS:
                self.childEvaluationForUCS(southwest, currentElevation, currentNodeCost, currentNode, costFactor=14)
            elif isAStar:
                self.childEvaluationForAStar(southwest, currentElevation, currentNodeCost, currentNode, costFactor=14)


if __name__ == "__main__":
    inputFilePath = open("input.txt", "r")
    algorithm = inputFilePath.readline().strip()
    marsPathFinderObj = MarsPathFinder(inputFilePath)
    marsPathFinderObj.findPath(algorithm)
