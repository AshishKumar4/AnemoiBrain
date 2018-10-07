from ImageProcessing.SpatialMaps import *

# Initial Pos Vector serves as origin vector, every other vector is in respect to this

class Navigator:
    def __init__(self, gmap = None, target = None):
        # Put all the Initialization Code here
        self.sp = SpatialMap()
        self.targetVector = None
        self.gmap = gmap
        self.targetPos = target
        return None 

    def SystemRun(self):
        # The Actual Navigation System Execution
        # Conduct Initial Path Planning
        self.plan = self.generalPathPlanning(self.gmap, self.getCurrentPos(), self.targetPos)

        # BreakDown Paths and choose first local path target
        self.planset = self.breakDownPaths(self.plan)

        # This target should be at line of sight
        initialPosVector = self.planset[0]['initialVector']

        for i in self.planset:
            # Get Local Target vector assuming the initial positional vector as origin
            ltargetVector = self.getVectorRelative(i['targetVector'], initialPosVector)

            # Rotate the drone to face in its direction
            self.rotateToVec(ltargetVector)

            while True:
                # Construct 3D map of frontal surroundings.
                self.map = self.sp.generate3DMap()

                # find the windows in the surroundings.
                # Choose the closest window which has the most chance of leading towards the target.
                window, vector = self.sp.findBestWindow(self.map, self.getCurrentPos(), ltargetVector)
                vector = self.getProjectionalVector(vector, ltargetVector)

                # Move towards the target through the window until next obstruction.
                self.moveInVector(vector)

                ltargetVector = self.getVectorTransform(initialPosVector, vector, ltargetVector)

                # Rotate in this direction
                self.rotateToVec(ltargetVector)

                # Update the Initial Positional Vector with the current positional vector
                initialPosVector = self.getCurrentPos()  # We hope that the errors in position is low

                # All Positional errors should be managed by the underlying abstraction layers i.e, the lower level code
                pass
        return True

    def getProjectionalVector(self, directionalVec, targetVec):
        # Given a Directional Vector of any length, return a vector with the projection of it on the target vector
        return None

    def getVectorRelative(self, tvec, origin):
        # Return a transformed vector of the tvec with respect to origin vector
        return None

    def getVectorTransform(self, posVec, transformVec, targetVec):
        # Transform a target vector based on the transformation vector and the initial vector and the target vector to return the new target vector
        return None

    def getCurrentPos(self):
        # Return the current positional vector
        return None

    def moveInVector(self, vector):
        # Given a vector, move in the direction of its end.
        return None 

    def rotateToVec(self, vector):
        # Given a vector, Rotate in that direction
        return None

    def generalPathPlanning(self, map, cpos, fpos):
        # Return an optimal path given a 3D low res map of the enviornment and current position and final position
        return None

    def breakDownPaths(self, plan):
        # Return a set of local target oriented paths by dividing the global paths.
        g = list()
        # Each element in this list should have 1. targetVector, 2. InitialVector 3. Distance
        return g