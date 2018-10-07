

class SpatialMap:
    def __init__(self):
        return None 

    # There are two types of map formats ->
    #   1. Stack maps that have 2D slices w.r.t altitudes
    #   2. Onion maps that have spherical slices w.r.t radial distance, like layers of onion

    def generate3DMap(self, center = None):
        # Generate 3D Map of enviornment as 2D slices w.r.t altitude.
        return None

    def onionToStackMap(self, omap):
        # Take as an input Onion map and return a Stack map
        return None 

    def stackToOnionMap(self, smap):
        # Take as an input Stack map and return an Onion map
        return None

    def getStackLayer(self, smap, alt):
        # Take as an input altitude and return a stack 2D map
        return None

    def getOnionLayer(self, omap, radius):
        # Take as an input Onion map and return a radial spherical slice
        return None

    def onionLayerRasterize(self, olayer, center = None):
        # Take as an input onion layer (spherical slice) and project it onto a plane w.r.t center point and return
        return None

    def get2DBirdEye(self, map, alt):
        # Return 2D slice of 3D Map w.r.t alt
        smap = map['smap']
        return self.getStackLayer(smap, alt)

    def get2DFrontal(self, map, dis):
        # return 2D slice of 2D Map w.r.t radial distance from camera, facing the camera. this slice is 
        # Basically 2D projection of a spherical slice of 3D map.
        omap = map['omap']
        return self.onionLayerRasterize(self.getOnionLayer(omap, dis))

    def findWindows(self, map):
        # Given a map, identify all the possible windows
        return None

    def rankWindows(self, windows, cpos, tvector):
        # Rank all the windows based on target direction vector and distance to reach there based on current position vector
        return None

    def findBestWindow(self, map, cpos, tvector):
        # Given a map, current position and target vector, get the best window and directional vector
        return None