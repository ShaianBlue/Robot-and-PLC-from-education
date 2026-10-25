import numpy as np

class Circle:
    def __init__(self, ellipseInformation):
        # The image coordinate of the detected circle
        self.coordinate = (int(ellipseInformation[0]), int(ellipseInformation[1]))
        
        # The axis of the detected circle
        self.minorAxis = ellipseInformation[2] / 2
        self.majorAxis = ellipseInformation[3] / 2

        # The angle of the detected circle
        self.angle = ellipseInformation[4]
        
        # The type of the detected circle
        # ("B" for blue, "R" for red, "W" for white, "G" for golf and "H" for hole)
        if ellipseInformation[5] == 0:
            self.type = "G"
        elif ellipseInformation[5] == 1:
            self.type = "R"
        elif ellipseInformation[5] == 2:
            self.type = "W"
        elif ellipseInformation[5] == 3:
            self.type = "B"
        elif ellipseInformation[5] == 4:
            self.type = "H"
        
        # Real-life information for each type
        if self.type == "G":
            self.realDiameter = 0.043
        elif self.type == "H":
            self.realDiameter = 0.052
        else:
            self.realDiameter = 0.048   # This is the height, the alghoritm will use the diameter instead

        # The area of the detected circle
        self.area = np.pi * self.majorAxis * self.minorAxis
        
        # Status for captured ball (False for not captured, True for captured)
        self.captured = False

    # Function calculating where the ball has to go
    def go_to(self, deliver):
        if self.captured:
            if self.type == "G":
                # Deliver to the hole
                pass
            elif self.type == "R":
                # Deliver to "B" with ArUco marker 10 or 11
                pass
            elif self.type == "W":
                # Deliver to any with ArUco marker 10, 11, 12, 13, 14, 15, 16 or 17
                pass
            elif self.type == "B":
                # Deliver to "C" with ArUco marker 14 or 15
                pass
            self.coordinate = deliver
        else:
            if self.type == "H":
                print("This is a hole")
            else:
                print("Ball not captured")
