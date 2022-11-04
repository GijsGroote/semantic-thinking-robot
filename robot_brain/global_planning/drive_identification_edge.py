from robot_brain.global_planning.edge import Edge

class DriveIdentificationEdge(Edge):



    def __init__(self, iden, source, to, verb, controller, path=False):

        Edge.__init__(self, iden, source, to, verb, controller, path)
        print("creaetd an driveidentificatiedge")



    def to_string(self):
        return f"iden: {self.iden}, controller: {self.controller.name}"
