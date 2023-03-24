class Sphere(object):
    """
    An object repersenting the ping pong ball / tennis ball
    """
    def __init__(self, position, radius):
        self.__position = position
        self.__radius = radius

    def update_position(self, newpos):
        postn = (newpos[0], newpos[1], self.__position[2], self.__position[3])
        self.__position = postn

    def get_position(self):
        return self.__position

    def get_radius(self):
        return self.__radius


class PingPongBall(Sphere):
    def __init__(self, position):
        average_ping_pong_ball_radii = 4 #cm 1.57 inches
        super().__init__(position, average_ping_pong_ball_radii)

    
class TennisBall(Sphere):
    def __init__(self, position):
        average_tennis_ball_radii = 6.7 #cm
        super().__init__(position, average_tennis_ball_radii)

