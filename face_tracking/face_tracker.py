import math


class FaceTracker:
    DEAD_ZONE_ANGLE = 5

    """
    Args:
        d - the distance between lenses
        width - the horizontal resolution of the camera
        fov - the field of view of the camera
    """
    def __init__(self, d, width, fov):
        self.d = d
        self.width = width
        self.fov = fov

    """
        Calculates the angle to rotate the camera
        so that its center axis will be aligned
        with the object. It's positive when
        rotated clockwise, and negative otherwise.

        It implements a dead zone angle equal to DEAD_ZONE_ANGLE deegres.
        This means that if the angle between the camera and
        the object is less than or equal DEAD_ZONE_ANGLE
        degrees then the returned angle will be 0.

        p_left - coordinates (x, y) of the object
            wrt. the left lens
        p_right - coordinates (x, y) of the object
            wrt. the right lens
        z - the distance between the camera and the object  
    """
    def calculate_rotation_angle(self, p_left, p_right, z):
        angle = self._calculate_angle(p_left, p_right, z)
        if math.abs(angle) <= FaceTracker.DEAD_ZONE_ANGLE:
            return 0
        else:
            return angle

    def _calculate_angle(self, p_left, p_right, z):
        if p_left is None:
            p = p_right
            rotation = -1
            real_width = math.tan(self.fov/2)*z
            x_real = real_width/self.width * p[0]
            x_alpha = math.abs(x_real - self.width/2)
            x_gamma = x_alpha + self.d/2
            gamma = math.atan(x_gamma/z)

            return rotation*gamma

        x_left_real = real_width/self.width * p_left[0]
        real_width = math.tan(self.fov/2)*z
        if x_left_real < real_width/2:
            x_gamma = real_width/2 - x_left_real + self.d/2
            rotation = 1
        elif x_left_real < real_width/2 + self.d/2:
            x_gamma = self.d/2 - (x_left_real - x_left_real/2)
            rotation = 1
        else:
            x_gamma = x_left_real - (real_width/2 + self.d/2)
            rotation = -1
        
        gamma = math.atan(x_gamma/z)

        return rotation*gamma
