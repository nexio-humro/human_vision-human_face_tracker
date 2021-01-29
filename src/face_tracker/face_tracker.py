import math


class FaceTracker:
    DEAD_ZONE_ANGLE = 0 # in radians
    INTERLENS_DISTANCE = 0.12 # in meters
    FOV = math.pi/4 # fov of the lens

    """
        Calculates the angle to rotate the camera
        so that its center axis will be aligned
        with the object. It's positive when
        rotated clockwise, and negative otherwise.

        It implements a dead zone angle equal to DEAD_ZONE_ANGLE deegres.
        What it does is that if the angle between the camera and
        the object is less than or equal DEAD_ZONE_ANGLE
        then the returned angle will be 0.

        x, z - the coordinates of the object
    """
    def calculate_rotation_angle(self, x, z):
        angle = self._calculate_angle(x, z)
        if abs(angle) <= FaceTracker.DEAD_ZONE_ANGLE:
            return 0
        else:
            return angle

    def _calculate_angle(self, x, z):
        d = FaceTracker.INTERLENS_DISTANCE
        if x <= d/2:
            x_gamma = -x + d/2
            rotation = 1
        else:
            x_gamma = x - d/2
            rotation = -1
        gamma = math.atan(x_gamma/z) * rotation

        return gamma

    def _calculate_scene_width(self, z):
        return math.tan(FaceTracker.FOV/2) * z * 2
