import math


class FaceTracker:
    DEAD_ZONE_ANGLE = math.pi/40 # in radians
    INTERLENS_DISTANCE = 0.065 # in meters
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
        if math.abs(angle) <= FaceTracker.DEAD_ZONE_ANGLE:
            return 0
        else:
            return angle

    def _calculate_angle(self, x, z):
        scene_width = self._calculate_scene_width(z)
        m = scene_width/2
        d = FaceTracker.INTERLENS_DISTANCE
        if x <= m + d/2:
            x_gamma = m + d/2 - x
            rotation = 1
        else:
            x_gamma = x - (m + d/2)
            rotation = -1
        gamma = math.atan(x_gamma/z) * rotation

        return gamma

    def _calculate_scene_width(self, z):
        return math.tan(FaceTracker.FOV/2) * z * 2
