import math


def pose_error(gt, estimate):
    distance = math.hypot(estimate[0] - gt[0], estimate[1] - gt[1])
    delta = estimate[2] - gt[2]
    angle = abs(math.atan2(math.sin(delta), math.cos(delta)))
    return distance, angle


class Convergence:
    def __init__(self, position_threshold, angle_threshold, confirmation_time):
        self.position_threshold = position_threshold
        self.angle_threshold = angle_threshold
        self.confirmation_time = confirmation_time
        self.since = None
        self.result = None

    def reset(self):
        self.since = None

    def update(self, time, distance, angle):
        if self.result is not None:
            return self.result
        if distance > self.position_threshold or angle > self.angle_threshold:
            self.reset()
        else:
            if self.since is None:
                self.since = time
            if time - self.since >= self.confirmation_time:
                self.result = self.since
        return self.result
