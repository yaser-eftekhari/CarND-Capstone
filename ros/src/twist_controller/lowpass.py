
class LowPassFilter(object):
    def __init__(self, old_w, new_w):
        self.a = new_w / (old_w + new_w)
        self.b = old_w / (old_w + new_w)

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
