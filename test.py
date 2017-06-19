

class A:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return 'X = %.3f    Y = %.3f    Z = %.3f' % (self.x, self.y, self.z)

param = A(1, 2, 3)
print(param)