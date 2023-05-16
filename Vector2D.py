class Vector2D:

    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)
        self.coord = (self.x, self.y)

    def move(self, x_move, y_move):
        self.x += x_move
        self.y += y_move

    def __str__(self):
        return f'x, y = {self.x}, {self.y}'

    def __repr__(self):
        return f'x, y = {self.x}, {self.y}'