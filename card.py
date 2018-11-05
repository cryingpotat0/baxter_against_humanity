class Card():
    def __init__(self, text, color):
        self.color = color
        self.text = text

    def __str__(self):
        return self.color + ": " + self.text
