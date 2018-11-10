from card import Card
class Action():
    def __init__(self):
        self.cards = []

    def draw_white_card(self):
        # ACTUATION TO PICK UP CARD AND PERCEPTION OF CARD
        card = Card("lmao", "white")
        self.cards.append(card)
        
    def get_black_card(self):
        # ACTUATION TO PICK UP, READ AND PLACE BLACK CARD, returns black card
        card = Card("lmao2", "black")
        return card

    def read_black_card(self):
        # ACTUATION TO CHECK IF BLACK CARD HAS BEEN PLACED, PICK IT UP AND READ IT
        card = Card("lmao3", "black")
        return card

    def read_white_cards(self):
        # pick up and read each white card in area
        return {
                 Card("1", "white"): (0, 0), 
                 Card("2", "white"): (0, 0), 
                 Card("3", "white"): (0, 0)
               }

    def play_card(self, card):
        if card.color == "black":
            pass
        else:
            pass

    def choose_best_white(self, white_cards, black_card):
        # logic to pick best white from black

        best = None
        for card in white_cards:
            best = card
        return best

    def move_best_white(self, best, white_cards):
        # use best coords + 
        best_coords = white_cards[best]
        return True
    
    def choose_white_to_play(self, black_card):
        return self.cards[0]


