from state import State
from action import Action

NUM_CARDS_PER_PLAYER = 7
class Simulator():
    def __init__(self, state_node, action_node):
        self.state = state_node
        self.action = action_node
        self.state.get_initial_state()
        for _ in range(NUM_CARDS_PER_PLAYER):
            self.action.draw_white_card()

    def step(self):
        if self.state.is_robot_turn():
            print("ROBOT TURN")
            black_card = self.action.get_black_card()
            print(black_card)
            self.action.play_card(black_card)
            self.state.wait_for_white_cards()
            white_cards = self.action.read_white_cards()
            for card in white_cards:
                print(card)
            best = self.action.choose_best_white(white_cards, black_card)
            print("best, ", card)
            self.action.move_best_white(best, white_cards)
        else:
            print("SOMEONE ELSE'S TURN")
            self.state.wait_for_black_card()
            black_card = self.action.read_black_card()
            print(black_card)
            white_card = self.action.choose_white_to_play(black_card)
            print(white_card)
            self.action.play_card(white_card)
            self.action.draw_white_card()
        self.state.increment_player()

if __name__ == "__main__":
    s = Simulator(State(), Action())
    while True:
        s.step()
