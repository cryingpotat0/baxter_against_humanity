from player import Player
import time

class State():
    def __init__(self):
        self.robot_player = 0

    def get_initial_state(self):
        # TODO: look around here and count num of players, assign each of them ids/ any identifying info, set self.curr_player to starting player
        # test data
        self.num_players = 5
        self.curr_player = 0 # 0 is robot player
        self.players = []
        for id in range(self.num_players + 1):
            player = Player(id)
            self.players.append(player)

    def increment_player(self):
        self.curr_player = (self.curr_player + 1) % (self.num_players + 1)

    def is_robot_turn(self):
        return self.curr_player == self.robot_player

    def wait_for_white_cards(self):
        # DO PERCEPTION STUFF HERE, SET TIMEOUT AFTER WHICH WE PLAY ANYWAY
        time.sleep(2)
        return True

    def wait_for_black_card(self):
        # DO PERCEPTION STUFF HERE
        time.sleep(2)
        return True
