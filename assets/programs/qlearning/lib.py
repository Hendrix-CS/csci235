from pybricks.tools import wait, StopWatch, DataLog

class QParameters:
    def __init__(self):
        self.pause_ms = 100
        self.actions = []
        self.num_states = 0
        self.state_func = lambda r: 0
        self.reward_func = lambda r, state, action: 0
        self.target_visits = 1
        self.discount = 0.5
        self.rate_constant = 10


def run_q(robot, params, max_steps=None):
    qs = QTable(params)
    loops = 0
    total_reward = 0
    action = 0
    while max_steps is None or loops < max_steps:
        params.actions[action](robot)
        wait(params.pause_ms)
        state = params.state_func(robot)
        reward = params.reward_func(robot, state, action)
        total_reward += reward
        action = qs.sense_act_learn(state, reward)
        robot.show(state, action, reward, total_reward)
        loops += 1

    robot.stop_all()
    while True:
        pass

class QTable:
    def __init__(self, params):
        self.q = [[0.0] * len(params.actions) for i in range(params.num_states)]
        self.visits = [[0] * len(params.actions) for i in range(params.num_states)]
        self.target_visits = params.target_visits
        self.discount = params.discount
        self.rate_constant = params.rate_constant
        self.last_state = 0
        self.last_action = 0
        self.log = None

    def activate_log(self):
        self.log = DataLog('q', 'visits', 'reward', 'new_state', 'new_action')

    def sense_act_learn(self, new_state, reward):
        alpha = self.learning_rate(self.last_state, self.last_action)
        update = alpha * (self.discount * self.q[new_state][self.best_action(new_state)] + reward)
        self.q[self.last_state][self.last_action] *= 1.0 - alpha
        self.q[self.last_state][self.last_action] += update

        self.visits[self.last_state][self.last_action] += 1
        if self.is_exploring(new_state):
            new_action = self.least_visited_action(new_state)
        else:
            new_action = self.best_action(new_state)

        if self.log:
            self.log.log(self.q, self.visits, reward, new_state, new_action)

        self.last_state = new_state
        self.last_action = new_action
        return new_action

    def learning_rate(self, state, action):
        return 1.0 / (1.0 + self.visits[state][action] / self.rate_constant)

    def best_action(self, state):
        best = 0
        for action in range(1, len(self.q[state])):
            if self.q[state][best] < self.q[state][action]:
                best = action
        return best

    def is_exploring(self, state):
        return min(self.visits[state]) < self.target_visits

    def least_visited_action(self, state):
        least_visited = 0
        for action in range(1, len(self.visits[state])):
            if self.visits[state][least_visited] > self.visits[state][action]:
                least_visited = action
        return least_visited
