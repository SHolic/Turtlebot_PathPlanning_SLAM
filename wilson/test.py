import math
import numpy as np
class TurtlebotController():
    def __init__(self):
        
        self.path = [(4, 5), (5, 5), (5, 6), (5, 7), (6, 7), (7, 7), (8, 7), (8, 8)]
       
        self.direction = "up"
        self.path_actions = []
        self.forward_distance = 0.47
        # Mapping from (current_direction, desired_direction) to (action, angle)
        self.direction_changes = {
            ('up', 'right'): ('turn_right', math.pi / 2),
            ('up', 'left'): ('turn_left', math.pi / 2),
            ('down', 'right'): ('turn_left', math.pi / 2),
            ('down', 'left'): ('turn_right', math.pi / 2),
            ('left', 'up'): ('turn_right', math.pi / 2),
            ('left', 'down'): ('turn_left', math.pi / 2),
            ('right', 'up'): ('turn_left', math.pi / 2),
            ('right', 'down'): ('turn_right', math.pi / 2),
            ('up', 'down'): ('turn_right', math.pi),
            ('down', 'up'): ('turn_right', math.pi),
            ('left', 'right'): ('turn_right', math.pi),
            ('right', 'left'): ('turn_right', math.pi),
        }        
        self.generate_path()


    def generate_path(self):
        for i in range(len(self.path) - 1):
            current_pos = self.path[i]
            next_pos = self.path[i + 1]
            dx, dy = next_pos[1] - current_pos[1], next_pos[0] - current_pos[0]
            desired_direction = self.calculate_desired_direction(dx, dy)
            
            if self.direction != desired_direction:
                action, angle = self.direction_changes[(self.direction, desired_direction)]
                self.path_actions.append((action, angle, current_pos, desired_direction))
                self.direction = desired_direction
            
            self.path_actions.append(("forward", self.forward_distance, next_pos, self.direction))
        
        self.path_actions.append(("stop", 0, current_pos, self.direction))

    def calculate_desired_direction(self, dx, dy):
        if dx == 1:
            return 'right'
        elif dx == -1:
            return 'left'
        elif dy == 1:
            return 'down'
        elif dy == -1:
            return 'up'



    

def main(args=None):
     node = TurtlebotController()
     print(node.direction)
     print(node.path_actions)

if __name__ == '__main__':
    main()



