

class ThetaStarRRT:
    '''Calculation of a Path using RRT based on existing A-Star solution.'''
    path_a_star = None
    
    def __init__(self, path_a_star):
        '''Constructor
        :param path_a_star: The path pre-calculated by A-Star
        '''
        self.path_a_star = path_a_star
        
    def calculate_path(self, max_iterations=10000):
        
        