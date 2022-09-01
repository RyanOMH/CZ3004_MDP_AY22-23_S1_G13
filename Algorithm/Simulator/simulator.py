import pygame
from abc import ABC, abstractmethod
from typing import List
from Map.obstacle import Obstacle
from Map.grid import Grid
from Settings.config import *
from Settings.colors import *
from Robot.robot import Robot
from GUI.button import Button


# Load button images
start_img = pygame.image.load("Assets/start_btn.png").convert_alpha()
exit_img = pygame.image.load("Assets/exit_btn.png").convert_alpha()

class AlgoApp(ABC):
    def __init__(self, obstacles: List[Obstacle]):
        self.grid = Grid(obstacles)
        self.robot = Robot(self.grid)
        
        self.start_button = Button(1000, 0, start_img, 0.7)
        self.exit_button = Button(1000, 100, exit_img, 0.7)

    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def execute(self):
        pass


class AlgoSimulator(AlgoApp):
    """
    Run the algorithm using a GUI simulator.
    """
    def __init__(self, obstacles: List[Obstacle]):
        super().__init__(obstacles)

        self.running = False
        self.size = self.width, self.height = WINDOW_SIZE
        self.screen = self.clock = None

    def init(self):
        """
        Set initial values for the app.
        """
        pygame.init()
        self.running = True

        self.screen = pygame.display.set_mode(self.size, pygame.FULLSCREEN)  # pygame.HWSURFACE | pygame.DOUBLEBUF pygame.RESIZABLE
        self.clock = pygame.time.Clock()

        # Inform user that it is finding path...
        pygame.display.set_caption("Calculating path...")
        font = pygame.font.SysFont("arial", 35)
        text = font.render("Calculating path...", True, WHITE)
        text_rect = text.get_rect()
        text_rect.center = WINDOW_SIZE[0] / 2, WINDOW_SIZE[1] / 2
        self.screen.blit(text, text_rect)
        pygame.display.flip()

    def settle_events(self):
        """
        Process Pygame events.
        """
        for event in pygame.event.get():
            # On quit, stop the game loop. This will stop the app.
            if event.type == pygame.QUIT:
                self.running = False

    def do_updates(self):
        self.robot.update()

    def render(self):
        """
        Render the screen.
        """
        self.screen.fill(WHITE, None)

        self.grid.draw(self.screen)
        self.robot.draw(self.screen)

        # Draw start and exit buttons
        if self.start_button.draw():
            # Calculate the path.
            self.robot.brain.plan_path()
            pygame.display.set_caption("Simulating path!")  # Update the caption once done.

        if self.exit_button.draw():
            self.running = False

        # Really render now.
        pygame.display.flip()

    def execute(self):
        """
        Initialise the app and start the game loop.
        """
        while self.running:
            # Check for Pygame events.
            self.settle_events()
            # Do required updates.
            self.do_updates()

            # Render the new frame.
            self.render()

            self.clock.tick(FRAMES)


class AlgoMinimal(AlgoApp):
    """
    Minimal app to just calculate a path and then send the commands over.
    """
    def __init__(self, obstacles):
        # We run it as a server.
        super().__init__(obstacles)

    def init(self):
        pass

    def execute(self):
        # Calculate path
        print("Calculating path...")
        self.robot.brain.plan_path()
        print("Done!")