import numpy as np
import pygame


class TX16S:

    TARGET_NAME = "OpenTX TX16SMK3 Joystick"

    def __init__(self):

        pygame.init()
        pygame.joystick.init()

        self.joystick = None

        self._connect()


    def _connect(self):

        for i in range(pygame.joystick.get_count()):

            joystick = pygame.joystick.Joystick(i)

            if joystick.get_name() == self.TARGET_NAME:

                self.joystick = joystick
                self.joystick.init()

                print(
                    f"Connected to: "
                    f"{self.joystick.get_name()}"
                )

                print(
                    f"Axes: {self.joystick.get_numaxes()} | "
                    f"Buttons: {self.joystick.get_numbuttons()}"
                )

                return

        raise RuntimeError(
            "TX16S not found."
        )


    def update(self):

        # Update pygame's controller state, call once per simulation frame
        pygame.event.pump()


    def axis(self, index):

        # Read an axis, returns -1.0 to +1.0
        return self.joystick.get_axis(index)


    def button(self, index):

        # Read buttons, returns True and False
        return bool(
            self.joystick.get_button(index)
        )


    def axes(self):

        # Return all axis values
        return np.array([

            self.axis(i)
            for i in range(self.joystick.get_numaxes())

        ])


    def close(self):

        # Disconnect the controller
        pygame.quit()