import armbot as ab

import time
import math
import threading

class ShapeDetector:
    def __init__(self):
        self.shape_thread = threading.Thread(target=self.shape_picker)
        self.current_colour = "blue"
        self.current_shape = "rectangle"

        self.exit_selected = False
        self.colour_selected = False

        self.colours = ["red", "blue", "green", "lilac"]
        self.shapes = ["triangle", "rectangle", "star", "circle"]

    def start_shape_picker(self):
        self.shape_thread.start()

    def shape_picker(self):
        state_machine = {
                    0: ["Start", self.start_interface, (lambda: self.colour_selected,), (1, 0)],
                    1: ["Colour Picker", self.select_colour, (lambda: self.exit_selected, lambda: self.colour_selected), (0, 2, 1)],
                    2: ["Shape Picker", self.select_shape, (lambda: self.exit_selected, lambda: self.colour_selected), (0, 1, 2)],
                    }

        state_index = 0
        exit_flag = False

        while exit_flag == False:
            current_state = state_machine[state_index]

            name = current_state[0];
            print("\nCurrent state:", name);

            action = current_state[1]
            
            action()

            for index, condition in enumerate(current_state[2] + (lambda: True,)):
                if condition() == True:
                    state_index = current_state[3][index]
                    break;
    
    def clear_conditions(self):
        self.exit_selected = False
        self.colour_selected = False

        print("\nCurrent Colour:", self.current_colour)
        print("\nCurrent Shape:", self.current_shape)
        
    def start_interface(self):
        self.clear_conditions()

        # Read user input
        print("\n################ SHAPE PICKER ################")
        userinput = input("ENTER \"C\" TO CONTINUE:")

        if userinput == "c" or userinput == "C":
            self.colour_selected = True

    def select_colour(self):
        self.clear_conditions()

        # Read user input
        print("\n################ SELECT COLOUR ################")
        userinput = input("\nEnter desired colour to be detected\n(r/R -> red, b/B -> blue, g/G -> green, l/L -> lilac)\n\nor \"E\" to exit:")

        colourInput = {"R": "red", "B": "blue", "G": "green", "L": "lilac",
                        "r": "red", "b": "blue", "g": "green", "l": "lilac"}

        if userinput == "E" or userinput == "e":
            self.exit_selected = True
        # If the user input is in colourInput, change current colour
        elif userinput in colourInput:
            self.current_colour = colourInput.get(userinput) # Get associated value in colourInput
            self.colour_selected = True
        else:
            print("Incorrect input.\n")

    def select_shape(self):
        self.clear_conditions()

        # Read user input
        print("\n################ SELECT SHAPE ################")
        userinput = input("\nEnter desired shape to be detected\n(r/R -> rectangle, t/T -> triangle, s/S -> star, o/O -> circle), \n\nelse enter \n\"C\" to pick colour again \nor \"E\" to exit:")

        shapeInput = {"R": "rectangle", "T": "triangle", "S": "star", "O": "circle",
                        "r": "rectangle", "t": "triangle", "s": "star", "o": "circle"}

        if userinput == "E" or userinput == "e":
            self.exit_selected = True
        elif userinput == "C" or userinput == "c":
            self.colour_selected = True

        # If the user input is in colourInput, change current colour
        elif userinput in shapeInput:
            self.current_shape = shapeInput.get(userinput) # Get associated value in colourInput

        else:
            print("Incorrect input.\n")

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        arm = ab.ArmBot("flashytokyobakerpt410");
        sd = ShapeDetector()

        arm.start_image_acquisition(show_feed=True)
        sd.start_shape_picker()

        while(True):
            image = arm.get_current_image()

            if image is not None:
                mask, masked_image = arm.detect_colour(image, sd.current_colour, frame_name="Colour Mask", show_frame=False)
                shapes, mask = arm.detect_shapes(mask, sd.current_shape)

                masked_image = arm.apply_mask(image, mask)

                for shape in shapes:
                    arm.draw_shape(masked_image, shape)

                arm.show_image("Shapes", masked_image, continuous=True)

                # masked_image = arm.apply_mask(image, mask)
                # arm.show_image("Shape Mask", masked_image, continuous=True)

    except KeyboardInterrupt:
        print('Interrupted!')