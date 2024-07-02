import armbot as ab

import time
import math
import threading

class HSVColourDetector:
    def __init__(self):
        self.colour_thread = threading.Thread(target=self.colour_picker)
        self.current_colour = "blue"
        self.current_parameter = ""
        self.current_range = ""
        self.current_hsv_value = 0

        self.exit_selected = False
        self.colour_selected = False
        self.parameter_selected = False
        self.range_selected = False

        self.colours = ["red", "blue", "green", "lilac"]
        self.parameters = ["hue", "saturation", "value"]
        self.ranges = ["low", "high"]

    def start_colour_picker(self):
        self.colour_thread.start()

    def colour_picker(self):
        state_machine = {
                    0: ["Start", self.start_interface, (lambda: self.colour_selected,), (1, 0)],
                    1: ["Colour Picker", self.select_colour, (lambda: self.exit_selected, lambda: self.colour_selected), (0, 2, 1)],
                    2: ["Parameter Picker", self.select_parameter, (lambda: self.exit_selected, lambda: self.colour_selected, lambda: self.parameter_selected), (0, 1, 3, 2)],
                    3: ["Range Picker", self.select_range, (lambda: self.exit_selected, lambda: self.colour_selected, lambda: self.parameter_selected, lambda: self.range_selected), (0, 1, 2, 4, 3)],
                    4: ["Value Chooser", self.change_value, (lambda: self.exit_selected, lambda: self.colour_selected, lambda: self.parameter_selected, lambda: self.range_selected), (0, 1, 2, 3, 4)],
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
        self.parameter_selected = False
        self.range_selected = False

        print("\nCurrent Colour:", self.current_colour)
        print("\nCurrent Parameter:", self.current_parameter)
        print("\nCurrent Range:", self.current_range)
        print("\nCurrent Value:", self.current_hsv_value)
        
    def start_interface(self):
        self.clear_conditions()

        # Read user input
        print("\n################ COLOUR PICKER & HSV VALUE SELECTOR ################")
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

    def select_parameter(self):
        self.clear_conditions()

        # Read user input
        print("\n################ SELECT HSV PARAMETER TO CHANGE ################")
        userinput = input("\nEnter desired HSV colour parameter to change\n(H -> hue, S -> saturation, V -> brightness value),\n\nelse enter \n\"C\" to pick colour again \nor \"E\" to exit:")

        parameterInput = {"H": "hue", "S": "saturation", "V": "value",
                          "h": "hue", "s": "saturation", "v": "value"}

        if userinput == "E" or userinput == "e":
            self.exit_selected = True
        elif userinput == "C" or userinput == "c":
            self.colour_selected = True

        # If the user input is in parameterInput, change current paremter
        elif userinput in parameterInput:
            self.current_parameter = parameterInput.get(userinput) # Get associated value in parameterInput
            self.parameter_selected = True

        else:
            print("Incorrect input.\n")

    def select_range(self):
        self.clear_conditions()

        # Read user input
        print("\n######### SELECT WHAT END OF THE PARAMETER RANGE TO CHANGE #########")
        userinput = input("\nEnter desired range end to change for the selected parameter\n(H -> high, L -> low),\n\nelse enter \n\"P\" to pick parameter again, \n\"C\" to pick colour again \nor \"E\" to exit:")

        rangeInput = {"H": "high", "L": "low",
                      "h": "high", "l": "low"}

        if userinput == "E" or userinput == "e":
            self.exit_selected = True
        elif userinput == "C" or userinput == "c":
            self.colour_selected = True
        elif userinput == "P" or userinput == "p":
            self.parameter_selected = True

        # If the user input is in rangeInput, change current range
        elif userinput in rangeInput:
            self.current_range = rangeInput.get(userinput) # Get associated value in rangeInput
            self.range_selected = True

        else:
            print("Incorrect input.\n")

    def change_value(self):
        self.clear_conditions()

        # Read user input
        print("\n################ SELECT PARAMETER VALUE ################")
        userinput = input("\nEnter the desired value for the chosen selections,\n\nelse enter \n\"R\" to pick range again, \n\"P\" to pick parameter again, \n\"C\" to pick colour again \nor \"E\" to exit:")

        if userinput == "E" or userinput == "e":
            self.exit_selected = True
        elif userinput == "C" or userinput == "c":
            self.colour_selected = True
        elif userinput == "P" or userinput == "p":
            self.parameter_selected = True
        elif userinput == "R" or userinput == "r":
            self.range_selected = True

        # If the user input is a number, change the value
        elif userinput.isdigit():
            self.current_hsv_value = int(userinput) # Get number from input

        else:
            print("Incorrect input.\n")
            

# Check if the node is executing in the main path
if __name__ == '__main__':
    try:
        arm = ab.ArmBot("flashytokyobakerpt410");
        cd = HSVColourDetector()

        arm.start_image_acquisition(show_feed=True)
        cd.start_colour_picker()

        previous_hsv_value = cd.current_hsv_value

        while(True):
            if cd.current_hsv_value != previous_hsv_value:
                arm.change_hsv_values(cd.current_colour, cd.current_parameter, cd.current_range, cd.current_hsv_value)
                previous_hsv_value = cd.current_hsv_value

            image = arm.get_current_image()

            if image is not None:
                arm.detect_colour(image, cd.current_colour, frame_name="Colour Mask")

    except KeyboardInterrupt:
        print('Interrupted!')