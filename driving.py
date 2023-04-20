
# !/usr/bin/env python3
from dataclasses import dataclass
from enum import Enum

import math
import time

import ev3dev.core
import ev3dev.ev3 as ev3

from copy import deepcopy

from planet import Direction



@dataclass
class Pose:
    x: float = 0  # cm
    y: float = 0  # cm
    angle: float = 0  # rads


# class representing the robot
class Driver:
    def __init__(self):
        # init pose and angle variables
        self.pose = Pose()
        self.correct_angle: float = 0
        self.past_angles: list = []

        # init motors
        self.right = ev3.Motor("outA"); self.right.reset(); self.right.stop_action = "brake"
        self.past_angle_right: float = 0
        self.left = ev3.Motor("outB");  self.left.reset();  self.right.stop_action = "brake"
        self.past_angle_left: float  = 0

        # init motor relevant variables
        self.target_speed = 0
        self.offset = 0

        # init sensors
        self.button       = ev3.TouchSensor("in2")
        self.sonic_sensor = ev3.UltrasonicSensor("in3")
        self.color_sensor = ev3.ColorSensor("in4"); self.color_sensor.mode = 'RGB-RAW'


    def set_target_speed(self, speed: float):
        self.target_speed = speed


    def set_turn_speed(self, speed: float):
        self.offset = speed


    def drive(self):
        self.update_odo()

        # set speeds for the 2 motors
        self.right.speed_sp = -round(self.target_speed + self.offset)  # speed is set to the negative
        self.left.speed_sp = -round(self.target_speed - self.offset)   # because motors are built in the
                                                                       # oposite way

        # need to tell them to start, before the speed updates
        self.right.command = "run-forever"
        self.left.command = "run-forever"


    def stop(self):
        self.right.stop()
        self.left.stop()



    def update_odo(self):
        # angle th mortor gives in in degrees and negative
        # correction for that
        angle_right = - self.right.position / 360 * 2 * math.pi
        angle_left = - self.left.position / 360 * 2 * math.pi

        angle_difference_right = angle_right - self.past_angle_right
        angle_difference_left  = angle_left  - self.past_angle_left

        # calculate the distance traveled by each wheel
        travel_right = angle_difference_right * Driver.WHEEL_RADIUS
        travel_left  = angle_difference_left  * Driver.WHEEL_RADIUS
        travel_mid = (travel_right + travel_left)/2

        driving_strait = self.right.speed_sp == self.left.speed_sp
        if driving_strait:
            travel_average = (travel_right + travel_left) / 2

            self.pose.x += math.cos(self.pose.angle) * travel_average
            self.pose.y += math.sin(self.pose.angle) * travel_average

        if not driving_strait:
            # calculate rotation radius for each wheel
            r_right = Driver.WHEEL_TO_WHEEL * (self.right.speed_sp / (self.right.speed_sp - self.left.speed_sp))
            r_left  = Driver.WHEEL_TO_WHEEL * (self.left.speed_sp / (self.right.speed_sp - self.left.speed_sp))
            r_mid = (r_right + r_left)/2

            # calculate rotation angle from the data of the left and right motors
            # calculate the average for better accuracy. ignore any rotation angle
            # that would be produced by 0 radius rotation
            rotated_angle_right = 0 if r_right == 0 else travel_right / r_right
            rotated_angle_left  = 0 if r_left  == 0 else travel_left  / r_left
            number_of_valid_calculated_angles = 1 if (r_right == 0 or r_left == 0) else 2
            rotated_angle = (rotated_angle_right + rotated_angle_left) / number_of_valid_calculated_angles

            # calculate center of rotation
            center = Pose()
            center.x = self.pose.x - math.sin(self.pose.angle) * r_mid
            center.y = self.pose.y + math.cos(self.pose.angle) * r_mid

            # shift the bot coordinate system, such that the rotation center is the coordinate system orgin for it
            self.pose.x -= center.x
            self.pose.y -= center.y

            # rotate around origin by rotated_angle degrees
            self.pose.x = math.cos(rotated_angle) * self.pose.x + -math.sin(rotated_angle) * self.pose.y
            self.pose.y = math.sin(rotated_angle) * self.pose.x + math.cos(rotated_angle) * self.pose.y

            # shift bot coordinate system back. also now add angle
            self.pose.x += center.x
            self.pose.y += center.y
            self.pose.angle += rotated_angle


        self.past_angle_right = angle_right
        self.past_angle_left  = angle_left

        self.past_angles.insert(0, self.pose.angle)
        if len(self.past_angles) > 5: self.past_angles.pop()




    def get_orientation(self) -> Direction:
        # we report the position 5 ticks ago
        # this is done so that calculated over correction on red/blue doesnt give wrong angle
        degrees = rad_to_deg(self.past_angles[-1])
        rounded_angle = round(degrees / 90) * 90

        return Direction(rounded_angle % 360)


    def correct_orientation(self, orinetation: Direction):
        print(f"  odometrie calculated angle: {rad_to_deg(self.past_angles[-1])}")
        print(f"  odometrie ROUNDED to angle: {round(rad_to_deg(self.past_angles[-1]) / 90) * 90}")
        print(f"  ACTUAL angle: {orinetation}")
        print("-------------------")

        self.correct_angle = deg_to_rad(orinetation.value)


    def update_oriantation(self):
        self.pose.angle = self.correct_angle


    def get_position(self) -> tuple[int, int]:
        x = self.pose.x
        y = self.pose.y
        return Rounding.round_to_nearest_map_coordinate(x, y)



    def correct_position(self, position: tuple[int, int]):
        x = self.pose.x
        y = self.pose.y
        rounded = Rounding.round_to_nearest_map_coordinate(x, y)

        print("-------------------")
        print(f"  odometrie calculated position: {(self.pose.x, self.pose.y)}")
        print(f"  odometrie ROUNDED to position: {(rounded)}")
        print(f"  ACTUAL position: {position}")
        print("+++++++++++++++++++")

        self.pose.x = position[0] * 50
        self.pose.y = position[1] * 50



    # constants of the robot
    WHEEL_DIAMETER = 5.5  # cm
    WHEEL_RADIUS = WHEEL_DIAMETER / 2 # cm
    WHEEL_PERIMETER = WHEEL_DIAMETER * math.pi # cm
    WHEEL_TO_WHEEL = 11  # cm
    WHEEL_TO_LIGHT = 5  # cm
    LIGHT_SENSOR_UPDATE_TIME = 0.025  # s,  determined experimentally. authenticity unknown





# main class, singleton
class Driving:
    @staticmethod
    def change_state(to):
        print(f"DRIVING changes state to: {to}")
        Driving.state = to

    class States(Enum):
        prepare = 0
        turn = 1
        normal = 2
        reverse = 3
        onred = 4
        onblue = 5
        adjust = 6

        callibrate = 99
        done = 100


    state: States = States.prepare
    bot: Driver = Driver()


    # excecutes different functions based on the state
    @staticmethod
    def drive(turn_towards=None):

        if turn_towards is None:
            # initial drive from start point to first
            # does setup code, like loading color calibiretion from file
            Driving.change_state(Driving.States.prepare)
        else:
            # after driven on a node
            # we are told in which direction to turn to and then drive down
            Driving.change_state(Driving.States.turn)

        while True:
            # states relating to preparation
            if Driving.state == Driving.States.callibrate:
                Driving.callibrate()

            if Driving.state == Driving.States.prepare:
                Driving.prepare()

            # states relating to driving related
            if Driving.state is Driving.States.turn:
                Driving.turn(turn_towards)

            if Driving.state is Driving.States.normal:
                Driving.normal()

            if Driving.state is Driving.States.reverse:
                Driving.reverse()

            if Driving.state is Driving.States.adjust:
                Driving.adjust()


            # states giving control back to main
            if Driving.state is Driving.States.onred or Driving.state is Driving.States.onblue:
                Driving.onpoint(); break

            if Driving.state is Driving.States.done:
                Driving.done(); break


    # exceutes the searching function if on a node
    @staticmethod
    def get_directions() -> list[Direction]:
        assert Driving.state is Driving.States.onred or Driving.state is Driving.States.onblue
        return Driving.search()



    @staticmethod
    def prepare():
        button = Driving.bot.button
        temp_button = try_connect_temp_buton()

        which = wait_until_pressed(button, temp_button)

        time.sleep(0.5)

        # if start button is pressed
        if which is button:
            print("start button pressed")
            Color.load_data_from_file()
            Driving.change_state(Driving.States.normal)

        # if the temporary color calibration button is pressed
        if which is temp_button:
            print("temp button pressed")
            Driving.change_state(Driving.States.callibrate)


    # main function for driving behaviour
    # follows the line on the left edge
    @staticmethod
    def normal():
        # avoid having to repeat the prefix over and over again
        bot = Driving.bot

        # init variables
        current_speed = 40
        max_speed = 180

        past_error: float = 0
        error_integral: float = 0

        DAMPENING = 0.8

        while Driving.state is Driving.States.normal:
            # get color and process it
            color_data = bot.color_sensor.bin_data("hhhh")
            current_light_level = Color.to_light_level(color_data)                  # normalized to the range  0 to 1
            current_red_blue_level = Color.to_red_blue_level(color_data)            # normalized to the range -1 to 1


            # calulate error and error integral
            error = current_light_level - Color.target_level                        # normalized to the range -0.5 to 0.5
            error_integral = DAMPENING * error_integral + (1 - DAMPENING) * error   # normalized to the range 0 to error


            # calculate turn speeds of the P I D
            propotional_speed_offset = 0.385 * - current_speed * error
            integral_speed_offset = 0.8755 * -current_speed * error_integral
            differential_speed_offset = 0.555 * -current_speed * (2 * error - past_error)

            speed_offset = propotional_speed_offset + integral_speed_offset + differential_speed_offset


            # handle speeds
            bot.set_turn_speed(speed_offset)
            bot.set_target_speed(current_speed)
            bot.drive()


            # update past error
            past_error = error


            # change state depending on factors
            object_close = bot.sonic_sensor.value() < 100
            on_a_curve = abs(error_integral) > 0.1
            if object_close and not on_a_curve:
                Driving.obstacle = True
                Driving.change_state(Driving.States.reverse)

            if current_red_blue_level > 0.8:
                Driving.change_state(Driving.States.onred)
            if current_red_blue_level < -0.8:
                Driving.change_state(Driving.States.onblue)

            if bot.button.is_pressed:
                Driving.change_state(Driving.States.done)


            # accelerate if not at max speed of 180
            current_speed = min(max_speed, current_speed + 4)


            # sleep a little for greater motor rotation per tick, lowering relative error for odo
            time.sleep(0.025)
            Driving.bot.update_odo()


        bot.stop()


    # return whether an obstacle was encountered
    # exceute only once per call of drive
    @staticmethod
    def get_obstacle_found() -> bool:
        print("Obstacle WAS found" if Driving.obstacle else "Obstacle NOT found")
        temp = Driving.obstacle
        Driving.obstacle = False
        return temp

    obstacle: bool = False


    # when seeing an obstacle, turn around
    @staticmethod
    def reverse():
        ev3dev.core.Sound.beep()

        # selecting rotation speed
        Driving.bot.set_target_speed(0)
        Driving.bot.set_turn_speed(-200)

        Driving.bot.drive()


        start_time = time.perf_counter()
        while True:
            # get color, process it and get time
            color_data = Driving.bot.color_sensor.bin_data("hhhh")
            current_light_level = Color.to_light_level(color_data)
            current_time = time.perf_counter()

            # calulate conditions for stopping at line
            time_over: bool = current_time - start_time > 1.8 * 0.9  # needs to have turned about the right amount of time
            error_small: bool = abs(current_light_level - Color.target_level) < 0.25  # needs sufficently low error
            if time_over and error_small:
                break

            # update odo
            Driving.bot.update_odo()


        Driving.change_state(Driving.States.adjust)


    # when on a node, turn toward the specified absolute direction
    @staticmethod
    def turn(turn_towards):
        current_orientation = rad_to_deg(Driving.bot.correct_angle)
        turn = turn_towards.value - current_orientation
        if abs(turn) == 270:
            turn = 90 * -sign(turn)

        print("-------------")
        print(f"  {current_orientation = }")
        print(f"  {turn_towards.value = }")
        print(f"  {turn = }")
        print("-------------")

        # selecting time to rotate and rotation speed
        # at 200 speed 0.1s rotations = 10 degrees of rotation
        turn_time = abs(turn) / 100
        Driving.bot.set_target_speed(0)
        Driving.bot.set_turn_speed(-200 * sign(turn))
        Driving.bot.drive()

        past_light_level = 0
        start_time = time.perf_counter()
        while True:
            # get color, process it and get time
            color_data = Driving.bot.color_sensor.bin_data("hhhh")
            current_light_level = Color.to_light_level(color_data)
            current_time = time.perf_counter()

            # calulate conditions for stopping at line
            time_over: bool = current_time - start_time > turn_time * 0.9  # needs to have turned about the right amount of time
            right_edge: bool = (past_light_level - current_light_level) * sign(turn) > 0.01  # needs to be over right edge
            error_small: bool = abs(current_light_level - Color.target_level) < 0.25  # needs sufficently low error
            if (time_over and right_edge and error_small) or sign(turn) == 0:
                break

            past_light_level = current_light_level

            # update odo
            Driving.bot.update_odo()

        Driving.bot.stop()
        Driving.bot.correct_orientation(turn_towards)
        time.sleep(0.01)

        # unconditionally change to normal state
        Driving.change_state(Driving.States.adjust)


    # notifies that a Node was found and moves onto that Node
    @staticmethod
    def onpoint():
        Driving.bot.stop()

        ev3.Sound.beep()
        ev3.Sound.beep()
        time.sleep(1)

        # get into position
        Driving.bot.set_target_speed(200)
        Driving.bot.set_turn_speed(0)
        Driving.bot.drive()
        time.sleep(0.55)

        Driving.bot.stop()


    # rotated around its own axis to determine outgoing paths
    @staticmethod
    def search() -> list[Direction]:
        # circle around
        Driving.bot.set_target_speed(0)
        Driving.bot.set_turn_speed(-200)
        Driving.bot.drive()

        # init
        current_max = 100
        needs_to_be_added = False
        line_hits = []
        strait_line_hit = False
        past_light_level = 0

        start_time = time.perf_counter()
        while True:
            # get color, process it and get time
            color_data = Driving.bot.color_sensor.bin_data("hhhh")
            current_light_level = Color.to_light_level(color_data)
            current_time = time.perf_counter()

            # calulate conditions for stopping at line
            time_over = current_time - start_time > 3.50  # needs to have turned about the right amount of time
            right_edge: bool = past_light_level - current_light_level > 0.01  # be over right edge
            error_small: bool = abs(current_light_level - Color.target_level) < 0.25  # sufficently low error
            done = True if not strait_line_hit else error_small and right_edge # we want to align the bot with strait path is present
            if time_over and done:
                break

            # hit dark, hit a line.
            # remember the darkest point to have the most accurate hit time
            if current_light_level < Color.target_level * 0.8:
                if current_light_level < current_max:
                    current_max = current_light_level
                    needs_to_be_added = True

            # hit white, therefore no longer on the line
            # put a found line into the line_hits list
            if current_light_level > Color.target_level * 1.25:
                if needs_to_be_added:
                    line_hits.append((current_time - start_time))
                    needs_to_be_added = False
                current_max = current_light_level

            if len(line_hits) > 0 and line_hits[0] < 0.8:
                strait_line_hit = True

            past_light_level = current_light_level

            # update odo
            Driving.bot.update_odo()

        Driving.bot.stop()


        # normalize the time
        end_time = time.perf_counter()
        line_hits = [a / (end_time - start_time) * 360 for a in line_hits]

        print([round(a, 2) for a in line_hits])

        # turn hit time percentages into the corresponding directions
        outgoing_paths = []
        for hit_time_percentage in line_hits:
            closest_direction = 0
            for dirction in [0, 90, 180, 270]:
                if biased_distance(hit_time_percentage, closest_direction) > biased_distance(hit_time_percentage, dirction):
                    closest_direction = dirction
            outgoing_paths.append(closest_direction)


        print(f"relative directions: {outgoing_paths}")

        # change the calulated relative directions to abolute directions
        relative_directions = outgoing_paths
        absolute_bot_orientation = rad_to_deg(Driving.bot.correct_angle)
        absolute_directions = [Direction((dir + absolute_bot_orientation) % 360) for dir in relative_directions]

        print(f"absolut directions: {absolute_directions}")

        Driving.change_state(Driving.States.adjust)
        return absolute_directions


    # simple line following code at low speeds
    # mostly a copy-paste of normal
    @staticmethod
    def adjust():
        # avoid having to repeat the prefix over and over again
        bot = Driving.bot

        # init variables
        target_speed = 90
        past_errors = [0]

        bot.set_target_speed(target_speed / 2)

        start_time = time.perf_counter()
        while Driving.state is Driving.States.adjust:
            # get color and process it
            color_data = bot.color_sensor.bin_data("hhhh")
            current_light_level = Color.to_light_level(color_data)
            current_red_blue_level = Color.to_red_blue_level(color_data)

            # calulate error and error integral
            error = current_light_level - Color.target_level

            # calculate turn speeds of the P D
            propotional_speed_offset = 0.55 * -target_speed * error
            differential_speed_offset = 0.55 * -target_speed * (2 * error - past_errors[0])

            speed_offset = propotional_speed_offset + differential_speed_offset

            # set turn speed
            bot.set_turn_speed(speed_offset); bot.drive()


            # change state depending on factors,
            # most importantly adjust until relativly strait and return to normal
            time_over = time.perf_counter() - start_time > 0.1
            goes_strait = all(abs(error) < 0.1 for error in past_errors)
            if time_over and goes_strait:
                Driving.change_state(Driving.States.normal)

            if bot.sonic_sensor.value() < 100:
                Driving.obstacle = True
                Driving.change_state(Driving.States.reverse)


            # update past errors list
            past_errors.insert(0, error)
            if len(past_errors) > 8: past_errors.pop()


            # update odo
            Driving.bot.update_odo()


        # orientation is only now set to the theoretical orientation given by mothership
        # to ensure that the bot actual orientation lines up with the theoretical orientation
        bot.update_oriantation()


    @staticmethod
    def callibrate():
        print("Turbo Torben callibriert die Farben!")

        print("weiß next...")
        wait_until_pressed(Driving.bot.button)
        print("weiß start...")
        count = 0; accumulate = [0, 0, 0]
        while not Driving.bot.button.is_pressed:
            count += 1
            color = Driving.bot.color_sensor.bin_data("hhhh")
            accumulate = [accumulate[i] + color[i] for i in range(3)]
        light = [accumulate[i]/count for i in range(3)]
        wait_until_pressed(Driving.bot.button)
        print("weiß end.")


        print("schwarz next...")
        wait_until_pressed(Driving.bot.button)
        print("schwarz überfahren start...")
        Driving.bot.set_target_speed(100); Driving.bot.drive()
        dark = [0, 0, 0]; score = 999
        start_time = time.perf_counter()
        while time.perf_counter() - start_time < 4:
            color = Driving.bot.color_sensor.bin_data("hhhh")
            currect_score = color[0] + color[1] + color[2]
            if currect_score < score:
                score = currect_score
                dark = color
        dark = [dark[i]*1.1 for i in range(3)]
        Driving.bot.stop()
        print("schwarz end.")

        print("rot next...")
        wait_until_pressed(Driving.bot.button)
        print("rot start...")
        count = 0; accumulate = [0, 0, 0]
        while not Driving.bot.button.is_pressed:
            count += 1
            color = Driving.bot.color_sensor.bin_data("hhhh")
            accumulate = [accumulate[i] + color[i] for i in range(3)]
        red = [accumulate[i] / count for i in range(3)]
        wait_until_pressed(Driving.bot.button)
        print("rot end.")

        print("blau next...")
        wait_until_pressed(Driving.bot.button)
        print("blau start...")
        count = 0; accumulate = [0, 0, 0]
        while not Driving.bot.button.is_pressed:
            count += 1
            color = Driving.bot.color_sensor.bin_data("hhhh")
            accumulate = [accumulate[i] + color[i] for i in range(3)]
        blue = [accumulate[i] / count for i in range(3)]
        wait_until_pressed(Driving.bot.button)
        print("blau end.")


        print(light, dark, red, blue)
        Color.generate_coefficients([light, dark, red, blue])
        Color.save_data_to_file()


        Driving.change_state(Driving.States.prepare)



    @staticmethod
    def done():
        pass




#########################
# color callibration

class Color:
    # returns normilized light level
    @staticmethod
    def to_light_level(color) -> float:
        weighted_r = color[0] * Color.coefficients_light_level[0]
        weighted_g = color[1] * Color.coefficients_light_level[1]
        weighted_b = color[2] * Color.coefficients_light_level[2]

        weighted_sum = weighted_r + weighted_g + weighted_b

        light_dark_difference = Color.high_light_level - Color.low_light_level
        light_level_normalized = (weighted_sum - Color.low_light_level) / light_dark_difference
        return light_level_normalized


    # returns normilized red-blue level
    @staticmethod
    def to_red_blue_level(color) -> float:
        weighted_r = color[0] * Color.coefficients_red_blue_level[0]
        weighted_g = color[1] * Color.coefficients_red_blue_level[1]
        weighted_b = color[2] * Color.coefficients_red_blue_level[2]

        weighted_sum = weighted_r + weighted_g + weighted_b

        red_blue_difference = Color.high_red_blue_level - Color.low_red_blue_level
        red_blue_level_normalized = (weighted_sum - Color.low_red_blue_level) / red_blue_difference
        return 2*red_blue_level_normalized - 1


    coefficients_light_level = list()
    low_light_level = 0
    high_light_level = 100

    coefficients_red_blue_level = list()
    low_red_blue_level = 0
    high_red_blue_level = 100


    target_level: float = 0.5

    @staticmethod
    def load_data_from_file():
        with open("/home/robot/color_values.txt", "r") as file:
            parsed = [line.split(",") for line in file.readline().split("|")]  # parse the file
            parsed = [[float(number) for number in list] for list in parsed]  # convert the strings to floats

            print(parsed)

            Color.low_light_level = parsed[0][0]  # unpack the single element list into float
            Color.coefficients_light_level = parsed[1]
            Color.low_red_blue_level = parsed[2][0]  # unpack the single element list into float
            Color.coefficients_red_blue_level = parsed[3]


    @staticmethod
    def save_data_to_file():
        with open("/home/robot/color_values.txt", "w") as file:
            out = str()
            for list in [[Color.low_light_level], Color.coefficients_light_level, [Color.low_red_blue_level], Color.coefficients_red_blue_level]:
                for number in list:
                    out += str(number) + ","
                out = out[0:-1]  # remove te trailing ","
                out += "|"
            out = out[0:-1]  # remove the trailing "|"
            file.writelines(out)






    @staticmethod  # used to generate the coefficients for the light level  and red-blue level calculations
    def generate_coefficients(input_matrix):
        # this is done, so that the red abd blue area have a light level very similar to
        # the neural level, so that there are no correction from the driving script,
        # when the robot partially drives over the red or blue areas

        for row in input_matrix:
            print(row)

        # helper function scalar-vector product
        def prod(k, v):
            return [k * i for i in v]

        # helper function vector-vector sum
        def sum(v1, v2):
            return [a + b for a, b in zip(v1, v2)]

        # finds the best coefficients for generating light values by solving an LGS
        def find_light_coefficients():
            matrix = deepcopy(input_matrix)
            matrix[0] += [-0.0, 100] # weiß
            matrix[1] += [-1.0,  00] # schwarz
            matrix[2] += [-0.5,  50] # rot
            matrix[3] += [-0.5,  50] # blau

            solution = solve_system_of_equasions(matrix)

            Color.coefficients_light_level = solution[0:3]
            Color.low_light_level          = solution[3]

        # finds the best coefficients for generating red-blue values by solving an LGS
        def find_red_blue_coefficients():
            matrix = deepcopy(input_matrix)
            matrix[0] += [-0.5,  50] # weiß
            matrix[1] += [-0.5,  50] # schwarz
            matrix[2] += [-0.0, 100] # rot
            matrix[3] += [-1.0,  00] # blau

            for row in matrix:
                print(row)

            solution = solve_system_of_equasions(matrix)

            print(solution)

            Color.coefficients_red_blue_level = solution[0:3]
            Color.low_red_blue_level          = solution[3]


        # solves the LGS Ax = b to find the coefficients to produce the calculated target
        # b needs to be appended to the right of A thus the name of A_with_b
        def solve_system_of_equasions(A_with_b):
            A = A_with_b

            A[0] = prod(1 / A[0][0], A[0])  # make diagonal entry of row 1 have a 1
            A[1] = sum(A[1], prod(-A[1][0], A[0]))  # make the off-diagonal entries equal to 0
            A[2] = sum(A[2], prod(-A[2][0], A[0]))  # make the off-diagonal entries equal to 0
            A[3] = sum(A[3], prod(-A[3][0], A[0]))  # make the off-diagonal entries equal to 0

            A[1] = prod(1 / A[1][1], A[1])  # make diagonal entry of row 2 have a 1
            A[0] = sum(A[0], prod(-A[0][1], A[1]))  # make the off-diagonal entries equal to 0
            A[2] = sum(A[2], prod(-A[2][1], A[1]))  # make the off-diagonal entries equal to 0
            A[3] = sum(A[3], prod(-A[3][1], A[1]))  # make the off-diagonal entries equal to 0

            A[2] = prod(1 / A[2][2], A[2])  # make diagonal entry of row 3 have a 1
            A[0] = sum(A[0], prod(-A[0][2], A[2]))  # make the off-diagonal entries equal to 0
            A[1] = sum(A[1], prod(-A[1][2], A[2]))  # make the off-diagonal entries equal to 0
            A[3] = sum(A[3], prod(-A[3][2], A[2]))  # make the off-diagonal entries equal to 0

            A[3] = prod(1 / A[3][3], A[3])  # make diagonal entry of row 4 have a 1
            A[0] = sum(A[0], prod(-A[0][3], A[3]))  # make the off-diagonal entries equal to 0
            A[1] = sum(A[1], prod(-A[1][3], A[3]))  # make the off-diagonal entries equal to 0
            A[2] = sum(A[2], prod(-A[2][3], A[3]))  # make the off-diagonal entries equal to 0

            return [A[0][4], A[1][4], A[2][4],  A[3][4]]





        # actual code of the function begine here
        find_light_coefficients()
        find_red_blue_coefficients()

        print(f"{Color.coefficients_light_level = }")
        print(f"{Color.low_light_level = }")

        print(f"{Color.coefficients_red_blue_level = }")
        print(f"{Color.low_red_blue_level = }")

        print(f"light value of light: {Color.to_light_level(input_matrix[0])}")
        print(f"light value of dark: {Color.to_light_level(input_matrix[1])}")
        print(f"light value of red: {Color.to_light_level(input_matrix[2])}")
        print(f"light value of blue: {Color.to_light_level(input_matrix[3])}")

        print(f"redblue value of light: {Color.to_red_blue_level(input_matrix[0])}")
        print(f"redblue value of dark: {Color.to_red_blue_level(input_matrix[1])}")
        print(f"redblue value of red: {Color.to_red_blue_level(input_matrix[2])}")
        print(f"redblue value of blue: {Color.to_red_blue_level(input_matrix[3])}")




#########################
# utils

def try_connect_temp_buton():
    try:
        button = ev3.TouchSensor("in1")
        if button.is_pressed or not button.is_pressed:
            print("temp button IS connected")
            return button
    except Exception:
        print("temp button NOT connected")
        return None


def wait_until_pressed(button, button2 = None):
    if button2 is not None:
        try:
            while not (button.is_pressed or button2.is_pressed):
                pass

            out = button if button.is_pressed else button2

            while button.is_pressed or button2.is_pressed:
                pass

            return out

        except Exception:
            button2 = None

    if button2 is None:
        while not button.is_pressed:
            pass

        while button.is_pressed:
            pass

        return button



def sign(x):
    return 0 if x == 0 else (1 if x > 0 else -1)



def rad_to_deg(x: float) -> int:
    percent = x / (2 * math.pi)
    properly_offset_percent = -percent + 1/4
    degrees = round(properly_offset_percent * 360)
    return degrees % 360

def deg_to_rad(x: int) -> float:
    percent = x / 360
    properly_offset_percent = -percent + 1/4
    randiants = properly_offset_percent * 2 * math.pi
    return randiants % (2 * math.pi)



# distance function, with bias
def biased_distance(messured, target):
    distance = abs(messured - target)
    if messured > target:
        return distance
    else:
        return distance * 2



class Rounding:
    even_point_color = 0
    odd_point_color = 0

    @staticmethod
    def initial_position_is(position: tuple):
        current_color = 1 if Driving.state == Driving.States.onred else -1
        oposite_color = 1 if Driving.state == Driving.States.onblue else -1

        modulus = (position[0] - position[1]) % 2
        if modulus == 0:
            Rounding.even_point_color = current_color
            Rounding.odd_point_color = oposite_color
        if modulus == 1:
            Rounding.odd_point_color = current_color
            Rounding.even_point_color = oposite_color

    # gives the color of the points at given position
    @staticmethod
    def color_of_point(x, y):
        modulus = (x - y) % 2
        if modulus == 0:
            return Rounding.even_point_color
        if modulus == 1:
            return Rounding.odd_point_color

    #rounds coordinated to nearest map point of the color the bot is positioned on
    @staticmethod
    def round_to_nearest_map_coordinate(x, y):
        assert Driving.state == Driving.state.onred or  Driving.state == Driving.state.onblue
        current_color = 1 if Driving.state == Driving.state.onred else -1

        map_x = x / 50
        map_y = y / 50

        rounded_x = round(map_x)
        rounded_y = round(map_y)

        if Rounding.color_of_point(rounded_x, rounded_y) == current_color:
            return rounded_x, rounded_y

        if Rounding.color_of_point(rounded_x, rounded_y) != current_color:
            closest_x = 0; closest_y = 0; closest_dist = 999
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                checked_x = rounded_x + dx
                checked_y = rounded_y + dy
                dist = ((checked_x - map_x)**2 + (checked_y - map_y)**2)**0.5

                if dist < closest_dist:
                    closest_dist = dist
                    closest_x = checked_x
                    closest_y = checked_y

            return (closest_x, closest_y)





