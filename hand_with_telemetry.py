# Blubot hand driver with telemetry to Cyberdeck
# Author: Luis Villazon
# License: Public Domain
from __future__ import division
import time  # for delays
from collections import deque  # to queue intermediate finger positions

import Adafruit_PCA9685  # support for the servo hat
import pygame  # used for keyboard input

import paho.mqtt.client as mqtt  # MQTT used to send/receive data via ethernet
BROKER = "192.168.187.205"  # 187.205 is BB.CD in hex (BlueBot CyberDeck)
TELEMETRY_TOPIC= "hand/telemetry"
COMMAND_TOPIC = "hand/commands"
PORT = 1883  # standard port, use 8883 for encrypted TLS connections
KEEP_ALIVE = 60  # seconds of inactivity before the connection is dropped

STATUS_READY = "status:READY"
STATUS_BUSY = "status:BUSY"

INPUT_NORMAL = 0
INPUT_BINARY = 1
INPUT_ARITHMETIC = 2

# Uncomment to enable debug output of register writes for the PCA9685
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 servo bonnet using the default address (0x40).
# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)
pwm = Adafruit_PCA9685.PCA9685(0x40)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

def cubic_ease_in_out(t):
    if t < 0.5:
        return 4 * t * t * t
    else:
        f = (2*t - 2)
        return 0.5 * f * f * f + 1

class Joint:
    # used with JX PDI6221MG servos that have 180 turning angle
    pulse_start = 0 # this is always 0 since we want the servo to move immediately
    servo_min = 114  # Min pulse length out of 4096
    servo_max = 644  # Max pulse length out of 4096
    delay_time = 0.005  # Delay between steps. Adjust based on total desired duration.
    
    def __init__(self, ch, name, open_pos=0, closed_pos=180):
        self.servo_channel = ch
        self.name = name
        self.min_pos = open_pos
        self.max_pos = closed_pos
        self.position = 0 # current servo angle from 0 - 180
        self.intermediate_positions = deque()
        #self.set_servo_angle(90) # set conservative initial angle at first
        
    # DEPRECATED
    # move servo directly to a given angle, without any easing
    def goto_servo_angle(self, angle):
        if angle < self.min_pos:
            angle = self.min_pos
        if angle > self.max_pos:
            angle = self.max_pos
        pulse_width = angle * (Joint.servo_max - Joint.servo_min)//180 + Joint.servo_min

        pwm.set_pwm(self.servo_channel, Joint.pulse_start, pulse_width)
        self.position = angle # update the current position of the servo
        print("setting", self.name, "to", self.position)

    # set target angle for the servo
    # this precalculates the intermediate positions using cubic easing
    # the update function moves through these positions until the target pos is reached
    # any positions left in the queue are not deleted, so moves can be chained together
    def set_servo_angle(self, target_angle):
        if target_angle < self.min_pos:
            target_angle = self.min_pos
        if target_angle > self.max_pos:
            target_angle = self.max_pos
            
        start_angle = self.position
        num_steps = 50  # The number of interpolation steps. Adjust as needed.

        for step in range(num_steps):
            t = step / (num_steps - 1)  # Normalize step to [0, 1]
            eased_step = cubic_ease_in_out(t)
            self.intermediate_positions.append((1 - eased_step) * start_angle + eased_step * target_angle)
           
    # advance the servo to the next position in the interpolated sequence
    # returns True when there are no more positions in the queue - ie the move is complete
    def update(self):
        if self.intermediate_positions:  # ie if the queue is not empty
            angle = self.intermediate_positions.popleft()  # get next pos in the queue

            # Compute the pulse width for this angle and command the servo
            pulse_width = angle * (Joint.servo_max - Joint.servo_min) // 180 + Joint.servo_min
            pwm.set_pwm(self.servo_channel, Joint.pulse_start, int(pulse_width))

            # Update the current position to the current angle 
            # and move the index to the next one
            self.position = angle
            time.sleep(Joint.delay_time)
            return False
        return True  # movement sequence is complete       
            
    # position records the actual servo angle. For a given finger, the degree of
    # joint flexing at a certain angle will vary, because of differences in string length
    # This is compensated for by the hard-coded max and min posititions for each joint
    # But to provide consistent telemetry, this function returns a normalised percentage
    # from 0 (min_pos) to 100 (max_pos) for each joint. It is returned as an integer
    # to make parsing simpler.
    def get_flexion(self):
        return int((self.position-self.min_pos)/(self.max_pos-self.min_pos)*100)
        
class Hand:
    def __init__(self):
        self.wrist = Joint(5, 'wrist')
        self.thumb = Joint(4, 't', 10, 150)
        self.index = Joint(3, 'i', 20, 130)
        self.middle = Joint(2, 'm', 0, 140)
        self.ring = Joint(1, 'r', 10, 130)
        self.pinkie = Joint(0, 'p', 10, 120)
        self.fingers = [self.thumb, self.index, self.middle, self.ring, self.pinkie]
        self.fingers_not_thumb = [self.index, self.middle, self.ring, self.pinkie]

    # give each finger a chance to move towards its target position
    def update(self):
        ready = True
        for f in self.fingers:
            if not f.update():
                ready = False
        # send telemetry is any fingers moved
        if not ready:
            self.show_positions()
        return ready
            
    def make_fist(self):
        self.clear_thumb()
        for f in self.fingers_not_thumb:
            self.close_finger(f)
        self.close_thumb()

    def make_palm(self):
        for f in self.fingers:
            self.open_finger(f)

    def make_call_me(self):
        self.open_finger(self.thumb)
        self.close_finger(self.index)
        self.close_finger(self.middle)
        self.close_finger(self.ring)
        self.open_finger(self.pinkie)

    def make_metal(self):
        self.clear_thumb()
        self.open_finger(self.index)
        self.close_finger(self.middle)
        self.close_finger(self.ring)
        self.open_finger(self.pinkie)
        self.close_thumb()

    def clear_thumb(self):
        # move the thumb just far enough out to avoid hitting the index
        self.thumb.set_servo_angle(70)
        while not self.thumb.update():
            pass  # give the servo time to respond

    def close_thumb(self):
        while not self.update():
            pass # wait for the other fingers to finish moving
        # close the thumb just enough to wrap around the index
        self.thumb.set_servo_angle(120)

    def open_finger(self, f):
        f.set_servo_angle(f.min_pos)

    def close_finger(self, f):
        f.set_servo_angle(f.max_pos)
        
    def make_number(self, n):
        self.clear_thumb()
        for f in self.fingers_not_thumb:
            if self.fingers.index(f) <= n:
                self.open_finger(f)
            else:
                self.close_finger(f)
        if n >= 5:
            self.open_finger(self.thumb)
        else:
            self.close_thumb()

    def make_casey_neistat_wave(self):
        # creepy single-handed wave goodbye
        self.make_palm()
        for i in range(2):
            time.sleep(0.5)
            for f in self.fingers_not_thumb:
                self.close_finger(f)
            time.sleep(0.5)
            for f in self.fingers_not_thumb:
                self.open_finger(f)

    def make_ripple_wave(self):
        # wiggle the fingers, as if to say 'cooeee!'
        self.make_palm()
        self.fingers_not_thumb.reverse() # swap list order so the wiggle starts from the pinkie 
        for i in range(2):
            for f in self.fingers_not_thumb:
                f.set_servo_angle(50)
                time.sleep(0.1)
            for f in self.fingers_not_thumb:
                self.open_finger(f)
                time.sleep(0.1)
        self.fingers_not_thumb.reverse() # swap it back
            
    def show_positions(self):
        data = []
        for f in self.fingers:
            data.append(f.name 
                        + ':' + str(f.position)
                        + '/' + str(f.get_flexion()))
        data.append(self.wrist.name + ':' + str(self.wrist.get_flexion()))
        data_string = ",".join(data)

        # send data to Cyberdeck
        send_data(TELEMETRY_TOPIC, "LEFT_HAND," + data_string)

    def display_binary(self, n):
        if n >= 0 and n <= 31: # can only display a 5-bit number on 5 fingers!
             self.make_palm()
             while not self.update():
                 pass  # wait for all fingers to be up
             
             self.fingers.reverse() # reverse finger order so the least significant bit is the pinkie
             for i in range(4,-1,-1): # from 4 to 0, inclusive 
                if n & 2**i:
                    self.open_finger(self.fingers[i])
                    print('1', end='')
                else:
                    self.close_finger(self.fingers[i])
                    print('0', end='')
             self.fingers.reverse() # swap back
        
def input_digit():
    # wait for one of the number keys to be pressed and return that
    # any other key returns -1
    number_keys = (pygame.K_0,
                   pygame.K_1, 
                   pygame.K_2, 
                   pygame.K_3, 
                   pygame.K_4, 
                   pygame.K_5, 
                   pygame.K_6, 
                   pygame.K_7, 
                   pygame.K_8, 
                   pygame.K_9)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key in number_keys:
                    return number_keys.index(event.key)
                else:
                    return -1

def input_operator():
    # wait for one of the arithmatic operator keys to be pressed and return that
    # any other key returns -1
    operator = ''
    while True:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_PLUS, pygame.K_EQUALS):
                    operator = '+'
                if event.key in (pygame.K_MINUS, pygame.K_UNDERSCORE):
                    operator = '-'
                if event.key in (pygame.K_SLASH, pygame.K_QUESTION):
                    operator = '/'
                if event.key in (pygame.K_ASTERISK, pygame.K_8):
                    operator = '*'
                if operator != '':
                    return operator
                else:
                    return -1

def input_number():
    # input a 1 or 2 digit number
    # press enter to return the number
    digit1 = input_digit()
    if digit1 != -1:
        digit2 = input_digit()
        if digit2 == -1: # single-digit number (because 2nd input was not a number)
            return digit1
        else:
            return digit1*10 + digit2

def asasd():
    # get a 1 or 2 digit number
    # then an arithmatic operator
    # then another number
    # return the result if between 1 and 5 or 0 otherwise
    # if the operator isnt valid, skip the 2nd number and also return 0
    num1 = input_number()
    operator = input_operator()
    if operator != -1:
        num2 = input_number()
        result = eval(str(num1) + operator + str(num2))
        if result >=1 and result <=5:
            return result
    return 0    

# message handler for MQTT
def on_message(client, userdata, message):
    global keypress  # used to store the key received from cyberdeck
    text = message.payload.decode('UTF-8')
    print(text)
    if message.topic == COMMAND_TOPIC:
        # parse the data to decide what to do with it
        # command data  is of the form:
        #   COMMAND:FIST

        command_word = text.split(":")[1]  # take the part after the colon
        if command_word == "PALM":
            left_hand.make_palm()
        elif command_word == "FIST":
            left_hand.make_fist()
        elif command_word == "CALL_ME":
            left_hand.make_call_me()
        elif command_word == "METAL":
            left_hand.make_metal()
        elif len(command_word) == 1:  # single character commands are assumed to be key presses
            keypress = command_word
            print("setting keypress to", command_word)
        else:
            print("unrecognised command:", text, "on topic", message.topic)

# connection callback function - DEPRECATED?
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    #client.subscribe(TOPIC)

# send data back to the cyberdeck via MQTT
def send_data(channel, data=None):
    print(channel, "TX", data)
    if data:
        cyberdeck.publish(channel, data)
 

###################################################

# MAIN PROGRAM

# set up the environment
left_hand = Hand()
pygame.init()
screen = pygame.display.set_mode((512,384))
clock = pygame.time.Clock()
finished = quit_flag = False

# initialise MQTT for telemetry feed and commands
cyberdeck = mqtt.Client()
cyberdeck.on_message = on_message
cyberdeck.connect(BROKER, PORT, KEEP_ALIVE)
cyberdeck.subscribe(TELEMETRY_TOPIC)
cyberdeck.subscribe(COMMAND_TOPIC)

print('Blubot starting up...')
status = True  # True means ready
# close and open as an obvious initialisation
left_hand.make_fist()
left_hand.make_palm()

# initialise command buffer & mode flags
keypress = ""
input_mode = INPUT_NORMAL
arithmetic_mode = False

# map of keybinds to gestures
gesture_map = {'z':left_hand.make_palm,
                'x':left_hand.make_fist,
                'c':left_hand.make_call_me,
                'v':left_hand.make_metal,
                'n':left_hand.make_casey_neistat_wave,
                'm':left_hand.make_ripple_wave}

status_timeout = 0
# main loop
while not finished:
    # check for local keyboard events
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            # quit option
            if quit_flag and event.key == pygame.K_y:
                finished = True
            if event.key == pygame.K_ESCAPE:
                print('Exit? (y/n)')
                quit_flag = True
            else:
                quit_flag = False
                
            # pass all printable ascii chars to the keypress command handler
            if event.key in range(13, 127):  # ASCII characters inc carriage return
                keypress = chr(event.key).lower()

    # keypress command handler
    # processes any keypresses, whether local or remote
    if keypress:
        print("pressed", keypress)
        # individual finger/wrist control keys
        if keypress in ('qa'):
            f = left_hand.pinkie
        if keypress in ('ws'):
            f = left_hand.ring
        if keypress in ('ed'):
            f = left_hand.middle
        if keypress in ('rf'):
            f = left_hand.index
        if keypress in ('tg'):
            f = left_hand.thumb
        
        if keypress in 'qwert':
            f.set_servo_angle(f.position-10)
        if keypress in 'asdfg':
            f.set_servo_angle(f.position+10)

        # numbers 1-5
        number_keys = '12345'
        if keypress in number_keys and input_mode == INPUT_NORMAL:
            left_hand.make_number(number_keys.index(keypress)+1) # zero-indexed
        
        # gestures
        if keypress in gesture_map:
            gesture_map[keypress]()

        # binary translation mode
        # hit B to enter this mode, input a number 0-31
        # when the 2nd digit is input (or Enter for single-digit numbers)
        # translate this number to binary digits on the fingers
        if keypress == 'b':
            input_mode = INPUT_BINARY
            input_string = ''
        if input_mode == INPUT_BINARY:
            if keypress in '0123456789':
                input_string += keypress
            # enter 2 digits or 1 digit + <ENTER>
            if keypress == chr(13) or len(input_number_string) == 2:
                left_hand.display_binary(int(input_string))
                input_mode = INPUT_NORMAL

        # arithmatic mode
        # hit the = key to enter this mode
        # enter any sequence of digits and +-
        # then enter to evaluate
        # shows that number as binary if it is 0 - 31. Or a fist, otherwise
        if keypress == '=' and input_mode == INPUT_NORMAL:
            print("arithmetic mode")
            input_mode = INPUT_ARITHMETIC
            input_string = ''
        if input_mode == INPUT_ARITHMETIC:
            if keypress in '0123456789-':
                input_string += keypress
            elif keypress == '=':  # SHIFT key is ignored, so + appears as =
                input_string += '+'
            elif keypress == chr(13):
                print("evaluating", input_string, "as", eval(input_string))
                left_hand.display_binary(eval(input_string))
                input_string = ''
                input_mode = INPUT_NORMAL
   
    # clear keypress after it has been handled
    keypress = ''

    #clock.tick(1) # not needed now, since we are contantly checking the MQTT messages
    
    cyberdeck.loop(timeout=0.01)  # process MQTT betwork traffice, dispatch callbacks
    old_status = status
    status = left_hand.update()
    # if status has changed (because we have begun or completed a movement)
    # send this as a telemetry message
    # TODO also send this periodically - just incase the message gets lost en route
    if status != old_status or time.monotonic() > status_timeout:
        status_timeout = time.monotonic() + 3.0  # 3 seconds
        if status:
            send_data(TELEMETRY_TOPIC, "LEFT_HAND," + STATUS_READY)
        else:
            send_data(TELEMETRY_TOPIC, "LEFT_HAND," + STATUS_BUSY)

print('Blubot shutting down.')

