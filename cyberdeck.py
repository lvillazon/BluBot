# Cyberdeck controller suitcase for Blubot
# Author: Luis Villazon
# License: Public Domain

import pi3d, time, math
import pyautogui  # used for touchscreen coordinates
import paho.mqtt.client as mqtt

# Graphics constants
BACKGROUND_COLOR = (0,0,0,1)
ROBOT_COLOR = (26, 189, 235)  # blubot blue
OUTLINE_COLOR = (0, 0, 0)
BUTTON_COLOR = (100, 100, 255)
BUTTON_TEXT_COLOR = (255, 255, 255)
GUI_BG = ROBOT_COLOR  # base colour for gui widgets
GUI_FG = (255,255,255)  # highlight and text colour
GUI_DISABLED = (128,128,128)  # grey colour for disabled buttons
GUI_DEPTH = 0.001  # arbitrarily small value to make 3D GUI elements appear essentially 2D

CURSOR_HIDE_DELAY = 2.0  # seconds before the cursor disappears, if stationary

# MQTT constants
MQTT_ENABLED = True  # if False, no communication attempts are made (used for testing only)
BROKER = "192.168.187.205"  # 187.205 is BB.CD in hex (BlueBot CyberDeck)
TELEMETRY_TOPIC= "hand/telemetry"
COMMAND_TOPIC = "hand/commands"
PORT = 1883  # standard port, use 8883 for encrypted TLS connections
KEEP_ALIVE = 60  # seconds of inactivity before the connection is dropped
RECONNECT_RETRY = 5  # seconds before we attempt to reconnect a missing robot

class Box3D:
    def __init__(self, x, y, z, w, h, d, shader, color=ROBOT_COLOR, outline=True):
        # block is the solid cuboid
        # outline is the wireframe vertices
        #self.block = pi3d.Cuboid(x=x, y=y, z=z, w=w, h=h, d=d, cx=-w/2, cy=h/2, cz=0)
        self.block = pi3d.Cuboid(x=x, y=y, z=z, 
                                 w=w, h=h, d=d, 
                                 cx=0, cy=h/2, cz=0,
                                 camera = CAMERA)
        self.block.set_material(color)
        self.block.set_shader(shader)

        if outline:
            self.outline = pi3d.Cuboid(x=0,y=0,z=0, w=w, h=h, d=d, camera=CAMERA)
            self.outline.set_material(OUTLINE_COLOR)
            self.outline.set_shader(shader)
            self.outline.set_line_width(2)
            
            self.block.add_child(self.outline)
        
    def add_child(self, child):
        self.block.add_child(child.block)

    def draw(self):
        self.block.draw()

    def rotate(self, axis, angle):
        if axis == 'x':
            self.block.rotateIncX(angle)
        elif axis == 'y':
            self.block.rotateIncY(angle)
        elif axis == 'z':
            self.block.rotateIncZ(angle)

    def x(self):
        return self.block.x()
    
    def y(self):
        return self.block.y()
    
    def z(self):
        return self.block.z()
    
class Finger:
    def __init__(self, x, y, z, w, l, shader):
        self.WIDTH = w
        self.LENGTH = l
        self.joint1_length = self.LENGTH * 0.5
        self.joint2_length = self.LENGTH * 0.25
        self.joint3_length = self.LENGTH * 0.25

        joint1_y_centre = y + self.joint1_length /2
        joint2_y_centre = self.joint1_length/2 + self.joint2_length/2
        joint3_y_centre = self.joint2_length/2 + self.joint3_length/2
        self.joints = [
            Box3D(x, joint1_y_centre, z, self.WIDTH, self.joint1_length, self.WIDTH, shader),
            Box3D(0, joint2_y_centre, 0, self.WIDTH, self.joint2_length, self.WIDTH, shader),
            Box3D(0, joint3_y_centre, 0, self.WIDTH, self.joint3_length, self.WIDTH, shader)
        ]

        self.cumulative_angles = [0, 0, 0]
        
        # set hierarchical relationship between the finger joints
        self.joints[0].add_child(self.joints[1])
        self.joints[1].add_child(self.joints[2])
        
    # return the cuboid that forms the base of the finger
    def get_base_joint(self):
        return self.joints[0]

    def draw(self):
        self.joints[0].draw()

    def flex(self, flexion):
        angle = int(-90*flexion/100)  # convert flex% to joint angle
        self.joints[0].block.rotateToX(angle)
        self.joints[1].block.rotateToX(angle)
        self.joints[2].block.rotateToX(angle)

    def old_flex(self, angle):
        # Rotate the base joint
        self.joints[0].block.rotateToX(self.cumulative_angles[0] + angle)
        self.cumulative_angles[0] += angle

        # Rotate the second joint
        self.joints[1].block.rotateToX(self.cumulative_angles[1] + angle)
        self.cumulative_angles[1] += angle

        # Rotate the third joint
        self.joints[2].block.rotateToX(self.cumulative_angles[2] + angle)
        self.cumulative_angles[2] += angle
            
class Hand:
    def __init__(self, x, y, z, size, shader):
        self.palm = Box3D(x, y, z, size, size, size/5, shader, ROBOT_COLOR)
        
#        self.cumulative_angles = [0, 0, 0]
        
        # create fingers
        self.fingers = []
        
        # create thumb as a special case
        thumb = Finger(-size/2, size/2, 0, size/4, 0.8, shader)
        thumb.get_base_joint().block.rotateToZ(45)
        self.fingers.append(thumb)

        # now the regular 4
        width = size/5
        base_x = -width*1.5  # offset by 2 fingers to the left and then half a finger back to the right
        sizes = [1.1, 1.2, 1.1, 1.0]
        for s in sizes:
            self.fingers.append(Finger(base_x, size/2, 0, width, s, shader))
            base_x += width
                   
        # set hierarchical relationship between the hand and fingers
        for f in self.fingers:
            self.palm.add_child(f.get_base_joint())
    
    def get_finger(self, i):
        return self.fingers[i]

    def draw(self):
        self.palm.draw()

class GUI:
    def __init__(self, shader):
        self.shader = SHADER
        # set the z_plane to an arbitrary point that shoint be in front of all 3D objects
        # but leave room for 2D elements stacked on top of each other        
        self.z = 20.0
        self.line_width = 5
        self.width = DISPLAY.width - self.line_width
        self.height = DISPLAY.height - self.line_width

        # create a transparent box to hold all the GUI widgets
        self.pane = pi3d.Cuboid(x=0, y=0, z=self.z, 
                                w=self.width, h=self.height, d=0.1, 
                                cx=-self.width/2, cy=self.height/2, cz=0,
                                camera = GUI_CAMERA)
        self.pane.set_material((255,0,0,0))  # red for debugging, but normally totally transparent
        self.pane.set_shader(shader)

        # outline for the GUI area
        self.frame = pi3d.Cuboid(x=self.pane.x(),y=self.pane.y(), z=self.pane.z(),
                                 w=self.pane.width, 
                                 h=self.pane.height, d=self.pane.depth,
                                 camera=GUI_CAMERA)
        self.frame.set_material(ROBOT_COLOR)
        self.frame.set_shader(shader)
        self.frame.set_line_width(self.line_width)

        # list of all widgets associated with this gui that must be manually redrawn
        # widgets that are just nested cuboids will redraw whenever the parent is drawn
        # but anything with a label needs a manual redraw
        self.widgets=[]

        # initialise the mouse
        self.cursor = MouseCursor(shader=SHADER)
        self.add(self.cursor)

    def calculate_height_for_plane(self, z_distance, fov_v=45.0):
        # Convert fov_v from degrees to radians
        fov_v_rad = math.radians(fov_v)

        # Using the formula for apparent height
        return 2 * z_distance * math.tan(fov_v_rad / 2)

    def draw(self):
        self.pane.draw()
        self.frame.draw()
        for w in self.widgets:
            if w and w.manual_redraw:  # some widgets need to call their own draw method to handle special stuff
                w.draw()
        self.cursor.draw()
#        self.string_caption.draw()

    def add(self, object):
        self.widgets.append(object)
        # widgets also get the base Shape added as a child for automatic redrawing
        if isinstance(object, Widget):
            self.pane.add_child(object.widget)

    def check_events(self):
        # Convert the mouse's screen coordinates to normalized display coordinates.
        x, y = self.cursor.get_normalised_pos()
        
        if self.cursor.left_button():
            if not self.cursor.is_dragging:
                # TODO replace with a more generic check for any widget
                # check slider as a special case
                if rotation_slider.colliding(x, y):
                    self.cursor.is_dragging = True
                else:
                    # check all buttons
                    buttons = [w for w in self.widgets if isinstance(w, Button)]
                    for b in buttons:
                        if b.enabled and b.colliding(x, y):
                            print(b.name, "clicked")
                            b.click() 
            else:
                print(x-rotation_slider.start_x-rotation_slider.width/2)
                pos_ratio = rotation_slider.move_handle(x-rotation_slider.start_x-rotation_slider.width/2)
                if pos_ratio is not None:
                    angle = pos_ratio * 180 -90
                    hand.palm.block.rotateToY(angle)
        else:
            self.cursor.is_dragging = False

    # disable all buttons to prevent repeat messages
    def lock_buttons(self):
        buttons = [w for w in self.widgets if isinstance(w, Button)]
        for b in buttons:
            b.enabled = False
            b.widget.set_material(GUI_DISABLED)

    def unlock_buttons(self):
        buttons = [w for w in self.widgets if isinstance(w, Button)]
        for b in buttons:
            b.enabled = True     
            b.widget.set_material(GUI_BG)

# base class for all 2D UI elements
class Widget:
    def __init__(self, parent, x, y, width, height):
        self.parent = parent
        self.shader = parent.shader
        self.width = width
        self.height = height
        self.z = parent.z - 0.1  # put each object slightly in front of its parent
        self.panel = pi3d.Cuboid(x=x-.15, y=y, z=self.z, 
                                 w=width, h=height, d=0.1, 
                                 cx=0, cy=height/2, cz=0,
                                 camera=GUI_CAMERA,
                                 )
        self.panel.set_material(GUI_BG)
        self.panel.set_shader(parent.shader)

# TODO outline box
#        self.outline = pi3d.Cuboid(x=0,y=0,z=0, w=w, h=h, d=d)
#        self.outline.set_material(OUTLINE_COLOR)
#        self.outline.set_shader(shader)
#        self.outline.set_line_width(2)      
#        self.block.add_child(self.outline)
        self.widget = self.panel
        self.manual_redraw = False  # override to True for anything with a label
        self.enabled = True  # if False, will not respond to mouse input

    def draw(self):
        if self.widget:
            self.widget.draw()

    # return true if x and y lie within the area of this widget
    def colliding(self, x,y):
        if (self.widget.x()-self.width/2 <= x <= self.widget.x()+self.width/2 and
            self.widget.y()-self.height/2 <= y <= self.widget.y()+self.height/2):
            return True
        else:
            return False

class Old_3D_Widget:  # deprecated class from when the GUI was rendered with the 3D camera
    def __init__(self, shader, parent, x, y, width, height):
        self.width = width * parent.scaling_factor
        self.height = height * parent.scaling_factor
        self.base_rect = Box3D(x*parent.scaling_factor, y*parent.scaling_factor, -GUI_DEPTH, 
                                self.width, self.height, GUI_DEPTH, shader, color=GUI_BG)
        self.widget = self.base_rect.block
        self.manual_redraw = False  # override to True for anything with a label
        self.enabled = True  # if False, will not respond to mouse input

    def draw(self):
        if self.widget:
            self.widget.draw()

    # return true if x and y lie within the area of this widget
    def colliding(self, x,y):
        if (self.base_rect.x()-self.width/2 <= x <= self.base_rect.x()+self.width/2 and
            self.base_rect.y()-self.height/2 <= y <= self.base_rect.y()+self.height/2):
            return True
        else:
            return False


class Slider(Widget):
    def __init__(self, parent, x, y, width, height=50):
        super().__init__(parent, x, y, width , height)
        self.start_x = x - self.width/2
        self.end_x = x + self.width/2
        self.current_position = x
        self.track = self.widget
        self.track.set_material(GUI_BG)
        self.track.set_shader(self.shader)
        self.handle = pi3d.Cuboid(x=0, y=0, z=-0.1,             # coords are relative to the track
                                 w=height-5, h=height-5, d=0.1, 
                                 cx=0, cy=height/2, cz=0,
                                 camera=GUI_CAMERA,
                                 )
        self.handle.set_material(GUI_FG)
        self.handle.set_shader(self.shader)
        self.track.add_child(self.handle)

    def move_handle(self, x_pos):
        if -self.width/2 <= x_pos <= self.width/2:
            delta = x_pos - self.current_position
            self.handle.translateX(delta)
            self.current_position = x_pos
            print(x_pos, self.handle.x())
            return (x_pos - self.start_x) / self.width
        return None
    
class Label():
    # although these are technically widgets in the GUI sense,
    # they don't inherit from Widget, since they have no underlying cuboid
    # and render in a different way
    def __init__(self, parent, x, y, size=72, 
                 text="", font_path="/usr/share/fonts/truetype/piboto/PibotoCondensed-Bold.ttf"):
        
        z = parent.z - 0.1  # slightly in front
        self.font = pi3d.Font(font_path)
        self.font.blend = True
        self.string_caption = pi3d.String(camera=GUI_CAMERA, 
                                          font=self.font,
                                          is_3d=False,
                                          string=text,
                                          #justify="L"  # - seems to be ignored, always centers
                                          )
#        self.string_caption.sprite.position(x, y, z)
        self.string_caption.position(x, y, 1.0)
        self.string_caption.set_shader(pi3d.Shader('uv_flat'))  # the mat_flat shader doesn't work for text
        self.draw()  # have to call this once before we can use the quick_change() method
        self.manual_redraw = True

    # TODO - update for 2D GUI
    def set_text(self, new_text):
        self.string_caption.quick_change(new_text)
        
    def draw(self):
        self.string_caption.draw()

# put aside while I test the quick change method
class OldLabel():
    # although these are technically widgets in the GUI sense,
    # they don't inherit from Widget, since they have no underlying cuboid
    # and render in a different way
    def __init__(self, parent, x, y, size=72, 
                 text="", font="/usr/share/fonts/truetype/piboto/PibotoCondensed-Bold.ttf"):
        
        z = parent.z - 0.1  # slightly in front
        self.font = font
        self.string_caption = pi3d.FixedString(camera=GUI_CAMERA, 
                                               font=font,
                                               string=text,
                                               color="#FFFFFF",
                                               font_size=size)
                                               #justify="L")  # - seems to be ignored, always centers
        self.string_caption.sprite.position(x, y, z)
        self.string_caption.set_shader(pi3d.Shader('uv_flat'))  # the mat_flat shader doesn't work for text
        self.manual_redraw = True

    # TODO - update for 2D GUI
    def set_text(self, new_text):
        self.string_caption = pi3d.FixedString(camera=GUI_CAMERA, 
                                               font=self.font,
                                               string=new_text,
                                               color="#FFFFFF", )
        self.string_caption.sprite.width = self.width  # height depends on the font & scaling
        self.string_caption.sprite.scale(self.scale, self.scale, 0.001)
        self.string_caption.sprite.position(self.x, self.y, 0.95)# 1.0-(GUI_DEPTH*50))
        self.string_caption.set_shader(pi3d.Shader('uv_flat'))  # the mat_flat shader doesn't work for text

    def draw(self):
        self.string_caption.draw()


# DEPRECATED    
class OldLabel3D():
    # although these are technically widgets in the GUI sense,
    # they don't inherit from Widget, since they have no underlying cuboid
    # and render in a different way
    def __init__(self, x, y, width, 
                 text, font="/usr/share/fonts/truetype/piboto/PibotoCondensed-Bold.ttf"):
        TOP = -0.01
        LEFT = -0.02
        SCALE = 0.001
        self.text = text
        self.string_caption = pi3d.FixedString(camera=CAMERA, 
                                               font=font,
                                               string=self.text,
                                               color="#FFFFFF", )
                                               #justify="L")  # - seems to be ignored, always centers
        #self.string_caption.sprite.position(x+LEFT, y+TOP, 1.0-(GUI_DEPTH*50))
        self.string_caption.sprite.width = width  # height depends on the font & scaling
        self.string_caption.sprite.scale(SCALE, SCALE, 0.001)
        self.string_caption.sprite.position(x, y, .950)# 1.0-(GUI_DEPTH*50))
        self.string_caption.set_shader(pi3d.Shader('uv_flat'))  # the mat_flat shader doesn't work for text
        self.manual_redraw = True

    def draw(self):
        self.string_caption.draw()

class Button(Widget):
    def __init__(self, parent, x, y, width=180, height=70, size=32, text=""):
        super().__init__(parent, x, y, width , height)
        self.name = text
        self.caption = Label(self, x, y, size=size, text=text)  # The text you want on your button
        self.manual_redraw = True
        self.click = None

    def draw(self):
        # no need to draw the button box, since the GUI class will automatically
        # draw the parent widget cuboid
        self.caption.draw()

    # inject a function to use when the button is clicked
    def on_click(self, callback_function):
        self.click = callback_function

# TODO - update for 2D GUI
# what would be a JLabel in Swing - ie a passive box with some text
class TextPanel():
    # although these are technically widgets in the GUI sense,
    # they don't inherit from Widget, since they have no underlying cuboid
    # and render in a different way
    def __init__(self, x, y, width, 
                 text, font="/usr/share/fonts/truetype/piboto/PibotoCondensed-Bold.ttf"):
        TOP = -0.01
        LEFT = -0.02
        SCALE = 0.001
        self.text = text
        self.string_caption = pi3d.FixedString(camera=GUI_CAMERA, 
                                               font=font,
                                               string=self.text,
                                               color="#FFFFFF", )
                                               #justify="L")  # - seems to be ignored, always centers
        #self.string_caption.sprite.position(x+LEFT, y+TOP, 1.0-(GUI_DEPTH*50))
        self.string_caption.sprite.width = width  # height depends on the font & scaling
        self.string_caption.sprite.scale(SCALE, SCALE, 0.001)
        self.string_caption.sprite.position(x, y, .950)# 1.0-(GUI_DEPTH*50))
        self.string_caption.set_shader(pi3d.Shader('uv_flat'))  # the mat_flat shader doesn't work for text
        self.manual_redraw = True

    def draw(self):
        self.string_caption.draw()

# replace this with a separate text object and try and line them up properly!

    def draw(self):
#        self.base_rect.draw()
        self.string_caption.draw()

# the cursor is drawn on the GUI plane, in front of everything else
class MouseCursor:
    def __init__(self, shader, size=50):
        self.mouse = pi3d.Mouse(restrict=False)  # allow mouse coords to go -ve
        self.mouse.start()
        
        self.manual_redraw = True
        self.is_dragging = False  # keeps track of whether we are in the middle of a drag operation
        self.old_x = -1.0  # used to check if the mouse has moved since last update
        self.old_y = -1.0

        # Define line vertices for a basic cross centered at (0, 0)
        vertices = [[-size, 0, 0], [size, 0, 0], [0, -size, 0], [0, size, 0]]
        self.cursor = pi3d.Lines(vertices=vertices, material=(0,0,0), 
                                 closed=False, strip=False, line_width=3,
                                 camera=GUI_CAMERA,)
        self.cursor.set_shader(shader)
        self.cursor.set_material(GUI_FG)
        self.widget = self.cursor

    # check if the cursor has moved and update it
    # or switch to the smaller 'resting' cursor, if not
    def update_position(self):
        x, y = self.get_normalised_pos()
        z = 0  # mouse cursor is always in front of everything
        if x != self.old_x or y != self.old_y:
            self.old_x = x
            self.old_y = y
            self.cursor_timeout=time.monotonic() + CURSOR_HIDE_DELAY
            self.widget.scale(1.0, 1.0, 1.0)
            self.cursor.position(x, y, z)
        elif time.monotonic() > self.cursor_timeout:
            self.widget.scale(0.1, 0.1, 0.1)

    # convert the touchscreen coords 
    # so that they match the drawing coords for the GUI
    def get_normalised_pos(self):
        mx, my = pyautogui.position()
        norm_mx = mx - DISPLAY.width//2
        norm_my = DISPLAY.height//2 - my
#        print("x={}, y={}".format(mx,my))  # DEBUG
        return norm_mx, norm_my
    
    # True if left mouse button pressed
    def left_button(self):
        return self.mouse.button_status()==9  

    def draw(self):
        self.update_position()
        #print("x={}, y={}".format(*self.get_normalised_pos()))
        self.cursor.draw()  # no need since the base widget is drawn by the parent draw()


# MQTT STUFF ####################################
# these functions handle the connection and mesaging
# between the cyberdeck and any robots connected to it

# initialise MQTT for telemetry feed and commands
def connect_to_broker(mqtt_enabled = False):
    # abort the connection if we are just testing
    if not mqtt_enabled:
        return None
    print("Connecting to MQTT broker...")
    try:
        telemetry = mqtt.Client()
        telemetry.on_message = on_message
        telemetry.connect(BROKER, PORT, KEEP_ALIVE)
        telemetry.subscribe(TELEMETRY_TOPIC)
        telemetry.subscribe(COMMAND_TOPIC)
        print("Connected!")
        gui.unlock_buttons()
        return telemetry
    except Exception as e:
        print("Failed: {0}. Automatic retry in {1} seconds.".format(e, RECONNECT_RETRY))
        gui.lock_buttons()
        return None

# message handler for MQTT
def on_message(client, userdata, message):
    # ignore non telemetry messages - these will just be our own
    # outgoing command messages
    if message.topic == TELEMETRY_TOPIC:
        text = message.payload.decode('UTF-8')
        print("Telemetry rx:", text, "on topic", message.topic)
        # parse the data to decide what to do with it
        # Hand data  is of the form:
        #   LEFT_HAND, t:78, i:100, m:100, r:100, p:100, wrist:50
        # t,i,m,r,p are thumb, index, middle, ring, pinkie
        # numbers are percentage extension 0-100
        parsed = text.split(",")
        if parsed[0] == "LEFT_HAND":
            # check for status messages
            if parsed[1] == "status:READY":
                gui.unlock_buttons()
            elif parsed[1] == "status:BUSY":
                pass  # buttons are auto-locked when sending a command so no need to do anything else for now
            else:  # everything else assumed to be telemetry data
                # extract bend angles from the telemetry for each finger
                for i in range(5):
                    flex_and_angle = parsed[i+1].split(":")[1].split("/")
                    new_bend = int(flex_and_angle[1])
                    hand.get_finger(i).flex(new_bend)
                    flexion_values[i].set_text(str(new_bend))

# connection callback function - UNUSED?
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    #client.subscribe(TOPIC)

# transmit commands to one of the attached robot parts
def send_command(client, robot=None, command=None):
    # TODO check to see what is connected
    if telemetry:
        print("Sending command <{}> to device {}".format(command, robot))
        if robot == "HAND" and command:
            client.publish("hand/commands", "COMMAND:"+command)
            gui.lock_buttons()  # disable buttons until the command is complete

# handlers for the GUI buttons
def palm_button_click():
    send_command(telemetry, robot="HAND", command="PALM")

def fist_button_click():
    send_command(telemetry, robot="HAND", command="FIST")

def call_me_button_click():
    send_command(telemetry, robot="HAND", command="CALL_ME")

def metal_button_click():
    send_command(telemetry, robot="HAND", command="METAL")

################################################################
# MAIN PROGRAM

print("+++ CYBERDECK STARTING UP +++")

# Set up the pi3D display elements
DISPLAY = pi3d.Display.create(display_config=pi3d.DISPLAY_CONFIG_HIDE_CURSOR)  # full screen with no w & h arguments
DISPLAY.set_background(*BACKGROUND_COLOR)
CAMERA = pi3d.Camera()  # the constant isn't actually used - just initialises the default camera
GUI_CAMERA = pi3d.Camera(is_3d=False)  # used for all the 2D overlay stuff
SHADER = pi3d.Shader("mat_flat")  # Use a flat shader without texture
hand = Hand(-1.8, -0.1, 5, size=1.2, shader=SHADER)  # 3D coords range -2.5 to +2.5 in the x

# initialise the GUI layer
gui = GUI(SHADER)
rotation_slider = Slider(parent=gui, x=-200, y=-220, width=500)
palm_button = Button(parent=gui, x=600, y=450, width=180, height=70, text="palm")
palm_button.on_click(palm_button_click)  # register event handler
fist_button = Button(parent=gui, x=800, y=450, width=180, height=70, text="fist")
fist_button.on_click(fist_button_click)  # register event handler
call_me_button = Button(parent=gui, x=600, y=350, width=180, height=70, text="call me")
call_me_button.on_click(call_me_button_click)  # register event handler
metal_button = Button(parent=gui, x=800, y=350, width=180, height=70, text="metal")
metal_button.on_click(metal_button_click)  # register event handler
#menu_label = TextPanel(parent=gui, shader=SHADER, x=3.0, y=0.0, width=2, height=1.4, 
#                       text="test/nline2")

# labels to display finger telemetry values
flexion_values = []
# set the position to be near the bottom left of the screen
# 0,0 is the middle, current screen size is 1920 x 1080 but might change in the future
x = -DISPLAY.width * 0.4
y = -DISPLAY.height * 0.4
font_size = 48
for f in 'TIMRP':  # Thumb Index Middle Ring Pinkie
    gui.add(Label(gui, x, y, size=font_size, text=f))
    label = Label(gui, x, y-50, size=font_size, text="00")
    flexion_values.append(label)
    gui.add(label)
    x += font_size*1.5

gui.add(rotation_slider)
gui.add(palm_button)
gui.add(fist_button)
gui.add(call_me_button)
gui.add(metal_button)
#gui.add(menu_label)1

# setup keyboard input
keyboard = pi3d.Keyboard()

# initial hand state & matching slider position
hand.palm.block.rotateToY(-45)
#pos_ratio = rotation_slider.move_handle(-rotation_slider.width/4)
#if pos_ratio is not None:
#    angle = pos_ratio * 180 -90
#    hand.palm.block.rotateToY(angle)

# initialise MQTT for telemetry feed and commands
telemetry = connect_to_broker(MQTT_ENABLED)

# main loop
reconnect = time.monotonic() + RECONNECT_RETRY
while DISPLAY.loop_running():
    # check for GUI input
    gui.check_events()

    if telemetry:
        print("checking keyb")
        # check keyboard and send any keypresses to the currently active robot
        key_event = keyboard.read()
        if key_event != -1:
            print("key=",key_event)  # DEBUG
            send_command(telemetry, robot="HAND", command=chr(key_event))

        telemetry.loop(timeout=0.01)  # process MQTT network traffic, dispatch callbacks
    elif MQTT_ENABLED and time.monotonic() > reconnect:
        print("reconnecting")
        telemetry = connect_to_broker()  # attempt to connect again
        if not telemetry:
            reconnect = time.monotonic() + RECONNECT_RETRY  # reset timer for next attempt
    
    hand.draw()
    gui.draw()



