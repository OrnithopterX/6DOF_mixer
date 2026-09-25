from vpython import *
import numpy as np

from main import Aircraft, Quaternion
from tx16s import TX16S


use_tx16s = False 
use_keyboard = True


# <<<<<<<<<< Scene configuration (Static) >>>>>>>>>> #

'''  << axis conventions >> 
        +X = forward
        +Y = left 
        +Z = up
'''

scene = canvas(
    title="Quaterion Transformation Simulation",
    width=1200,
    height=700,
    background=color.black
    #background=vector(0.1, 0.1, 0.1) #RGB value, if needed
)

scene.center = vector(0, 0, 0)
scene.range = 5
scene.up = vector(0, 0, 1) # set the z-axis as up, not y. 
scene.forward = vector(1, 1, -0.5) # initial camera position

aircraft = Aircraft()


# <<<<<<<<<< GLOBAL COMMANDS >>>>>>>>>> #

if use_tx16s:
    tx = TX16S()

desired_translation_global = np.zeros(3)
desired_rotation_global = np.zeros(3)

'''
else: 
    desired_translation_global = np.array([
        0.0,
        1.0,
        0.4
    ])

    desired_rotation_global = np.array([
        0.0,
        1.0,
        0.0
    ])
'''

# <<<<<<<<<< Simulation >>>>>>>>>> #

running = True
dt = 1 / 60


# <<<<<<<<<<<< Reset and Pause >>>>>>>>>> # 

def reset():

    aircraft.position = np.zeros(3)
    aircraft.orientation = Quaternion()


def pause():

    global running

    running = not running

    if running:
        pause_button.text = "Pause"
    else:
        pause_button.text = "Resume"


# define keyboard binds 
pressed_keys = set()

def keyboard_handler(event):

    key = event.key.lower()

    pressed_keys.add(key)

    if key == "p":
        pause()

    elif key == "t":
        reset()


def keyboard_release_handler(event):

    key = event.key.lower()

    pressed_keys.discard(key)



# <<<<<<<<<< Controls >>>>>>>>>> # 

scene.append_to_caption("\n")

reset_button = button(
    text="Reset",
    bind=reset
)

scene.append_to_caption("    ")

pause_button = button(
    text="Pause",
    bind=pause
)

scene.append_to_caption("\n\n")

scene.bind("keydown", keyboard_handler)
scene.bind("keyup", keyboard_release_handler)


# <<<<<<<<<< Keyboard control >>>>>>>>>>>> # 

def update_keyboard():

    desired_translation_global[:] = 0
    desired_rotation_global[:] = 0

    # Translation
    if "w" in pressed_keys:
        desired_translation_global[0] = 1
    if "s" in pressed_keys:
        desired_translation_global[0] = -1

    if "a" in pressed_keys:
        desired_translation_global[1] = 1
    if "d" in pressed_keys:
        desired_translation_global[1] = -1

    if "r" in pressed_keys:
        desired_translation_global[2] = 1
    if "f" in pressed_keys:
        desired_translation_global[2] = -1

    # Rotation
    if "i" in pressed_keys:
        desired_rotation_global[0] = 1
    if "k" in pressed_keys:
        desired_rotation_global[0] = -1

    if "j" in pressed_keys:
        desired_rotation_global[1] = 1
    if "l" in pressed_keys:
        desired_rotation_global[1] = -1

    if "u" in pressed_keys:
        desired_rotation_global[2] = 1
    if "o" in pressed_keys:
        desired_rotation_global[2] = -1


# <<<<<<<<<< TX16s passthrough >>>>>>>>>> #

def update_controller():

    tx.update()

    desired_translation_global[:] = np.array([
        tx.axis(1),   # CH1 / X => forward-back
        tx.axis(0),   # CH2 / Y => left-right
        tx.axis(2)    # CH3 / Z => vertical
    ])

    desired_rotation_global[:] = np.array([
        tx.axis(5),   # CH4 / x => roll
        tx.axis(4),   # CH5 / y => pitch
        tx.axis(3)    # CH6 / z => yaw
    ])


# <<<<<<<<<< Telemetry >>>>>>>>>>>> # 


telemetry = wtext(text="")


# <<<<<<<<<< Global coordinate frame (Static) >>>>>>>>>> # 

global_x = arrow(
    pos=vector(0, 0, 0),
    axis=vector(3, 0, 0),
    shaftwidth=0.04,
    color=color.red
)

global_y = arrow(
    pos=vector(0, 0, 0),
    axis=vector(0, 3, 0),
    shaftwidth=0.04,
    color=color.green
)

global_z = arrow(
    pos=vector(0, 0, 0),
    axis=vector(0, 0, 3),
    shaftwidth=0.04,
    color=color.blue
)

''' commented out axis labels to keep UI clean

label(
    pos=vector(3.2, 0, 0),
    text="+X  FORWARD",
    box=False,
    opacity=0
)

label(
    pos=vector(0, 3.2, 0),
    text="+Y  LEFT",
    box=False,
    opacity=0
)

label(
    pos=vector(0, 0, 3.2),
    text="+Z  UP",
    box=False,
    opacity=0
)
'''


# <<<<<<<<<< Aircraft body >>>>>>>>>>>> #

body = box(
    pos=vector(0, 0, 0),
    size=vector(0.8, 0.4, 0.6),
    color=color.white
)

local_x = arrow(
    pos=vector(0, 0, 0),
    axis=vector(2, 0, 0),
    shaftwidth=0.08,
    color=color.red
)

local_y = arrow(
    pos=vector(0, 0, 0),
    axis=vector(0, 1.5, 0),
    shaftwidth=0.08,
    color=color.green
)

local_z = arrow(
    pos=vector(0, 0, 0),
    axis=vector(0, 0, 1.5),
    shaftwidth=0.08,
    color=color.blue
)



# <<<<<<<<<< Command vectors >>>>>>>>>> # 

# desired global translation state
global_translation_arrow = arrow(
    pos=vector(0, 0, 0),
    axis=vector(0, 0, 0),
    shaftwidth=0.12,
    color=color.yellow
)

# translated command expressed in local (body) coordinates,
# then converted back to global coordinates for visualization.
body_translation_arrow = arrow(
    pos=vector(0, 0, 0),
    axis=vector(0, 0, 0),
    shaftwidth=0.10,
    color=color.magenta
)


# <<<<<<<<<< Utility >>>>>>>>>> # 

def np_to_vector(v):

    return vector(
        v[0],
        v[1],
        v[2]
    )


# <<<<<<<<<< Visualization >>>>>>>>>> # 

def update_visualization():

    position = np_to_vector(
        aircraft.position
    )


    # Local axes
    x = aircraft.body_to_global(
        np.array([2.0, 0.0, 0.0])
    )

    y = aircraft.body_to_global(
        np.array([0.0, 1.5, 0.0])
    )

    z = aircraft.body_to_global(
        np.array([0.0, 0.0, 1.5])
    )

    local_x.pos = position
    local_x.axis = np_to_vector(x)

    local_y.pos = position
    local_y.axis = np_to_vector(y)

    local_z.pos = position
    local_z.axis = np_to_vector(z)


    # Aircraft
    body.pos = position
    body.axis = np_to_vector(x)
    body.up = np_to_vector(z)


    # Global commands
    global_translation_arrow.pos = position
    global_translation_arrow.axis = np_to_vector(
        desired_translation_global
    )


    # Global -> Body transformation
    translation_body = aircraft.global_to_body(
        desired_translation_global
    )

    rotation_body = aircraft.global_to_body(
        desired_rotation_global
    )


    # Body -> Global
    # Only used so the local command can be displayed in the 3D global scene.
    translation_global_from_body = aircraft.body_to_global(
        translation_body
    )

    body_translation_arrow.pos = position
    body_translation_arrow.axis = np_to_vector(
        translation_global_from_body
    )


    # <<<<<<<<<< Telemetry >>>>>>>>>> #

    q = aircraft.orientation

    telemetry.text = (

        "<b>OUTPUT DATA</b><br><br>"

        "<b>GLOBAL TRANSLATION COMMAND</b><br>"
        f"X: {desired_translation_global[0]: .4f}&nbsp;&nbsp;"
        f"Y: {desired_translation_global[1]: .4f}&nbsp;&nbsp;"
        f"Z: {desired_translation_global[2]: .4f}"
        "<br><br>"

        "<b>LOCAL TRANSLATION COMMAND</b><br>"
        f"X: {translation_body[0]: .4f}&nbsp;&nbsp;"
        f"Y: {translation_body[1]: .4f}&nbsp;&nbsp;"
        f"Z: {translation_body[2]: .4f}"
        "<br><br>"

        "<b>GLOBAL ROTATION COMMAND</b><br>"
        f"X: {desired_rotation_global[0]: .4f}&nbsp;&nbsp;"
        f"Y: {desired_rotation_global[1]: .4f}&nbsp;&nbsp;"
        f"Z: {desired_rotation_global[2]: .4f}"
        "<br><br>"

        "<b>LOCAL ROTATION COMMAND</b><br>"
        f"X: {rotation_body[0]: .4f}&nbsp;&nbsp;"
        f"Y: {rotation_body[1]: .4f}&nbsp;&nbsp;"
        f"Z: {rotation_body[2]: .4f}"
        "<br><br>"

        "<b>ORIENTATION QUATERNION</b><br>"
        f"w: {q.w: .4f}<br>"
        f"x: {q.x: .4f}<br>"
        f"y: {q.y: .4f}<br>"
        f"z: {q.z: .4f}<br><br>"

        "<b>POSITION</b><br>"
        f"X: {aircraft.position[0]: .4f}<br>"
        f"Y: {aircraft.position[1]: .4f}<br>"
        f"Z: {aircraft.position[2]: .4f}"
    )



# <<<<<<<<<<< Main loop >>>>>>>>>> #

while True:

    rate(60)

    if use_tx16s: 
        update_controller()

    if use_keyboard:
        update_keyboard()

    if running:

        # Apply global translation command.
        aircraft.move_global(
            desired_translation_global,
            dt
        )

        # Apply global rotation command.
        aircraft.rotate_global(
            desired_rotation_global,
            dt
        )

    print(desired_translation_global) ## remove after testing

    update_visualization()