import odrive
import pygame


pygame.init()

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates.
clock = pygame.time.Clock()

# Initialize the joysticks.
pygame.joystick.init()

# -------- Main Program Loop -----------
while not done:
    #
    # EVENT PROCESSING STEP
    #
    # Possible joystick actions: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
    # JOYBUTTONUP, JOYHATMOTION
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
            done = True
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    print("Number of joysticks: {}".format(joystick_count))
    
    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        
        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        
        for i in range(axes):
            axis = joystick.get_axis(i)
            print("Axis {} value: {:>6.3f}".format(i, axis))

    # Limit to 20 frames per second.
    clock.tick(20)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()

# odv = odrive.find_any()
# odv.axis0

# A - White
# B - Blue
# C - Yellow

# Set pole pairs, cpr, and encoder mode
# odrv0.axis0.motor.config.pole_pairs = 7
# odrv0.axis0.encoder.config.cpr = 42
# odrv0.axis0.encoder.config.mode = ENCODER_MODE_HALL
# odrv0.config.brake_resistance = 5
# odrv0.config.enable_brake_resistor = True
# Other parameters: vel gain, breaking current

# Individual Callibration
# odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
# odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
# odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

# 1 command calibration
# odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# Running odrive
# odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Velocity Control
# odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
# odrv0.axis0.controller.input_vel = 1