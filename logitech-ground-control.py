import serial
import time
import pygame

#Initialize the 9 channel variables, globally
thr = 1000
pitch = 0.0
roll = 0.0
yaw = 0.0
x = 0
a = 0
b = 0
y = 0
back = 0
thrTrim = 0

SIZE = width,height = 320,240 #pygame needs a size for its popup window
clock = pygame.time.Clock() #create a clock instance to use for ticking
screen = pygame.display.set_mode(SIZE)

def Initialize_Controller():
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    print count, "controller(s) connected."

    global Logitech #globalize the controller instance
    Logitech = pygame.joystick.Joystick(0)
    Logitech.init() #initalize the controller instance

    print "Controller:", Logitech.get_name()
    print "Axes detected:", Logitech.get_numaxes()
    print "Buttons detected:", Logitech.get_numbuttons()


def init_serial(comPort, baudRate, timeout=10.0):
    #print out all of the available COM ports
    from serial.tools import list_ports
    print "AVAILABLE SERIAL PORTS:"
    for i in list_ports.comports():
        print i
    print "-----------------------"


    global ser #the serial object must be global

    #initialize and open the serial port
    ser = serial.Serial(comPort, baudRate)

    #print port open or closed
    if ser.isOpen():
        print 'Open: ' + ser.portstr



def Read_Controller(Joystick):
	"""
	Reads all of the states on the controller, returns them as a list of values
	"""
	global thrTrim,thr,pitch,roll,yaw,x,a,b,y,back

	#allow more precise throttle control

 	if y==1:
		thrTrim+=1
	if a==1:
		thrTrim-=1
	if b==1:
		thrTrim=0 #reset the throttle trim

	#update the axes
	if abs(Joystick.get_axis(1)) > 0.05:
		thr = thr -10*Joystick.get_axis(1) #the quadcopter uses throttle values from 1000 to 2000

	pitch = -15*Joystick.get_axis(3) #pitch and roll go from -45 to 45 deg
	roll = 15*Joystick.get_axis(2)
	yaw = 40*Joystick.get_axis(0) #yaw goes from -150 to 150 deg

	#update the buttons
	x = Joystick.get_button(0)
	y = Joystick.get_button(3)
	a = Joystick.get_button(1)
	b = Joystick.get_button(2)
	back = Joystick.get_button(8)

	#if any of the values are really close to zero, set them to zero
	if thr<1000:
		thr = 1000
	if abs(pitch)<1:
		pitch = 0
	if abs(roll)<1:
		roll = 0
	if abs(yaw)<1:
		yaw = 0


	return [thr+thrTrim,pitch,roll,yaw,x,a,b,y,back]



def Run_Ground_Control(sampleRate, baudRate = 57600, comPort = 'COM5'):
	"""
	Samples the controller and sends a packet with 8 channels"
	thr = 0.0
	pitch = 0.0
	roll = 0.0
	yaw = 0.0
	x = 0
	a = 0
	b = 0
	y = 0
	back = 0 // Note: "back" is used to terminate communication, but is not sent as part of the packet

	"""
	Initialize_Controller() #run the helper function to start up the controller
	init_serial('COM5', 57600) #initalize the serial connection to COM port 5


	while True:
		pygame.event.pump() # must pump events to keep updating the controller state

		controls = Read_Controller(Logitech)

		#if the back button is pressed, quit
		if controls[8] == 1:
			print "Quitting."
			break

		command = "c" + \
	    		":" + str(int(controls[0]+thrTrim)) + \
	    		":" + str(int(controls[1])) + \
	    		":" + str(int(controls[2])) + \
	    		":" + str(int(controls[3])) + \
	    		":" + str(int(controls[4])) + \
	    		":" + str(int(controls[5])) + \
	    		":" + str(int(controls[6])) + \
	    		":" + str(int(controls[7])) + \
	    		":/"
	    		
		print "Sending packet:", command
		ser.write(command)

		pygame.time.delay(int(float(1000)/sampleRate)) #this takes care of the sample rate


#RUNNING THE GROUND CONTROL
Run_Ground_Control(50)
ser.close()
Logitech.quit()