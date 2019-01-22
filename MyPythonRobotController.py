#!/usr/bin/python2.

############################################################################
#  Python Roomba Controller  
#  by Paul Schmitt.  March 2016.
#
#  Based upon the great Create 2 Tethered Driving Project code.  Thanks!  
#  To begin your Python Robot Controller program, go to the bottom of this 
#  program and look for "Start my Python Robot Controller below here".
#  
###########################################################################

from Tkinter import *
#import tkMessageBox
#import tkSimpleDialog
import math, numpy
import time
import thread
import socket
import select
import struct
import random
import sys, glob # for listing serial ports
import os  # to command the mp3 and wav player omxplayer

g_nobeep=False
g_bumped=False

try:
    import serial
except ImportError:
    print "Import error.  Please install pyserial."
    raise

connection = None
global FAILURE
FAILURE = False

def toTwosComplement2Bytes( value ):
        """ returns two bytes (ints) in high, low order
        whose bits form the input value when interpreted in
        two's complement
        """
        # if positive or zero, it's OK
        if value >= 0:
            eqBitVal = value
            # if it's negative, I think it is this
        else:
            eqBitVal = (1<<16) + value
    
        return ( (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF )

# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
    cmd = ""
    for v in command.split():
        cmd += chr(int(v))
    sendCommandRaw(cmd)

# sendCommandRaw takes a string interpreted as a byte array
def sendCommandRaw(command):
    global connection
    try:
        if connection is not None:
            connection.write(command)
        else:
            print "Not connected."
    except serial.SerialException:
        print "Lost connection"
        connection = None
    #print ' '.join([ str(ord(c)) for c in command ])

# getDecodedBytes returns a n-byte value decoded using a format string.
# Whether it blocks is based on how the connection was set up.
def getDecodedBytes( n, fmt):
    global connection
        
    try:
        return struct.unpack(fmt, connection.read(n))[0]
    except serial.SerialException:
        print "Lost connection"
        tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
        connection = None
        return None
    except struct.error:
        print "Got unexpected data from serial port."
        return None

def bytesOfR( r ):
        """ for looking at the raw bytes of a sensor reply, r """
        print('raw r is', r)
        for i in range(len(r)):
            print('byte', i, 'is', ord(r[i]))
        print('finished with formatR')

def toBinary( val, numBits ):
        """ prints numBits digits of val in binary """
        if numBits == 0:  return
        toBinary( val>>1 , numBits-1 )
#        print((val & 0x01), end=' ')  # print least significant bit

def bitOfByte( bit, byte ):
    """ returns a 0 or 1: the value of the 'bit' of 'byte' """
    if bit < 0 or bit > 7:
        print('Your bit of', bit, 'is out of range (0-7)')
        print('returning 0')
        return 0
    return ((byte >> bit) & 0x01)

# get8Unsigned returns an 8-bit unsigned value.
def get8Unsigned():
    return getDecodedBytes(1, "B")

# get lowest bit from an unsigned byte
def getLowestBit():
    wheelsAndBumpsByte = getDecodedBytes(1, "B")
    print wheelsAndBumpsByte
    return bitOfByte(0, wheelsAndBumpsByte)

# get second lowest bit from an unsigned byte
def getSecondLowestBit():
    wheelsAndBumpsByte = getDecodedBytes(1, "B")
    print wheelsAndBumpsByte
    return bitOfByte(1, wheelsAndBumpsByte)

def bumped():
    try:
        sendCommandASCII('142 7') 
        time.sleep( 0.02 )
        bumpedByte = getDecodedBytes( 1, "B" )
        if bumpedByte == 0:
            return False
        elif bumpedByte > 3:
            print "CRAZY BUMPER SIGNAL!"
        elif bumpedByte == None:
            return None
        else:
            return True
    except Exception as e:
        return None

def cleanButtonPressed():
    sendCommandASCII('142 18') 
    buttonByte = getDecodedBytes( 1, "B" )
    if buttonByte == 0:
	return False
    elif buttonByte == 1:
	print "Clean Button Pressed!"
	return True
    elif buttonByte == 4:
	return False
    else:
	print "Some other button pressed!"
	FAILURE = True
	return False

def dockButtonPressed():
    sendCommandASCII('142 18') 
    buttonByte = getDecodedBytes( 1, "B" )
    if buttonByte <> 4:
	return False
    else:
	print "Dock button pressed!"
	return True

def shudder( period, magnitude, numberOfShudders):
    i = 0
    timestep = 0.02
    while i < numberOfShudders:
	i = i + 1
	#shake left
	t = 0
	while t < period:
	    driveDirectRot( 0, magnitude )
	    t = t + timestep
	    time.sleep( timestep )
	#Shake right
	t = 0
	while t < period:
	    driveDirectRot( 0, -magnitude )
	    t = t + timestep
	    time.sleep( timestep )
    driveDirect( 0, 0 )  # stop the previous motion command

def onConnect():
    global connection

    if connection is not None:
        print "Oops- You're already connected!"
        return

    try:
        ports = getSerialPorts()
	print "Available ports:\n" + '   '.join(ports)
        #port = raw_input("Port? Enter COM port to open.\nAvailable options:\n" + '\n'.join(ports))
	port = str( ports[0] )  # I'm guessing that the Roomba port is first in the list.  So far this works!  :)
    except EnvironmentError:
        port = raw_input("Port?  Enter COM port to open.")

    if port is not None:
        print "Trying " + str( port ) + "... "
    try:   #:tty
        #connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        #connection = serial.Serial( str(port), baudrate=115200, timeout=1 )
        connection = serial.Serial( str(ports[0]), baudrate=115200, timeout=1 )
        print "Connected!"
    except:
        print "Failed.  Could not connect to " + str( port )

def getSerialPorts():
    """Lists serial ports
    From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result    

def driveDirectTime( left, right, duration ):
    #print"driveDirectTime()"
    t = 0   # initialize timer
    while t < duration:
        if g_bumped is True:
            break
        driveDirect( left, right )
        time.sleep( 0.05 )
        t = t + .05
    driveDirect( 0, 0 )  # stop

def driveDirect( leftCmSec = 0, rightCmSec = 0 ):
    """ sends velocities of each wheel independently
           left_cm_sec:  left  wheel velocity in cm/sec (capped at +- 50)
           right_cm_sec: right wheel velocity in cm/sec (capped at +- 50)
    """
    #print "driveDirect()"
    if leftCmSec < -50: leftCmSec = -50
    if leftCmSec > 50:  leftCmSec = 50
    if rightCmSec < -50: rightCmSec = -50
    if rightCmSec > 50: rightCmSec = 50
    # convert to mm/sec, ensure we have integers
    leftHighVal, leftLowVal = toTwosComplement2Bytes( int( leftCmSec * 10 ) )
    rightHighVal, rightLowVal = toTwosComplement2Bytes( int( rightCmSec * 10 ) )

    # send these bytes and set the stored velocities
    byteListRight = ( rightHighVal , rightLowVal )
    byteListLeft = ( leftHighVal , leftLowVal )
    sendCommandRaw(struct.pack( ">Bhh", 145, int(rightCmSec * 10), int(leftCmSec * 10) ))
    return

def driveDirectRot( robotCmSec = 0, rotation = 0 ):
    """ implements the driveDirect with a given rotation
        Positive rotation turns the robot CCW
        Negative rotation turns the robot CW
    """
    #print "driveDirectRot()"
    vl = robotCmSec - rotation/2
    vr = robotCmSec + rotation/2
    driveDirect ( vl, vr )

def initiateRobotCommunication():
    print "Initiating Communications to the Create 2 Robot..."
    onConnect()
    time.sleep( 0.3 )
    sendCommandASCII('128')   # Start Open Interface in Passive
    time.sleep( 0.3 )
    if not g_nobeep:
        sendCommandASCII('140 3 1 64 16 141 3')  # Beep
    time.sleep( 0.3 )
    sendCommandASCII('131')   # Safe mode
    #sendCommandASCII( '132' )   # Full mode 
    time.sleep( 0.3 )
    if not g_nobeep:
        sendCommandASCII('140 3 1 64 16 141 3')  # Beep
    time.sleep( 0.1 )
    sendCommandASCII('139 4 0 255')  # Turn on Clean and Dock buttons
    time.sleep( 0.03 )

def closeRobotCommunication():
    print "Closing Communication to the Create 2 Robot..."
    driveDirect( 0, 0 )  # stop robot if moving
    time.sleep( 0.05 )
    if not g_nobeep:
        sendCommandASCII('140 3 1 64 16 141 3')  # Beep
    time.sleep( 0.3 )
    #sendCommandASCII('139 0 0 0')  # Turn off Clean and Dock buttons
    time.sleep( 0.03 )
    sendCommandASCII('138 0')  # turn off vacuum, etractors, and side brush
    time.sleep( 0.03 )
    #sendCommandASCII( '7' )  # Resets the robot 	
    sendCommandASCII( '173' )  # Stops the Open Interface to Roomba
    time.sleep( 0.3 )
    connection.close()
    time.sleep( 0.1 )
    raise SystemExit	#  Exit program


##########################################################################
##########################################################################
#---------Start my Python Robot Controller Program below here!------------
#---------Change the code below
##########################################################################
##########################################################################


#START OF AMELIA AND KEVIN ADDITIONS

#set this to True to turn off the beep when the connection to the robot is successful
g_nobeep=True


#constants, do not change these
'''
The measured wheel diameter from the center of each wheel is 23.5 cm.
wheel_diameter_cm needs to be set to 22 for rotate_degrees
to rotate the correct amount.  When a speed is specified (up to
50 cm/sec), is that actually the speed the robot is moving?
'''
wheel_diameter_cm = 22.0
wheel_radius_cm = wheel_diameter_cm/2
wheel_travel_cm_for_180degrees = wheel_radius_cm * 3.14159
wheel_travel_cm_for_1degree = wheel_travel_cm_for_180degrees/180

#initialize g_keeprunning
g_keeprunning=True

def signal_handler(sig, frame):
    global g_keeprunning
    print 'Caught CTRL-C'
    g_keeprunning=False
    closeRobotCommunication()
    time.sleep(.1)
    sys.exit(0)


import tempfile
import signal
class do_awesomeness:
    def __init__(self, enablerobot=True, print_dbg_messages=False):

        self.print_welcome_banner()

        self.tempfile_handle = tempfile.TemporaryFile('w+b')
        self.enable_robot = enablerobot
        self.print_dbg_messages = print_dbg_messages

        self.function_list=[
            driveDirect,
            driveDirectTime,
            time.sleep,
            driveDirectRot,
            shudder,
            self.rotate_degrees
        ]

        #Setup a signal handler to shut down comms when ctrl-c is pressed
        signal.signal(signal.SIGINT, signal_handler)

        #Open the robot communication interface 
        if self.enable_robot is True:
            initiateRobotCommunication()
            #bumped() returns None if connection failed
            if bumped() is None:
                print 'Could not communicate with the robot. If the clean button is not illuminated, press the button, and re-run this script.'
                sys.exit(1)
            thr = threading.Thread(target=testthread)
            thr.start()
            #thr.run()

    def print_helpscreen(self):
        print 'Help menu.  If the robot does not respond to any of the commands, make sure that you pressed the'
        print 'button and that it is illuminated'
        print 'Available functions:'
        print '\t0: driveDirect(left_speed, right_speed)'
        print '\t1: driveDirectTime(left_speed, right_speed, seconds to drive before stopping)'
        print '\t2: sleep(seconds)'
        print '\t3: driveDirectRot(rotation_speed(cm/sec), rotation(neg=ccw, pos=cw)'
        print '\t4: shudder(period, magnitude, number of shudders)'
        print '\t5 X rotate X degrees. Negative degrees == counterclockwise, positive degrees == clockwise'
        print 'The shortcut commands below will execute immediately, skipping all prompts. Positive values are forward, \nvalues are backwards.'
        print '\t100 stop. Positive value only.'
        print '\t101 seek_dock. Positive value only.'
        print '\t102 drive straight forward/backwards at 25% of maximum speed' 
        print '\t103 drive straight forward/backwards at 50% of maximum speed' 
        print '\t104 drive straight forward/backwards at 75% of maximum speed' 
        print '\t105 drive straight forward/backwards at 100% of maximum speed' 
        print '\t106 sharp circle left, 50% speed'
        print '\t107 medium circle left, 50% speed'
        print '\t108 slight circle left, 50% speed'
        print '\t109 sharp circle right, 50% speed'
        print '\t110 medium circle right, 50% speed'
        print '\t111 slight circle right, 50% speed'
        print '\t112 X rotate X degrees. Negative degrees == counterclockwise, positive degrees == clockwise'
        print '\t113 X Y Drive for X centimeters at Y centimeters per second, then stop.'

    def debug(self, msg):
        if self.print_dbg_messages is True:
            print '%s'%msg

    def drive_distance(self, distance_cm, speed_cm_sec):
        self.debug('Calculating time to drive %d cm at %d cm/sec'%(distance_cm, speed_cm_sec))
        drivetime = distance_cm/float(abs(speed_cm_sec))
        self.debug('Driving %d centimeters for %d seconds'%(distance_cm, drivetime))
        driveDirectTime(speed_cm_sec, speed_cm_sec, drivetime)


    def rotate_degrees(self, degrees):
        #negative degrees == rotate left, positive degrees == rotate right
        #rspeed and lspeed are in centimeters per second
        speed_cm_sec = 25.0
        rspeed = speed_cm_sec 
        lspeed = speed_cm_sec * -1 
        if degrees > 0:
            rspeed = speed_cm_sec * -1
            lspeed = speed_cm_sec
        time_sec_cm = 1/speed_cm_sec
        print 'time_sec_cm = %f'%time_sec_cm
        wheel_travel_cm_for_degrees = wheel_travel_cm_for_1degree * abs(degrees)
        self.debug('wheel_travel_cm_for_degrees = %f'%wheel_travel_cm_for_degrees)
        seconds_to_rotate = time_sec_cm * wheel_travel_cm_for_degrees
        self.debug('seconds to rotate = %f'%seconds_to_rotate)
        
        #calculate the amount of time to rotate at the above speed values
        #seconds_to_rotate = 1/(speed / (wheel_travel_cm_for_1degree * abs(degrees)))
        driveDirectTime(lspeed, rspeed, seconds_to_rotate)


    def read_existing_commands(self, filename, commands):
        with open(filename, 'r') as f:
            commandlist = str(f.read())
            command = commandlist.split('\n')[0:-1]
            print 'command = %s'%str(command)
            for i in range(len(command)):
                print 'processing command %s'%command[i]
                self.tempfile_handle.write(command[i] + '\n')
                command_list = command[i].split(' ')
                if len(command_list) == 2:
                    commands.append([int(command_list[0]), int(command_list[1])])
                elif len(command_list) == 3:
                    commands.append([int(command_list[0]), int(command_list[1]), int(command_list[2])])
                elif len(command_list) == 4:
                            commands.append([int(command_list[0]), int(command_list[1]), int(command_list[2]), int(command_list[3])])
                else:
                    raise Exception('Either two or three argumebnts required')
            return commands

    def get_input(self):
        global g_bumped
        self.print_helpscreen()
        existing_commands=[]
        commands=[]
        while g_keeprunning is True:
            if g_bumped is True:
                print 'Bumped into something. Exiting.'
                sys.exit(1)
            command = raw_input('Enter the name of a file to open that contains the commands, or enter the command number\nfollowed by arguments, space delimited. Enter ctrl-c to exit the script: \n')
            if g_bumped is True:
                print 'Bumped into something. Exiting.'
                sys.exit(1)
            
            if len(command) == 0:
                ans = raw_input('Enter the name of the file to save your commands to, or just press enter to not save \nthem to a file: ')
                try:
                    with open(ans, 'w') as f:
                        self.tempfile_handle.seek(0)
                        data = self.tempfile_handle.read()
                        f.write(data)
                except Exception as e:
                    pass
                break
            
            elif os.path.exists(command):
                existing_commands = self.read_existing_commands(command, commands)
                ans = raw_input('Would you like to enter more commands in addition to the commands you read from %s? [y/N] :')
                if ans == 'y' or ans == 'Y':
                    pass
                else:
                    return existing_commands

            elif abs(int(command.split(' ')[0])) >= 100:
                command_id_list = command.split(' ')
                command_id = int(command_id_list[0])
                if abs(command_id) == 100:
                    print '\t%d stop'%(command_id)
                    driveDirect(0, 0)
                elif abs(command_id) == 101:
                    print '\t%d seek_dock'%(command_id)
                    sendCommandASCII('143')   # Start Open Interface in Passive
                elif abs(command_id) == 102:
                    print '\t%d drive straight forward/backwards at 25 percent of maximum speed'%(command_id)
                    speed = 12
                    if command_id == -102:
                        speed = -12
                    driveDirect(speed, speed)
                elif abs(command_id) == 103:
                    print '\t%d drive straight forward/backwards at 50 percent of maximum speed'%(command_id)
                    speed = 25
                    if command_id == -103:
                        speed = -25
                    driveDirect(speed, speed)
                elif abs(command_id) == 104:
                    print '\t%d drive straight forward/backwards at 75 percent of maximum speeddd'%(command_id)
                    speed = 37
                    if command_id == -104:
                        speed = -37
                    driveDirect(speed, speed)
                elif abs(command_id) == 105:
                    print '\t%d drive straight forward/backwards at 100 percent of maximum speed'%(command_id)
                    speed = 50
                    if command_id == -105:
                        speed = -50
                    driveDirect(speed, speed)
                elif abs(command_id) == 106:
                    print '\t%d sharp circle left, 50 percent speed'%(command_id)
                    lspeed = 10
                    rspeed = 50
                    if command_id < 0:
                        lspeed = -10
                        rspeed = -50
                    driveDirect(lspeed, rspeed)
                elif abs(command_id) == 107:
                    print '\t%d medium circle left, 50 percent speed'%(command_id)
                    lspeed = 25
                    rspeed = 50
                    if command_id < 0:
                        lspeed = -25
                        rspeed = -50
                    driveDirect(lspeed, rspeed)
                elif abs(command_id) == 108:
                    print '\t%d slight circle left, 50 percent speed'%(command_id)
                    lspeed = 40
                    rspeed = 50
                    if command_id < 0:
                        lspeed = -40
                        rspeed = -50
                    driveDirect(lspeed, rspeed)
                elif abs(command_id) == 109:
                    print '\t%d sharp circle right, 50 percent speed'%(command_id)
                    lspeed = 50
                    rspeed = 10
                    if command_id < 0:
                        lspeed = -50
                        rspeed = -10
                    driveDirect(lspeed, rspeed)
                elif abs(command_id) == 110:
                    print '\t%d medium circle right, 50 percent speed'%(command_id)
                    lspeed = 50
                    rspeed = 25
                    if command_id < 0:
                        lspeed = -50
                        rspeed = -25
                    driveDirect(lspeed, rspeed)
                elif abs(command_id) == 111:
                    print '\t%d slight circle right, 50 percent speed'%(command_id)
                    lspeed = 50
                    rspeed = 40
                    if command_id < 0:
                        lspeed = -50
                        rspeed = -40
                    driveDirect(lspeed, rspeed)
                elif abs(command_id) == 112:
                    print '\t%d X rotate X degrees and stop.. Negative for counter-clockwise, positive for clockwise'%(command_id)
                    self.rotate_degrees(int(command_id_list[1]))
                elif abs(command_id) == 113:
                    print '\t%d X Y Drive for X centimeters at Y centimeters per second, then stop.'%(command_id)
                    self.drive_distance(int(command_id_list[1]), int(command_id_list[2]))

                return None
            else:
                for i in range(len(existing_commands)):
                    commands.append(existing_commands[i])

            command_list=command.split(' ')
            if self.enable_robot is False:
                print 'command_list=%s'%str(command_list)
            self.tempfile_handle.write(command + '\n')

            if len(command_list) is 2:
                commands.append([int(command_list[0]), int(command_list[1])])
            elif len(command_list) is 3:
                commands.append([int(command_list[0]), int(command_list[1]), int(command_list[2])])
            elif len(command_list) is 4:
                commands.append([int(command_list[0]), int(command_list[1]), int(command_list[2]), int(command_list[3])])
            else:
                raise Exception('Either two or three arguments required')
        return commands

    def start(self):
        while g_keeprunning is True:
            commandlist = self.get_input()
            if commandlist is not None:
                numrepeats = raw_input('Enter the number of times to repeat the command list (0 == repeat forever): ')
                if len(numrepeats) is 0:
                    numrepeats = 1
                else:
                    numrepeats = int(numrepeats)
                print 'Repeating command list %d time(s)'%numrepeats

                if numrepeats == 0:
                    numrepeats = 3000000

                print 'len(commandlist) = %d'%len(commandlist)
                for numrepeat in range(numrepeats):
                    if g_bumped is True:
                        break
                    for i in range(len(commandlist)):
                        if g_bumped is True:
                            break
                        arglist = commandlist[i][1:]
                        if self.enable_robot is True:   
                            print 'Executing command %d with arguments %s'%(commandlist[i][0], str(arglist))
                            self.function_list[commandlist[i][0]](*arglist)
                        else:
                            print 'enable_robot is False, but command executed would have been %d with arguments %s'%(commandlist[i][0], str(arglist))
        closeRobotCommunication()
        sleep(1)

    def print_welcome_banner(self):
        #change to if False to disable the hello message
        if False:
            welcome_message='HELLO AMELIA'
            for i in range(len(welcome_message)):
                os.system('banner %c'%welcome_message[i])
                time.sleep(.2)



'''
set use_new_awesome_code=True to use the help menu. Set it to False to use the original code
downloaded from github.
'''
use_new_awesome_code=True

def testthread():
    global g_bumped
    sendping_seconds=5.0
    sleeptime=.02
    counter=0.0
    while g_keeprunning is True:
        if counter < sendping_seconds:
            counter += sleeptime
        else:
            #print 'Sending keep-alive ping'
            counter=0
            sendCommandASCII('139 4 0 255')  # Turn on Clean and Dock buttons, a harmless command to keep the robot serial link open, a heartbeat
        
        isbumped = bumped()
        if isbumped is not None:
            if isbumped is True:
                print 'Bumped into something!'
                g_bumped=True
                time.sleep(.11)
                driveDirect(0, 0)
                time.sleep(1)
                break
        time.sleep(sleeptime)
    return            

if use_new_awesome_code:
    #print "Hellooooooooooooo, Amelia!"
    import threading


    print "Starting my Python Robot Controller Program.  This is so cool!"
    #set enablerobot to False if testing this script, robot will not turn on
    awesomeness = do_awesomeness(enablerobot=True, print_dbg_messages=True)
    awesomeness.start()
    g_keeprunning=False
    if awesomeness.enable_robot is True:
        closeRobotCommunication()
    time.sleep(.1)
else:
    #Hello, world!
    print "Hellooooooooooooo, Amelia!"
    print "Starting my Python Robot Controller Program.  This is so cool!"

    #Set any variables here
    #myVariable = 3
    #---------------Start My robot program.----------------------
    #---------------Change the code below.----------------
    driveDirect(5 , 5 )     # Go Straight at 50 cm/sec
    time.sleep( 2 )           # Wait 15  seconds
    driveDirect( 0, 0 )       # Full stop!
    #driveDirectTime( 10, 10, 2 )  # Go straight for two seconds at 10 cm/sec
    driveDirectRot( 0, 100)   # Turns the robot counterclockwise.  Right wheel at 50cm/s.  Left wheel at -50cm/s.  
    time.sleep( 5 )           # Wait 5 seconds
    driveDirectRot( 0, -10)   # Slowly turns the robot clockwise.  Right wheel at -5cm/s.  Left wheel at 5cm/s.
    driveDirect( 0, 0 )       # Full stop!
    time.sleep( 1 )           # Wait one second 
    shudder( 0.05, 45, 5 )    # This shakes the robot back and forth quickly five times
    driveDirect( 0, 0 )       # Full stop! 
    time.sleep( 1 )           # Wait one second
    shudder( 0.05, 45, 3 )    # This shakes the robot back and forth quickly three times
    driveDirect( 0, 0 )       # Full stop!
    #if cleanButtonPressed(): driveDirect( 0, 0 )  # If CLEAN button is pressed stop robot.  
    #if bumped(): driveDirect( -10, -10 )          # If the bumper is hit, go backwards
    #  Note: The communication interface will become corrupt
    #  once the robot goes to sleep after about a minute or so of inactivity.  
    #  One workaround is to use the following command periodically to keep the robot awake.
    #sendCommandASCII('139 4 0 255')  # Turn on Clean and Dock buttons, a harmless command to keep the robot serial link open, a heartbeat

    #Close the robot communication interface.  Do not delete this line.  This is needed to keep the communications protocol happy.
    closeRobotCommunication()

#END OF AMELIA AND KEVIN CHANGES

