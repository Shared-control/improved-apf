#coding=utf-8

from pykeyboard import PyKeyboardEvent
from threading import Thread

import rospy
from keyboard.msg import KeyCommand
from geometry_msgs.msg import Twist

class Keyboard(PyKeyboardEvent):
    def __init__(self):
        PyKeyboardEvent.__init__(self)

        self.pub_command = rospy.Publisher("key_command", KeyCommand, queue_size=1)
        self.twist_msg = Twist()
        self.command_msg = KeyCommand()

        self.thread = Thread(target=self.run)
        self.thread.start()

    def stop(self):
        super(Keyboard, self).stop()
        self.thread.join()

    def tap(self, keycode, character, press):
        self.command_msg.header.stamp = rospy.Time.now()

        #Create zero twist message
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        key_char = str(character)
        key_pressed = press

        #COMMAND
        # "Up"  = PLACE
        # "Down" = PICK
        # "BackSpace" = HOME
        # " " (Space) = FINISH
        # "Alphabet" = TWIST
        
        #Manage command message
        if(key_char == "Up"):
            self.command_msg.command = self.command_msg.PLACE

        elif(key_char == "Down"):
            self.command_msg.command = self.command_msg.PICK

        elif(key_char == "BackSpace"):
            self.command_msg.command = self.command_msg.HOME

        elif(key_char == " "):
            self.command_msg.command = self.command_msg.FINISH

        else:
            self.command_msg.command = self.command_msg.TWIST

            if((key_char == "q") and (key_pressed)):
                self.twist_msg.linear.z = 0.2

            elif((key_char == "w") and (key_pressed)):
                self.twist_msg.linear.z = -0.2

            elif((key_char == "s") and (key_pressed)):
                self.twist_msg.linear.y = 0.2

            elif((key_char == "a") and (key_pressed)):
                self.twist_msg.linear.y = -0.2

            elif((key_char == "z") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
            
            elif((key_char == "x") and (key_pressed)):
                self.twist_msg.linear.x = -0.2

            elif((key_char == "e") and (key_pressed)):
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.z = 0.2

            elif((key_char == "c") and (key_pressed)):
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.x = 0.2

            elif((key_char == "v") and (key_pressed)):
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.x = -0.2

            elif((key_char == "r") and (key_pressed)):
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "b") and (key_pressed)):
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.x = 0.2

            elif((key_char == "t") and (key_pressed)):
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.z = 0.2

            elif((key_char == "y") and (key_pressed)):
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "n") and (key_pressed)):
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.x = -0.2

            elif((key_char == "d") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
                self.twist_msg.linear.z = 0.2

            elif((key_char == "f") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "g") and (key_pressed)):
                self.twist_msg.linear.x = -0.2
                self.twist_msg.linear.z = 0.2
            
            elif((key_char == "h") and (key_pressed)):
                self.twist_msg.linear.x = -0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "u") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.z = 0.2
            
            elif((key_char == "i") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "o") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.z = 0.2

            elif((key_char == "p") and (key_pressed)):
                self.twist_msg.linear.x = 0.2
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "j") and (key_pressed)):
                self.twist_msg.linear.x = -0.2
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.z = 0.2

            elif((key_char == "k") and (key_pressed)):
                self.twist_msg.linear.x = -0.2
                self.twist_msg.linear.y = 0.2
                self.twist_msg.linear.z = -0.2

            elif((key_char == "l") and (key_pressed)):
                self.twist_msg.linear.x = -0.2
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.z = 0.2

            elif((key_char == "m") and (key_pressed)):
                self.twist_msg.linear.x = -0.2
                self.twist_msg.linear.y = -0.2
                self.twist_msg.linear.z = -0.2

            #Set twist command message
            self.command_msg.twist = self.twist_msg
        #END else

        #Publish command message
        self.pub_command.publish(self.command_msg)

