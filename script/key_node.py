#!/usr/bin/env python
import rospy
from rosi_defy.msg import KeyArray
from pynput.keyboard import Listener, Key

#[a,b,c,d]
# a/b => Rodas
# c/d => Esteiras
command_array = [0.0,0.0,0.0,0.0]

class KeyboardPublisher:

    

    def __init__(self):
        # logging the node start
        rospy.loginfo("Starting node key_node")

        # Initialise the node
        rospy.init_node("key_node", anonymous=True)

        # Create a publisher to the keypress topic
        self.key_publisher = rospy.Publisher('/keyboard', KeyArray, queue_size=1)

    def publish_keypress(self, key_press):

        if key_press == Key.space:
            command_array[0] *= 0
            command_array[1] *= 0
            command_array[2] *= 0
            command_array[3] *= 0
        
        #RODAS
        if key_press == Key.up:
            if command_array[0] < 10:
                command_array[0] += 1
        if key_press == Key.down:
            if command_array[0] > -10:
                command_array[0] -= 1
        if key_press == Key.left:
            if command_array[1] < 10:
                command_array[1] += 1
        if key_press == Key.right:
            if command_array[1] > -10:
                command_array[1] -= 1
        

        #Esteiras frontais
        if key_press == Key.page_up:
            if command_array[2] == 1:
                command_array[2] = 0
            else:
                command_array[2] = 1
        if key_press == Key.page_down:
            if command_array[2] == -1:
                command_array[2] = 0
            else:
                command_array[2] = -1

        #Esteiras trasieras
        if key_press == Key.home:
            if command_array[3] == 1:
                command_array[3] = 0
            else:
                command_array[3] = 1
        if key_press == Key.end:
            if command_array[3] == -1:
                command_array[3] = 0
            else:
                command_array[3] = -1


        self.key_publisher.publish(command_array)

    def on_press(self, key):
        print(key) #<-- this prints out the key press and then crashes
        self.publish_keypress(key)

    def on_release(self, key):
        print('{0} release'.format(
            key))
        if key == Key.esc:
            # Stop listener
            return False

    def keyboard_listener(self):
        # Collect events until released
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()


if __name__ == '__main__':
    key_publisher = KeyboardPublisher()
    key_publisher.keyboard_listener()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Stopping key_node")
