# import rospy
# from std_msgs.msg import String

# def listen_for_command():
#     rospy.init_node('voice_command_listener')
#     pub = rospy.Publisher('voice_commands', String, queue_size=10)
#     rate = rospy.Rate(10)  
#     while not rospy.is_shutdown():
#         command = "jack"  # Replace with actual voice command logic
#         rospy.loginfo(f"Publishing command: {command}")
#         pub.publish(command)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         listen_for_command()
#     except rospy.ROSInterruptException:
#         pass
    
    
# # import speech_recognition as sr

# # names = ["Giovanni", "barbaros", "rajeck", "joren", "yağmur"]
# # recognizer = sr.Recognizer()

# # def listen_for_command():
# #     with sr.Microphone() as source:
# #         print("Listening for a command...")
# #         audio = recognizer.listen(source)
        
# #         try:
# #             # Recognize the command using Google's Web Speech API
# #             command = recognizer.recognize_google(audio)
# #             print(f"Command received: {command}")
# #             return command.lower()
# #         except sr.UnknownValueError:
# #             print("Sorry, I did not understand the command.")
# #         except sr.RequestError:
# #             print("Could not request results from Google Speech Recognition service.")
# #     return None

# # def main():
# #     while True:
# #         command = listen_for_command()
# #         if command:
# #             if "giovanni" in command:
# #                 print("Going to Giovanni.")
# #                 # code to move the robot to Gio's location here
# #                 break
# #             elif "rajeck" in command:
# #                 print("Going to Rajeck.")
# #                 # code to move the robot to Rajeck's location here 
# #                 break    
# #             elif "joren" in command:
# #                 print("Going to Joren.")
# #                 # code to move the robot to Joren's location here
# #                 break
# #             elif "barbaros" in command:
# #                 print("Going to Barbaros.")
# #                 # code to move the robot to Barbaros' location here
# #                 break
# #             elif "yağmur" in command:
# #                 print("Going to Yağmur.")
# #                 # code to move the robot to Yağmur's location here   
# #                 break       
# #             elif "stop" in command:
# #                 print("Stopping the robot.")
# #                 # code to stop the robot here
# #                 break
# #             else:
# #                 print("Command not recognized. Please try again.")

# # if __name__ == "__main__":
# #     main()