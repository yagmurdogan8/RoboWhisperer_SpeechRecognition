import speech_recognition as sr
import pyaudio

names = ["Giovanni", "Barbaros", "Rajeck", "Joren", "Yağmur"]
recognizer = sr.Recognizer()

def test_microphone():
    with sr.Microphone() as source:
        print("Adjusting for ambient noise, please wait...")
        recognizer.adjust_for_ambient_noise(source, duration=5)
        print("Say something!")
        audio = recognizer.listen(source)
        
        try:
            # Recognize the audio using Google Web Speech API
            print("Google Speech Recognition thinks you said: " + recognizer.recognize_google(audio))
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand the audio")
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")

test_microphone()


def listen_for_command():
    with sr.Microphone() as source:
        print("Listening for a command...")
        audio = recognizer.listen(source)
        
        try:
            # Recognize the command using Google's Web Speech API
            command = recognizer.recognize_google(audio)
            print(f"Command received: {command}")
            return command.lower()
        except sr.UnknownValueError:
            print("Sorry, I did not understand the command.")
        except sr.RequestError:
            print("Could not request results from Google Speech Recognition service.")
    return None

def main():
    while True:
        command = listen_for_command()
        if command:
            if "giovanni" in command:
                print("Going to Giovanni.")
                # code to move the robot to Gio's location here
            elif "rajeck" in command:
                print("Going to Rajeck.")
                # code to move the robot to Rajeck's location here     
            elif "joren" in command:
                print("Going to Joren.")
                # code to move the robot to Joren's location here
            elif "barbaros" in command:
                print("Going to Barbaros.")
                # code to move the robot to Barbaros' location here
            elif "yağmur" in command:
                print("Going to Yağmur.")
                # code to move the robot to Yağmur's location here          
            elif "stop" in command:
                print("Stopping the robot.")
                # code to stop the robot here
            else:
                print("Command not recognized. Please try again.")

if __name__ == "__main__":
    main()