import speech_recognition as sr

names = ["Giovanni", "Barbaros", "Rajeck", "Joren", "Yağmur"]

def listen_for_name():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for name ...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        command = recognizer.recognize_google(audio)
        print("You said:", command)
        return command.lower() 
    except sr.UnknownValueError:
        print("Sorry, I could not understand what you said.")
        return None
    except sr.RequestError as e:
        print("Could not request results from Google Web Speech Recognition service; {0}".format(e))
        return None

def navigate_to_person(name):
    if name in names:
        print("Navigating to", name)
    else:
        print("Name not recognized!")

if __name__ == "__main__":
    while True:
        name_command = listen_for_name()
        if name_command:
            for name in names:
                if name in name_command:
                    navigate_to_person(name)
