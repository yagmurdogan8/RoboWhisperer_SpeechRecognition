import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer

model_paths = {
    "it-IT": r"C:\Users\1\Documents\GitHub\RoboWhisperer_SpeechRecognition\language_models\italian\vosk-model-small-it-0.22",
    "tr-TR": r"C:\Users\1\Documents\GitHub\RoboWhisperer_SpeechRecognition\language_models\turkish\vosk-model-small-tr-0.3",
    "nl-NL": r"C:\Users\1\Documents\GitHub\RoboWhisperer_SpeechRecognition\language_models\dutch\vosk-model-small-nl-0.22"
}


names = ["giovanni", "barbaros", "rajeck", "joren", "yağmur"]

language_codes = {
    "giovanni": "it-IT",
    "barbaros": "tr-TR",
    "rajeck": "nl-NL",
    "joren": "nl-NL",
    "yağmur": "tr-TR"
}

def load_model(language_code):
    model_path = model_paths.get(language_code)
    if not model_path or not os.path.exists(model_path):
        raise FileNotFoundError(f"Model for language {language_code} not found at {model_path}.")
    return Model(model_path)

def recognize_speech(model):
    recognizer = KaldiRecognizer(model, 16000)
    recognizer.SetWords(True)
    
    p = pyaudio.PyAudio()
    
    try:
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4096)
        stream.start_stream()

        print("Listening for a command...")

        while True:
            data = stream.read(4096, exception_on_overflow=False)
            if len(data) == 0:
                break
            if recognizer.AcceptWaveform(data):
                result = recognizer.Result()
                result_dict = json.loads(result)
                command = result_dict.get("text", "").lower()
                if command:
                    print(f"Name of the person: {command}")
                    return command
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()
        
    return None

def process_command(command):
    for name in names:
        if name.lower() in command:
            print(f"Going to {name.capitalize()}...")
            # code to move robot to that person
            return True
    if "stop" in command:
        print("Stopping the robot...")
        # code to stop the robot
        return True
    return False

def main():
    vocabulary = ["giovanni", "barbaros", "rajeck", "joren", "yağmur", "stop"]
    while True:
        for lc in set(language_codes.values()):
            try:
                model = load_model(lc)
                command = recognize_speech(model)
                if command and process_command(command):
                    return
            except Exception as e:
                print(f"Error while processing language {lc}: {e}")
        print("Listening for another command...")

if __name__ == "__main__":
    main()



# import speech_recognition as sr

# names = ["Giovanni", "barbaros", "rajeck", "joren", "yağmur"]
# recognizer = sr.Recognizer()

# def listen_for_command():
#     with sr.Microphone() as source:
#         print("Listening for a command...")
#         audio = recognizer.listen(source)
        
#         try:
#             # Recognize the command using Google's Web Speech API
#             command = recognizer.recognize_google(audio)
#             print(f"Command received: {command}")
#             return command.lower()
#         except sr.UnknownValueError:
#             print("Sorry, I did not understand the command.")
#         except sr.RequestError:
#             print("Could not request results from Google Speech Recognition service.")
#     return None

# def main():
#     while True:
#         command = listen_for_command()
#         if command:
#             if "giovanni" in command:
#                 print("Going to Giovanni.")
#                 # code to move the robot to Gio's location here
#                 break
#             elif "rajeck" in command:
#                 print("Going to Rajeck.")
#                 # code to move the robot to Rajeck's location here 
#                 break    
#             elif "joren" in command:
#                 print("Going to Joren.")
#                 # code to move the robot to Joren's location here
#                 break
#             elif "barbaros" in command:
#                 print("Going to Barbaros.")
#                 # code to move the robot to Barbaros' location here
#                 break
#             elif "yağmur" in command:
#                 print("Going to Yağmur.")
#                 # code to move the robot to Yağmur's location here   
#                 break       
#             elif "stop" in command:
#                 print("Stopping the robot.")
#                 # code to stop the robot here
#                 break
#             else:
#                 print("Command not recognized. Please try again.")

# if __name__ == "__main__":
#     main()