import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer
from pathlib import Path
src = Path('C:/Users/Playground/filename.xlsx')

# Paths to the models for each language
model_paths = {
    "it-IT": Path('C:\Users\1\Documents\GitHub\RoboWhisperer_SpeechRecognition\language_models\vosk-model-small-it-0.22\vosk-model-small-it-0.22'),
    "tr-TR": Path('C:\Users\1\Documents\GitHub\RoboWhisperer_SpeechRecognition\language_models\vosk-model-small-tr-0.3'),
    "nl-NL": Path('C:\Users\1\Documents\GitHub\RoboWhisperer_SpeechRecognition\language_models\vosk-model-small-nl-0.22')
}

# List of names in different languages
names = ["giovanni", "barbaros", "rajeck", "joren", "yağmur"]

# Dictionary to map names to language codes
language_codes = {
    "giovanni": "it-IT",
    "barbaros": "tr-TR",
    "rajeck": "nl-NL",
    "joren": "nl-NL",
    "yağmur": "tr-TR"
}

def check_model_files(model_path):
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model path {model_path} does not exist.")
    
    # Print the contents of the model directory
    print(f"Contents of model directory {model_path}:")
    for root, dirs, files in os.walk(model_path):
        for file in files:
            print(os.path.join(root, file))
    
    # Check for expected files
    required_files = ['am', 'graph', 'ivector']
    for req_file in required_files:
        if not any(req_file in file for file in files):
            raise FileNotFoundError(f"Required file '{req_file}' not found in the model directory {model_path}.")

def load_model(language_code):
    model_path = model_paths.get(language_code)
    print(f"Loading model for language {language_code} from path: {model_path}")  # Debugging statement
    if not model_path or not os.path.exists(model_path):
        raise FileNotFoundError(f"Model for language {language_code} not found.")
    
    # Check model files
    check_model_files(model_path)
    
    return Model(model_path)

def recognize_speech(model):
    recognizer = KaldiRecognizer(model, 16000)
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
                print(f"Command received: {command}")
                return command
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()
        
    return None

def process_command(command):
    for name in names:
        if name.lower() in command:
            print(f"Going to {name.capitalize()}.")
            # Insert code to move the robot to the respective location here
            return True
    if "stop" in command:
        print("Stopping the robot.")
        # Insert code to stop the robot here
        return True
    return False

def main():
    # Check all models at the start
    for lc in set(language_codes.values()):
        try:
            check_model_files(model_paths[lc])
        except Exception as e:
            print(f"An error occurred while checking model files for language {lc}: {e}")
            return
    
    while True:
        for lc in set(language_codes.values()):
            try:
                model = load_model(lc)
                command = recognize_speech(model)
                if command:
                    if process_command(command):
                        return
            except Exception as e:
                print(f"An error occurred while processing language {lc}: {e}")
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