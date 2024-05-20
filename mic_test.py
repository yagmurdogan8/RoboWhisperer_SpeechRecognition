import speech_recognition as sr

recognizer = sr.Recognizer()

def test_microphone():
    with sr.Microphone() as source:
        print("Adjusting, please wait...")
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
