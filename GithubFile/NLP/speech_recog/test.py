#Usage goal:
#Use the livespeech library

from pocketsphinx import LiveSpeech
for phrase in LiveSpeech():
    print(phrase)
    if str(phrase) ==  "go forward":
        print("okay") 
