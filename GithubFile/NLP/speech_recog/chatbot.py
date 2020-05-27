#import necessary libraries
import io
import random
import string # to process standard python strings
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import nltk
from nltk.stem import WordNetLemmatizer
from nltk.corpus import wordnet
from pocketsphinx import LiveSpeech
import speech_recognition as sr
import time
import os
from nltk.chat.util import Chat
import sys
script_dir = "chat_pairs"
sys.path.append(os.path.abspath(script_dir))
import rosie_pairs as chatbot

def extract_text_from_mv(name_source):
    r = sr.Recognizer()
    from_wav = sr.AudioFile(name_source)
    input_from_speech=''
    with from_wav as source:
        audio = r.record(source)
    try:
        s =  r.recognize_google(audio)
        return s
    except Exception as e:
        print("Exception: "+str(e))

def inputs_controller():
    input_from_speech = []
    input_from_speech.append(extract_text_from_mv('source_data/sample_run_sample.wav'))
    input_from_speech.append("Hi")
    input_from_speech.append("I am doing good thanks")
    input_from_speech.append( "pretty good")
    input_from_speech.append(extract_text_from_mv('source_data/sample_bye.wav'))
    #input_from_speech = " ".join(input_from_speech)
    #print(input_from_speech)
    return input_from_speech

def baxter_commands(user_response): 
    basic_commands_array = ['forward', 'stop', 'slow', 'spin', 'turn', 'execute']
    commands = user_response.split(" ")
    run_cmd = '*rospy cmd*'
    run_flag = False
    for i in commands:
        if run_flag == False:
            if i not in basic_commands_array:
                return False
            if ('forward' == i):
                print('moving forward....')
                os.system("python action_Scripts/move-original.py -x 5")
            if ('stop' == i):
                print('stopping.......')
                os.system("python action_Scripts/stop.py")
            if ('slow' == i):
                print('slowing down.....')
                os.system("python action_Scripts/slow.py -x 5")
            if ('spin' == i):
                print("spinning........")
		os.system("python action_Scripts/rotate.py -y 45")
            if ('turn' == i):
                print("turning.....")
		os.system("python action_Scripts/rotate.py -y 45")
                ##insert rospy command here
            if ('execute' == i):
                run_flag = True
        else:
            if i not in basic_commands_array:
                run_cmd = run_cmd+" "+i
            if i == commands[-1]:
                print("execute the rospy command: python "+run_cmd)
		#hard coded:
	        os.system("python action_Scripts/move-to-door.py -p")
		os.system("python action_Scripts/beerPusher.py")
                os.system("python action_Scripts/waveLikeMade.py")
		os.system("sh ../../../../lift-arms")
                ##insert rospy command here

#implement script controls. e.g: rosie execute script move
def Enable_pocketsphinx():
    print("Rosie: If you want to exit, say Bye!")
    for phrase in LiveSpeech():
        print('you said:')
        print(phrase)
    
        user_response = str(phrase)
        user_response = user_response.lower()
        #Insert Baxter bot commands#
        if baxter_commands(user_response) == False:
            ##Conversational
            if(user_response!='bye'):
                print("you said: "+user_response)
                chatbot.rosie_chat(user_response)
            else:
                print("Rosie: Bye! take care..")
                break
        

def Enable_wav():
    print("************** Rosie **************")
    print("=" * 72)
    print("Hello I am Rosie.  How may i help you today?")
    for i in inputs_controller():
        user_response = str(i)
        user_response = user_response.lower()
	print("you said: "+user_response)
        #Insert Baxter bot commands#
        if baxter_commands(user_response) == False:
            ##Conversational
            if(user_response!='bye'):
                print("you said: "+user_response)
                chatbot.rosie_chat(user_response)
            else:
                print("Rosie: Bye! take care..")  
                break

#Toggle if you want to run with Wav or with pocketsphinx
Enable_wav()
#Enable_pocketsphinx
