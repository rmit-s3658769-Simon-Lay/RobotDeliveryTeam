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
import os
import sys
script_dir = "chat_pairs"
sys.path.append(os.path.abspath(script_dir))
import rosie_pairs as chatbot

#changes required
chatbot.rosie_chat()



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
    input_from_speech.append(extract_text_from_mv('source_data/sample_forward.wav'))
    input_from_speech.append(extract_text_from_mv('source_data/sample_stop.wav'))
    input_from_speech.append(extract_text_from_mv('source_data/sample_bye.wav'))
    #input_from_speech = " ".join(input_from_speech)
    #print(input_from_speech)
    return input_from_speech

nltk.download('popular', quiet=True) # for downloading packages
# uncomment the following only the first time
#nltk.download('punkt') # first-time use only
#nltk.download('wordnet') # first-time use only
#Reading in the corpus
with open('chatbot-corpus.txt','r') as fin:
    raw = fin.read().lower()

 #Tokenisation
sent_tokens = nltk.sent_tokenize(raw)# converts to list of sentences 
word_tokens = nltk.word_tokenize(raw)# converts to list of words

# Preprocessing
lemmer = WordNetLemmatizer()
def LemTokens(tokens):
    return [lemmer.lemmatize(token) for token in tokens]
remove_punct_dict = dict((ord(punct), None) for punct in string.punctuation)
def LemNormalize(text):
    return LemTokens(nltk.word_tokenize(text.lower().translate(remove_punct_dict)))

def greeting(sentence):
    """If user's input is a greeting, return a greeting response"""
    return_array = []
    for synonym in wordnet.synsets(sentence):
        for lemma in synonym.lemmas():
            return_array.append(lemma.name())
    #return a random string
    return random.choice(return_array)

def baxter_commands(user_response): 
    basic_commands_array = ['forward', 'stop', 'slow', 'spin', 'turn', 'execute']
    commands = user_response.split(" ")
    run_cmd = '*rospy cmd*'
    run_flag = False
    print(commands)
    for i in commands:
        if run_flag == False:
            if i not in basic_commands_array:
                return False
            if ('forward' == i):
                print('moving forward....')
                os.system("python move-original.py -x 5")
            if ('stop' == i):
                print('stopping.......')
                os.system("python stop.py")
            if ('slow' == i):
                print('slowing down.....')
                ##insert rospy command here
            if ('spin' == i):
                print("spinning........")
                ##insert rospy command here
            if ('turn' == i):
                print("turning.....")
                ##insert rospy command here
            if ('execute' == i):
                run_flag = True
        else:
            if i not in basic_commands_array:
                run_cmd = run_cmd+" "+i
            if i == commands[-1]:
                print("execute the rospy command: python "+run_cmd)
		#hard coded:
	        os.system("python move-to-door.py -p")
		os.system("python beerPusher.py")
                ##insert rospy command here

# Generating response
def Use_Corpus(user_response):
    robo_response=''
    sent_tokens.append(user_response)
    TfidfVec = TfidfVectorizer(tokenizer=LemNormalize, stop_words='english')
    tfidf = TfidfVec.fit_transform(sent_tokens)
    vals = cosine_similarity(tfidf[-1], tfidf)
    idx=vals.argsort()[0][-2]
    flat = vals.flatten()
    flat.sort()
    req_tfidf = flat[-2]
    if(req_tfidf==0):
        robo_response=robo_response+"I am sorry! I don't understand you"
        return robo_response
    else:
        robo_response = robo_response+sent_tokens[idx]
        return robo_response

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
                analyse = wordnet.synsets(user_response)
                if isinstance(analyse,list) == True and not analyse == []:
                    definition = analyse[0].definition()
                    print(definition)
                    if ('greeting' in definition):
                        print("Rosie: "+greeting(user_response))
                else:
                    print("Rosie:")
                    print(Use_Corpus(user_response))
                    sent_tokens.remove(user_response)
            else:
                print("Rosie: Bye! take care..")  
                break

def Enable_wav():
    print("Rosie: If you want to exit, say Bye!")
    for i in inputs_controller():
	#to simulate real world
	time.sleep(15)
	#above is to simulate real world

        user_response = str(i)
        user_response = user_response.lower()
        print("you said: "+user_response)
        #Insert Baxter bot commands#
        if baxter_commands(user_response) == False:
            ##Conversational
            if(user_response!='bye'):
                analyse = wordnet.synsets(user_response)
                if isinstance(analyse,list) == True and not analyse == []:
                    definition = analyse[0].definition()
                    print(definition)
                    if ('greeting' in definition):
                        print("Rosie: "+greeting(user_response))
                else:
                    print("Rosie:")
                    print(Use_Corpus(user_response))
                    sent_tokens.remove(user_response)
            else:
                print("Rosie: Bye! take care..")  
                break
Enable_wav()

