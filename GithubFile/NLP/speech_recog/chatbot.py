#import necessary libraries
import io
import random
import string # to process standard python strings
import warnings
import numpy as np
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import warnings
warnings.filterwarnings('ignore')

import nltk
from nltk.stem import WordNetLemmatizer
#Usage goal:
#Use the livespeech library

from pocketsphinx import LiveSpeech
#for phrase in LiveSpeech():
#    print(phrase)
#    if str(phrase) ==  "go forward":
#        print("okay")

nltk.download('popular', quiet=True) # for downloading packages
# uncomment the following only the first time
#nltk.download('punkt') # first-time use only
#nltk.download('wordnet') # first-time use only
#Reading in the corpus
with open('chatbot-corpus.txt','r') as fin:
    raw = fin.read().lower()
#TOkenisation
sent_tokens = nltk.sent_tokenize(raw)# converts to list of sentences 
word_tokens = nltk.word_tokenize(raw)# converts to list of words
# Preprocessing
lemmer = WordNetLemmatizer()
def LemTokens(tokens):
    return [lemmer.lemmatize(token) for token in tokens]
remove_punct_dict = dict((ord(punct), None) for punct in string.punctuation)
def LemNormalize(text):
    return LemTokens(nltk.word_tokenize(text.lower().translate(remove_punct_dict)))

# Keyword Matching
#Import a list of responses from a txt file and parse then
GREETING_INPUTS = ("hello", "hi", "greetings", "sup", "what's up","hey",)
GREETING_RESPONSES = ["hi", "hey", "*nods*", "hi there", "hello", "I am glad! You are talking to me"]

def greeting(sentence):
    """If user's input is a greeting, return a greeting response"""
    for word in sentence.split():
        if word.lower() in GREETING_INPUTS:
            return random.choice(GREETING_RESPONSES)


# Generating response
def response(user_response):
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
print("Rosie: If you want to exit, say Bye!")
for phrase in LiveSpeech():
    print('you said:')
    print(phrase)
#    if str(phrase) ==  "go forward":
#        print("okay")
    
    user_response = str(phrase)
    user_response = user_response.lower()
    #Insert Baxter bot commands#
    if (user_response == 'go forward'):
        print('moving forward....')
    elif(user_response!='bye'):
        if(user_response=='thanks' or user_response=='thank you' ):
            flag=False
            print("Rosie: You are welcome..")
        else:
            if(greeting(user_response)!=None):
                print("Rosie: "+greeting(user_response))
            else:
                print("Rosie:\n")
                print(response(user_response))
                sent_tokens.remove(user_response)
    else:
        print("Rosie: Bye! take care..")  
        break
