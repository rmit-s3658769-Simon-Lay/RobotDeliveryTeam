#Live speech sample from:
#https://pypi.org/project/pocketsphinx/
from pocketsphinx import LiveSpeech
for phrase in LiveSpeech(): print(phrase)
