import speech_recognition as sr
r = sr.Recognizer()

hellow = sr.AudioFile('source_data/sample_run_sample.wav')
with hellow as source:
    audio = r.record(source)
try:
    s = r.recognize_google(audio)
    print("Text: "+s)
except Exception as e:
    print("Exception: "+str(e))
