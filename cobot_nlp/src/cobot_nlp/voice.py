from sound_play.libsoundplay import SoundClient

class Voice:

    def __init__(self):
        self.soundhandle = SoundClient()

    def speak(msg):
        s3 = soundhandle.voiceSound(msg)
        s3.play()
