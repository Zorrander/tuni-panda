#!/usr/bin/env python3

from opendr.perception.speech_recognition.matchboxnet.matchboxnet_learner import MatchboxNetLearner

def download():
    learner = MatchboxNetLearner(device="cpu")
    learner.download_pretrained()

if __name__ == '__main__':
    download()
