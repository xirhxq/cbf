import json
import os


class DataLoader:
    def __init__(self, files):
        self.files = files
        self.folders = [os.path.dirname(file) for file in files]
        self.file = files[0]
        self.folder = self.folders[0]
        self.datas = [json.load(open(file)) for file in files]
        self.data = self.datas[0]
