import time

class DataModel:

    def __init__(self):
        self.data = None

    def load_data(self):

        print("DataModel: Loading data (simulated) ...")
        time.sleep(2)  
        self.data = "Loaded Data!"
        print("DataModel: Data loaded successfully.")
        return self.data