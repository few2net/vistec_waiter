from sklearn.svm import SVC
from sklearn.model_selection import GridSearchCV

class SVM(object):
    def __init__(self):

        self.model = SVC(probability=True, kernel='linear')
        self.is_trained = False

    def train(self, X_train, y_train, tuning=False):
        
        self.model.fit(X_train, y_train)
        self.is_trained = True

    def get_prediction(self, X_test):
        if self.is_trained:
            return self.model.predict_proba(X_test)
        else: 
            return [[0,0]]

    def reset(self):
        self.is_trained=False
