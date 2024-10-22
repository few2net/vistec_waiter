from sklearn.svm import SVC
from sklearn.model_selection import GridSearchCV
import math
import numpy as np

class SVM(object):
    def __init__(self):

        self.model = SVC(probability=True, kernel='linear')
        self.is_trained = False

    def train(self, X_train, y_train, tuning=False):
        
        self.model.fit(X_train, y_train)
        self.is_trained = True

    def get_prediction(self, X_test):
        if self.is_trained:
            return self.model.predict(X_test)
        else: 
            return 0

    def get_entropy(self, X_test):
        # (n_sample, n_features)
        labels = self.model.predict(X_test)
        prob = self.model.predict_proba(X_test)
        log_prob = self.model.predict_log_proba(X_test)

        entropy = []
        for i in range(len(labels)):
            entropy.append(-prob[i][labels[i]] * log_prob[i][labels[i]])

        return (labels, entropy)

    def reset(self):
        self.is_trained=False
