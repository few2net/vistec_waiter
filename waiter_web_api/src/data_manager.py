#!/usr/bin/env python3

import os
import numpy as np

from sklearn.mixture import GaussianMixture
from ranking_SVM import SVM


class DataManager:
    def __init__(self, n_size=4):
        root_path = os.path.dirname(os.path.realpath(__file__))
        self.src_img_path = os.path.join(root_path, "image-capture/")
        self.src_np_path = os.path.join(root_path,"image-features/")
        self.web_img_path = os.path.join(root_path,"static/assets/image-capture/")
        self.dataset_path = os.path.join(root_path, "data/")

        self.n_size = n_size
        self.reset()


    # Clear data.npy file in data folder
    def reset(self):
        print("clearing trained data...")
        try:
            os.remove(self.dataset_path + 'data.npy')
            print("Remove file /data/data.npy")
            os.remove(self.dataset_path + "svm_rank.npy")
            print("Remove file /data/svm_rank.npy")
        except:
            pass

        self.classifier = SVM()
        self.cluster = GaussianMixture(n_components=self.n_size)
        return True


    # Clear all files
    def clear_memory(self):
        img_list = os.listdir(self.src_img_path)
        for img in img_list:
            # print(img)
            if os.path.exists(self.src_img_path + img):
                os.remove(self.src_img_path + img)
                print("Remove file /image-capture/" + img)
            else:
                print("/image-capture/" + img + " does not exist")

        # Clear files in copied image-capture folder in static/assets/
        img_list_assets = os.listdir(self.web_img_path)
        for img in img_list_assets:
            if os.path.exists(self.web_img_path):
                os.remove(self.web_img_path + img)
                print("Remove file static/assets/image-capture/" + img)
            else:
                print("assets/image-capture/" + img + " does not exist")

        # Clear files in image-features folder
        fe_list = os.listdir(self.src_np_path)
        for fe in fe_list:
            if os.path.exists(self.src_np_path + fe):
                os.remove(self.src_np_path + fe)
                print("Remove file /image-features/" + fe)
            else:
                print("/image-features/" + fe + " does not exist")

        self.reset()
        return True


    # Train SVM of all features data
    def train_svm(self):
        print("Trains model")
        try:
            train_data = np.load(self.dataset_path + "data.npy", allow_pickle=True)
            train_data = train_data.item()
            X = np.array(train_data['x'])
            Y = train_data['y']
            self.classifier.train(X, Y)
            return [True, '']

        except:
            return [False, '']


    # Clustering data to N group and random for each
    def train_clustering(self):
        print("Trains clustering")
        try:
            fe_files = os.listdir(self.src_np_path)
            print ("Read %s images..."%len(fe_files))
            name = []
            X = []
            for f in fe_files:
                name.append("img_" + f[3:-4:1] + ".png")

                data = np.load(self.src_np_path + f, allow_pickle=True)
                x = np.array(data.item()['data'])
                X.append(x)

            labels = self.cluster.fit_predict(X)
            
            sep_class = [[]]*self.n_size
            for i in range(len(labels)):
                sep_class[labels[i]].append(name[i])

            rand_list = []
            for n in range(self.n_size):
                rand_list.append(str(np.random.choice(sep_class[n])))

            return rand_list

        except:
            return [False, '']


    # Sort data with high uncentainty of labels [0,1]
    def sort_data(self):
        print("Ranks data")
        fe_list = os.listdir(self.src_np_path)
        print ("Read %s images..."%len(fe_list))
        name = []
        all_fe = []

        for fe in fe_list:
            name.append("img_" + fe[3:-4:1] + ".png")
            data = np.load(self.src_np_path + fe, allow_pickle=True)
            x = np.array(data.item()['data'])
            all_fe.append(x)

        labels, entropy = self.classifier.get_entropy(all_fe)
        y0 = []
        y1 = []

        for i in range(len(labels)):
            if labels[i]==0:
                y0.append([name[i], entropy[i], labels[i]])
            else:
                y1.append([name[i], entropy[i], labels[i]])

        y0 = np.array(y0)
        y1 = np.array(y1)
        sorted_y0 = y0[y0[:,1].argsort()[::-1]]
        sorted_y1 = y1[y1[:,1].argsort()[::-1]]

        num = int(self.n_size/2) + (self.n_size % 2 > 0)
        sortedRank = np.concatenate((sorted_y0[:num],sorted_y1[:num]))
        np.save(self.dataset_path + "svm_rank.npy", sortedRank)

