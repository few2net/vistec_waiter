#!/usr/bin/env python3
import json
import os
import numpy as np

from shutil import copyfile
from flask import Flask, request, render_template, Response

from data_manager import DataManager

app = Flask(__name__)
dm = DataManager()

@app.route("/")
def main():
    return render_template('index.html')


@app.route("/training", methods=['GET', 'POST'])
def training():
    return render_template('training.html')


@app.route("/requestPhoto")
def requestPhoto(size=4):
    if(not dm.classifier.is_trained):
        print("Clustering image for selecting target")
        images = dm.train_clustering()
        for img in images:
            print(img)
            copyfile(dm.src_img_path + img, dm.web_img_path + img)
        
        return json.dumps(images)

    else:
        print("Chooses images with high entropy for labelling")
        img_list = np.load(dm.dataset_path + "svm_rank.npy", allow_pickle=True)
        chosenImg = []

        for img in img_list[:size]:
            print(img[0])
            copyfile(dm.src_img_path + img[0], dm.web_img_path + img[0])
            chosenImg.append(img[0])

        return json.dumps(chosenImg)


@app.route("/saveLabel")
def saveLabel():
    data = request.args.get('data', '')
    parsedData = json.loads(data)

    # load the existing file
    try:
        data = np.load(dm.dataset_path + 'data.npy', allow_pickle=True)
        data = data.item()

    except:
        data = {}
        data['x'] = []
        data['y'] = []

    for i in parsedData:
        test = np.load(dm.src_np_path + 'fe_'+ str(i['x']) + '.npy',allow_pickle=True)
        x = test.item()['data']
        data['x'] += [x]
        data['y'] += [i['y']]

    np.save(dm.dataset_path + 'data.npy', data)
    dm.train_svm()
    dm.sort_data()

    return "Training SVM successfully"


@app.route("/resetSVM")
def resetSVM():
    dm.reset()
    return 'Reset svm successfully'


@app.route("/clearMemory")
def clearMemory():
    dm.clear_memory()
    return 'cleared raw, features, data.npy, and svm_rank.npy files'


@app.route("/get_svm")
def get_svm():
    return dm.classifier.is_trained


#########################################
if __name__ == '__main__':
    # start the flask app
    app.run(host='0.0.0.0', debug=True,
        threaded=True, use_reloader=False)

