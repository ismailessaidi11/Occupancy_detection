# Training Neural Networks to predict room occupancy

## Overview
This project explores different datasets, methodologies, and training techniques to find the dataset requirements (features and data diversity) to train a 2-layered neural network (NN) aiming to be deployed on an embedded system in order to predict room occupancy.

## Structure
1. **datasets**: Different datasets were used to find dataset requirements before training the best deployed model.
2. **notebooks**: These notebooks were used to find the dataset requirements, apply changes to current datasets (data augmentation and feature adding), select the best model and train it in order to be deployed.
3. **deploy**: Trained models ready to be deployed on embedded systems.

## Datasets
1. **HVAC and room_occupancy**: These two datasets were utilized to find the dataset requirements needed for better predictions.
2. **tera dataset**: This dataset was collected during our project in order to have the best predictions in our specific environnment. This dataset was collected based on the dataset requirements learned from the previous study of the two datasets.

## Notebooks
1. **dataset_requirements**: We compared the importance and relevance of different features. We also studied the importance of a balanced target ditribution (balanced datasets vs unbalanced (original) datasets). We then studied the impact of having the same sample rate in our time series (same sample rate vs different sample rate). This notebook aimed to give us insight about the requirements that our own dataset needed to fulfill.
2. **data_augmentation**: Due to the lack of data, we proceded a data augmentation to the tera dataset. This process was studied in order to add noisy data that still represents real-life data. More info about the data augmentation technique are in the extract_noisy_df and augment_data_with_noise functions.
3. **NN_hyperparameter_tuning**: We proceeded to find the best hyperparameters by applying a grid search.
4. **train_model**: Training, evaluating, and comparing different trained models with different datasets and feature sets. This notebook also contains various optimization techniques.Finally, it contains a deployment function that creates a zip file that contains the trained model weight in .h5 format along with scaling constants for all the features.

## Deploy
Hosts different trained NN with different combinations of datasets and feature sets.

## Key Features
- **Metrics**: Models are trained using validation loss metric and evaluated using metrics such as accuracy, F1 score, confusion matrix, and training time.
- **Optimizations**: - During model training trocess: implementation of EarlyStopping and ModelCheckpoint to save the model that minimizes validation loss.
                     - During decision boundary selection: Cross-validation to find the threshold that maximizes f1 score.
                     - During hyperparameter tuning: Grid search to find the hyperparameters that maximize validation accuracy.
- **Models**: Implementation of a 2-layered Artificial Neural Networks (ANN) and a Long Short-Term Memory (LSTM) model.
- **Feature Engineering**: Exploration of different feature sets: basic features, regular features, and acceleration features (more details in Notations section of train_model notebook).

## Conclusion
This project provides a comprehensive framework for predicting room occupancy using neural networks. It covers data preparation and augmentation, feature engineering, model training and evaluation that can be deployed on an embedded system making it a valuable resource for similar predictive modeling tasks.
