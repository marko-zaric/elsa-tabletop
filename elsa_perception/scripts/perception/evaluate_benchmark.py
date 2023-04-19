import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def select_value(row):
    if row.iloc[2] == 1:
        return row.iloc[1]
    elif row.iloc[2] == 0:
        return row.iloc[0]
    else:
        return None

def mod_pi_half(row):
    return row % (np.pi/2)
    
def add_180_deg(row):
    if row.iloc[2] == True:
        return 0.0
    if row.iloc[1] == 1:
        return (row.iloc[0] + np.pi/2) % (2*np.pi) 
    else:
        return row.iloc[0]

class BenchmarkEvaluation():
    def __init__(self, file):
        self.csv_file = open(file, 'r')

    def evaluate(self):
        df = pd.read_csv(self.csv_file, sep=',', header=0)
        

        # Percived as round
        df['round_correct'] = np.where(df['round'] == df['gt_round'], True, False)
        print("Round detected correctly: ")
        print(df['round_correct'].value_counts())

        # Position Error
        x_error = df['gt_x'] - df['x']
        y_error = df['gt_y'] - df['y']

        plt.hist(x_error*100)
        plt.title('Absolute position error in x axis')
        plt.xlabel('Error in cm')
        plt.show()

        plt.hist(y_error*100)
        plt.title('Absolute position error in y axis')
        plt.xlabel('Error in cm')
        plt.show()

        # Bounding Box error
        dz_error = df['gt_dz'] - df['dz'] # skewed dz error indicates bad camera hight selection
        

        ## account for orientation error of 180deg
        dy_error_1 = df['gt_dy'] - df['dy']
        dy_error_2 = df['gt_dy'] - df['dx']
        dy_error = pd.concat([dy_error_1,dy_error_2],axis=1).min(axis=1)
        
        dy_error_argmin = pd.concat([dy_error_1,dy_error_2],axis=1).idxmin(axis='columns')
        dx_error = df['gt_dx'] - pd.concat([df['dx'], df['dy'], dy_error_argmin],axis=1).apply(select_value, axis=1)

        dphi_error = df['gt_phi'].apply(mod_pi_half) - df['phi'].apply(mod_pi_half)
        

        plt.hist(dphi_error)
        plt.title('Absolute orientation error')
        plt.xlabel('Error in rad')
        plt.show()

        plt.hist(dx_error*100)
        plt.title('Absolute bounding box error in x axis')
        plt.xlabel('Error in cm')
        plt.show()

        plt.hist(dy_error*100)
        plt.title('Absolute bounding box error in y axis')
        plt.xlabel('Error in cm')
        plt.show()


        plt.hist(dz_error*100)
        plt.title('Absolute bounding box error in z axis')
        plt.xlabel('Error in cm')
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str,default='/home/marko/benchmark/test_bench_dat.csv', help='File containing benchmark data.')
    FLAGS = parser.parse_args()
    bench_eval = BenchmarkEvaluation(FLAGS.file)
    bench_eval.evaluate()



