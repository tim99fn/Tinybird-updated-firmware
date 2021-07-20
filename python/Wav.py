from scipy.io.wavfile import write
import numpy as np
import pandas as pd
#import tqdm
def make_wav():
    data = np.array(pd.read_csv("yourdirectory.csv", sep=',',header=None))
    if(data.size%2==1):
        data=data[:-1]
    other_byte=data[:-1]
    other_byte=np.delete(other_byte,0)
    other_byte=np.asarray(other_byte, dtype=np.uint8)
    other_byte=other_byte.reshape((int(other_byte.size/2),2))
    other_byte16=other_byte.view(dtype=np.uint16,type=np.matrix)
    other_byte16=np.asarray(other_byte16,dtype=np.int16)
    data_uint8 = np.asarray(data, dtype=np.uint8)
    print(data_uint8.shape)
    data_uint8=data_uint8.reshape((int(data_uint8.size/2),2))
   
    data_uint16=data_uint8.view(dtype=np.uint16,type=np.matrix)
    data_uint16=np.asarray(data_uint16,dtype=np.int16)
   

    write("/yourdirectory/NewWAV.wav",16000,data_uint16)
    write("/yourdirectory/otherWAV.wav",16000,other_byte16)
make_wav()