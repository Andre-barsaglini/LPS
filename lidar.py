import argparse
import serial
import datetime
import time
import os
import sys
import h5py
import statistics
import math
#import numpy as np

##################################CRC##################################
CRCtable = (0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 
0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16,
0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 
0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 
0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 
0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 
0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 
0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 
0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 
0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 
0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 
0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 
0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d, 
0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 
0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 
0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 
0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 
0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 
0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8)

def crc(sample):
    val = sample[:-1]
    c = 0
    e = 0
    for i in range(0,len(val)):
        c = CRCtable[(c^val[i])&255]
    if (c != int.from_bytes(sample[-1:])):
        e = 1
    return e

##################################LiDAR##################################
   
parser = argparse.ArgumentParser(prog="LRPS", 
                    description="Sistema de posicionamento por LiDAR do LiTS",
                    formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-d", "--device", help="device to read from", 
                     default="/dev/ttyUSB0")
parser.add_argument("-plot", "--plot", help="plotagem, por hora, somente em modo detect", 
                     default=False)
parser.add_argument("modo", help="modo de operação do script", choices=['log', 
                    'show', 'debug', 'detect'])
args = parser.parse_args()

with serial.Serial(args.device, 230400) as ser: 
    if(args.modo == "log"):
        try:
            f = h5py.File(str(datetime.datetime.now().strftime("%y%m%d%H%M%S")+
                            ".hdf5"),'a')            
        except:
            print("falha na criação do arquivo")
            sys.exit(1)
        print("Log iniciado as: "+str(
                     datetime.datetime.now().strftime("%H:%M:%S:%f"))+
                     "... Ctrl+C para parar!") 
        
        ds = f.create_dataset("dados", (0,4), maxshape=(None, 4), dtype='int64',
                            chunks=True) 
    if(args.modo =="detect"):
        print("\n")
        print("\tCtrl+C para parar!")
        print("\n")
        print("\tdirecao:\tdistancia:\tamostras:")
        cur = [0,40000,0,0]
        vert=0
        basedis = 30
        radius = 200
        criterio = 150
        sweep = [[0 for i in range(4)] for j in range(500)] 
        direcoes = []
        distancias = []     
    try:
        ser.flush()
        head = b'\x54\x2c'
        while True:
            package = ser.read_until(head)
            msg = b''.join([head, package[:-2]])
            teste = crc(msg)
            if (teste>0):
                teste = 0
                continue
            rs = msg[2:4]
            sa = int.from_bytes(msg[4:6], byteorder='little')
            ea = int.from_bytes(msg[42:44], byteorder='little')
            ts = int.from_bytes(msg[44:46], byteorder='little')
            data = msg[6:42]
            meas = [[0 for i in range(4)] for j in range(12)]
            for k in range(12):
                meas[k][0] = int.from_bytes(data[3*k:(3*k)+2], byteorder='little')
                meas[k][1] = int((sa + ((ea-sa)/11)*k))
                if (ea<sa):
                    ea2 = ea+36000
                    meas[k][1] = int((sa + ((ea2-sa)/11)*k))
                if (meas[k][1]>36000):
                    meas[k][1]=meas[k][1]-36000
                meas[k][2] = int.from_bytes(data[3*k+2:(3*k)+3], byteorder='little')
                meas[k][3] = int(datetime.datetime.now().strftime("%H%M%S%f"))
            #aquitemos os dados completos de um pacote lido
            #
            
##################################"log"##################################                
            if (args.modo == "log"):
                dataset = f["dados"]
                current_size = dataset.shape[0]
                dataset.resize(current_size + len(meas), axis=0)
                dataset[current_size:] = meas       
##################################"debug"##################################
            if (args.modo =="debug"):
                print("Radar Speed: "+str(int.from_bytes(rs, byteorder='little'))
                        +" deg/s")
                print("Start Angle: "+str(float(sa)/100)+" deg")
                print("End Angle: "+str(float(ea)/100)+" deg")
                print("Timestamp: "+str(ts)+" ms")
                print("Data: "+data.hex())
                print("range|ang|conf")
                for k in range(12):
                    print (meas[k][0],meas[k][1], meas[k][2])
                print("\n")
                print("erros de CRC: "+str(erros))    
                print("CTRL + C para finalizar")
                print("=====================================================================")
                print("\n")
##################################"show"##################################
            if (args.modo =="show"):
                for k in range(12):
                    print (f"{meas[k][0]:05d}",f"{meas[k][1]:05d}",
                             f"{meas[k][2]:03d}", meas[k][3])
##################################"detect"##################################
            if (args.modo =="detect"):
                for m in range(12):
                    if (meas[m][1]<cur[1]): 
                        if (len(direcoes)>0):
                            dismed = int(statistics.mean(distancias))
                            if (max(direcoes)-min(direcoes)>33000):
                                for i in range(len(direcoes)):
                                    if (direcoes[i]<20000):
                                        direcoes[i] += 36000
                            dirmed = int(statistics.mean(direcoes))
                            if dirmed>36000:
                                dirmed -= 36000
                            amostras = len(distancias)
                            
                            print("\t"+str(f"{int(dirmed/100):03d}")+"º\t\t"+ 
                                    str(f"{dismed:05d}")+" mm\t\t"+
                                    str(amostras), end='\r')
                        else:
                            print("\t-     \t\t-        \t\t-     ", end='\r')
                        vert=0
                        direcoes.clear()
                        distancias.clear()
                        sweep.clear()
                        sweep = [[0 for i in range(4)] for j in range(500)]
                    cur = meas[m]
                    if(meas[m][0]>basedis and  meas[m][0]<radius and meas[m][2]>criterio):
                        sweep[vert] = cur
                        vert+=1
                        direcoes.append(meas[m][1])
                        distancias.append(meas[m][0])
                    #if
                            
            
##################################fim##################################               
    except KeyboardInterrupt:
        if(args.modo == "log"):
            print("Log finalizado as: "+str(datetime.datetime.now().strftime("%H:%M:%S:%f")))
            f.close()
        print("\nFim")
