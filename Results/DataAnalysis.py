import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.transforms as trans

def main():

    #### Pista 1
    data_vfh = np.genfromtxt('Pista1_VFH_t_data.csv', delimiter=',')
    data_brait = np.genfromtxt('Pista1_Brait_t_data.csv', delimiter=',')

    print "Pista 1:"
    for data in [data_vfh, data_brait]:
        if data is data_vfh:
            Name = 'VFH'
        else:
            Name = 'Brait'

        ISE = np.trapz(np.square(data[:,4]),data[:,0]/1000)
        IAE = np.trapz(np.abs(data[:,4]),data[:,0]/1000)
        ITAE = np.trapz(np.abs(data[:,4])*data[:,0]/1000,data[:,0]/1000)
        print "{} ISE = {:.2f}".format(Name,ISE)
        print "{} IAE = {:.2f}".format(Name,IAE)
        print "{} ITAE = {:.2f}".format(Name,ITAE)
    print

    #### Pista 2
    data_vfh = np.genfromtxt('Pista2_VFH_t_data.csv', delimiter=',')
    data_brait = np.genfromtxt('Pista2_Brait_t_data.csv', delimiter=',')

    print "Pista 2:"
    for data in [data_vfh, data_brait]:
        if data is data_vfh:
            Name = 'VFH'
        else:
            Name = 'Brait'

        ISE = np.trapz(np.square(data[:,4]),data[:,0]/1000)
        IAE = np.trapz(np.abs(data[:,4]),data[:,0]/1000)
        ITAE = np.trapz(np.abs(data[:,4])*data[:,0]/1000,data[:,0]/1000)
        print "{} ISE = {:.2f}".format(Name,ISE)
        print "{} IAE = {:.2f}".format(Name,IAE)
        print "{} ITAE = {:.2f}".format(Name,ITAE)
    print

    #### Pista 3
    data_vfh = np.genfromtxt('Pista3_VFH_t_data.csv', delimiter=',')
    data_brait = np.genfromtxt('Pista3_Brait_t_data.csv', delimiter=',')

    print "Pista 3:"
    for data in [data_vfh, data_brait]:
        if data is data_vfh:
            Name = 'VFH'
        else:
            Name = 'Brait'

        ISE = np.trapz(np.square(data[:,4]),data[:,0]/1000)
        IAE = np.trapz(np.abs(data[:,4]),data[:,0]/1000)
        ITAE = np.trapz(np.abs(data[:,4])*data[:,0]/1000,data[:,0]/1000)
        print "{} ISE = {:.2f}".format(Name,ISE)
        print "{} IAE = {:.2f}".format(Name,IAE)
        print "{} ITAE = {:.2f}".format(Name,ITAE)
    print

if __name__ == "__main__":
    main()
