import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as mpatches
import numpy as np

def plot_NIS(NIS_file):

	f = open(NIS_file, 'r')
	lidar_NIS = f.readline().split(" - ")
	radar_NIS = f.readline().split(" - ")
	NIS = f.readline().split(" - ")

	lidar_NIS = np.asarray(lidar_NIS).astype(np.float)
	radar_NIS = np.asarray(radar_NIS).astype(np.float)
	NIS = np.asarray(NIS).astype(np.float)
	lidar_limit = [5.991 for i in range(0,len(lidar_NIS))]
	radar_limit = [7.815 for i in range(0,len(radar_NIS))]

	combined_limit = [5.991 for i in range(0,len(NIS))]


	f, (ax0, ax1, ax2)  = plt.subplots(1, 3)

	ax0.plot(lidar_NIS,'r-',lidar_limit,'b-')
	ax0.grid()

	#ax0.set_title('Original RGB Image', fontsize=20)
	ax1.plot(radar_NIS,'r-',radar_limit,'b-')
	#ax1.set_title('Color/Sobel mask application', fontsize=20)
	#plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)

	#plt.legend(handles=[original_data,computed_data])
	ax1.grid()
	ax2.plot(NIS,'r-',combined_limit,'b-')
	ax2.grid()

	ax0.set_ylim([0,max(10,max(lidar_NIS))])
	ax1.set_ylim([0,max(10,max(radar_NIS))])
	ax2.set_ylim([0,max(10,max(NIS))])

	plt.show()

def plot_Path(input_file,output_file):

	i_f = open(input_file, 'r')
	o_f = open(output_file, 'r')

	input_px = []
	input_py = []

	output_px = []
	output_py = []
	for line in i_f:
		entry_values = line.split()
		if(entry_values[0] == "L"):
			input_px.append(float(entry_values[1]))
			input_py.append(float(entry_values[2]))
		if(entry_values[0] == "R"):
			input_px.append(float(entry_values[1]) * np.cos(float(entry_values[2])))
			input_py.append(float(entry_values[1]) * np.sin(float(entry_values[2])))
	o_f.readline()
	for line in o_f:
		entry_values = line.split()
		output_px.append(entry_values[1])
		output_py.append(entry_values[2])


	original_data = mpatches.Patch(color='red', label='The original data')
	computed_data = mpatches.Patch(color='blue', label='The computed data')

	plt.plot(input_px,input_py,'ro',output_px,output_py,'bx')
	plt.plot(input_px,input_py,'r-',output_px,output_py,'b-')

	plt.legend(handles=[original_data,computed_data], loc=4)
	plt.grid()

	plt.show()

if __name__ == "__main__":

	input_file =  "..\data\obj_pose-laser-radar-synthetic-input.txt"
	output_file =  "..\data\obj_pose-laser-radar-synthetic-input-output.txt"
	NIS_file =  "..\data\obj_pose-laser-radar-synthetic-NIS.txt"

	#input_file =  "..\data\sample-laser-radar-measurement-data-1.txt"
	#output_file =  "..\data\sample-laser-radar-measurement-data-1-output.txt"
	#NIS_file =  "..\data\sample-laser-radar-measurement-data-1-NIS.txt"

	#input_file =  "..\data\sample-laser-radar-measurement-data-2.txt"
	#output_file =  "..\data\sample-laser-radar-measurement-data-2-output.txt"
	#NIS_file =  "..\data\sample-laser-radar-measurement-data-2-NIS.txt"

	plot_NIS(NIS_file)
	plot_Path(input_file, output_file)