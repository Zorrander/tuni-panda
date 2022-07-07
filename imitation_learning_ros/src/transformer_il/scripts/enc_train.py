
import gym
import numpy as np
import torch
import wandb

import argparse
import pickle
import random
import sys
import matplotlib.pyplot as plt

from imitation_transformer import *
from sklearn.metrics import mean_squared_error

from argparse import ArgumentParser

def mse(a, b):
	a = np.array(a)
	b = np.array(b)
	return np.square(np.subtract(a, b)).mean()



def encoder_train(ratio):
	weight_pth = 'weights/insert_goal_encoder_'+str(ratio)+'.pth'
	val_weight_pth = 'weights/insert_goal_encoder_'+str(ratio)+'_validated.pth'
	runner = ImitationTransformer('configs/config_origin.yaml')
	#runner.load()

	#runner.encoder.load_encoder(weight_pth)
	runner.encoder.init_enc_dataset(ratio=ratio)
	runner.encoder.update_enc_dataset_info()


	x, y, m, l = runner.encoder.get_enc_batch(batch_size=1)


	#test_s = x.detach().numpy()[0]


	batch_pred = runner.encoder.model(x, m, l)
	batch_pred = reverse_max_normalize(batch_pred.cpu().detach().numpy(), runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
	batch_gt = reverse_max_normalize(y.cpu().detach().numpy(), runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
	#print(runner.encoder.predict(test_s), batch_gt)
	print(batch_pred, batch_gt)
	#a()

	err = 100000
	step_cnt = 0
	while True:
		#runner.encoder.load_encoder(weight_pth)
		runner.encoder.init_enc_dataset(ratio=ratio)
		runner.encoder.update_enc_dataset_info()
		runner.encoder.train(max_iters=1, num_steps=100)
		runner.encoder.save_encoder(weight_pth)
		vali_err = runner.encoder.validation_step()
		step_cnt+=1
		'''
		if step_cnt % 5 == 0:
			
			res = runner.encoder.enc_evaluation()
			preds = []#res[0]
			gts = []#res[1]
			for r in res[:]:
				if r[2] > 5:
					y_hat = minmax_normalize(r[0], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
					y = minmax_normalize(r[1], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
					preds.append(y_hat)
					gts.append(y)

			#print(preds)
			vali_err = mse(preds, gts)
			print('validation:', vali_err)
		'''
		if vali_err < err:
			print('validation minimizing:', vali_err)
			runner.encoder.save_encoder(val_weight_pth)
			err = vali_err

def run_eval(ratio, weight_pth):
	runner = ImitationTransformer('configs/config_origin.yaml')
	runner.encoder.load_encoder(weight_pth)
	runner.encoder.init_enc_dataset(ratio=ratio)
	runner.encoder.update_enc_dataset_info()
	print(runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
	res = runner.encoder.enc_evaluation()
	return res, runner



def enc_eval_visualization(ratios, vis_type='mse'):
	#plt.plot (means)
	#plt.fill_between(range(6),means-stds,means+stds,alpha=.1)
	print('eval')

	weight_pth = 'weights/insert_goal_encoder_1.0_validated.pth'

	runner = ImitationTransformer('configs/config_origin.yaml')
	runner.encoder.init_enc_dataset(ratio=1.0)
	runner.encoder.update_enc_dataset_info()

	# make line plot	
	legends = [r'$D_{5}$', r'$D_{20}$',r'$D_{50}$', r'$D_{100}$', r'$D_{150}$', r'$D_{200}$']
	#plt.figure(figsize=(18, 6))
	for rid, ratio in enumerate(ratios):
		eval_pth = 'data/evaluations/enc_eval_res_'+str(ratio)+'.npy'

		res = np.load(eval_pth, allow_pickle=True)
		errs = []

		all_errs = []

		for r in res[0]:

			if r[2] < 12:
				y_hat = minmax_normalize(r[0], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
				y = minmax_normalize(r[1], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
				#print(y, y_hat)
				xe = np.abs(r[1][0]-r[0][0][0])* 1000
				ye = np.abs(r[1][1]-r[0][0][1])* 1000
				ange = np.abs(r[1][2]-r[0][0][2])
				
				msee = mse([y], [y_hat[0]])

				if vis_type=='mse':
					errs.append(msee)
				elif vis_type=='x':
					errs.append(xe)
				elif vis_type=='y':
					errs.append(ye)
				elif vis_type=='ang':
					errs.append(ange)
				
			elif r[2]==12:
				#errs = np.array(errs)
				#plt.figure(1)
				#plt.plot(errs, '-o')
				#plt.plot(errs)
				all_errs.append(errs)
				errs = []
			else:
				True

		e_std = np.std(np.array(all_errs).flatten())
		e_mean = np.mean(np.array(all_errs).flatten())
		print(vis_type, rid, 'mean pm std', f'{e_mean:.2f}', '\\pm', f'{e_std:.2f}')
		
		# calculate mean and std
		all_errs = np.array(all_errs)
		all_means = np.mean(all_errs, axis=0)
		#print(all_means.shape)
		all_stds = np.std(all_errs, axis=0)
		plt.plot (all_means, '-o', label=legends[rid])
		plt.fill_between(range(all_errs.shape[1]),all_means-all_stds,all_means+all_stds,alpha=.15)

	SMALL_SIZE = 8
	MEDIUM_SIZE = 14
	BIGGER_SIZE = 12

	plt.rc('font', size=MEDIUM_SIZE)		  # controls default text sizes
	plt.rc('axes', titlesize=MEDIUM_SIZE)	 # fontsize of the axes title
	plt.rc('axes', labelsize=MEDIUM_SIZE)	# fontsize of the x and y labels
	plt.rc('xtick', labelsize=MEDIUM_SIZE)	# fontsize of the tick labels
	plt.rc('ytick', labelsize=MEDIUM_SIZE)	# fontsize of the tick labels
	plt.rc('legend', fontsize=MEDIUM_SIZE)	# legend fontsize
	plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title

	plt.xticks(np.arange(0, 11, 1.0))
	plt.grid()
	plt.legend(loc="upper right")
	if vis_type=='mse':
		plt.yticks(np.arange(0, 0.9, 0.1))
		plt.xlabel('Number of interaction steps', fontsize=MEDIUM_SIZE)
		plt.ylabel('MSE', fontsize=MEDIUM_SIZE)
	if vis_type=='x':
		#plt.yticks(np.arange(0, 0.9, 0.1))
		plt.xlabel('Number of interaction steps', fontsize=MEDIUM_SIZE)
		plt.ylabel('predicted x-distance difference (mm)', fontsize=MEDIUM_SIZE)
	if vis_type=='y':
		#plt.yticks(np.arange(0, 0.9, 0.1))
		plt.xlabel('Number of interaction steps', fontsize=MEDIUM_SIZE)
		plt.ylabel('predicted y-distance difference (mm)', fontsize=MEDIUM_SIZE)
	if vis_type=='ang':
		#plt.yticks(np.arange(0, 0.9, 0.1))
		plt.xlabel('Number of interaction steps', fontsize=MEDIUM_SIZE)
		plt.ylabel('predicted angle difference (degree)', fontsize=MEDIUM_SIZE)

	plt.savefig('enc_eval_'+vis_type+'.pdf')
	plt.show()


	# make hista plot
	his_values = []
	for rid, ratio in enumerate(ratios):
		eval_pth = 'data/evaluations/enc_eval_res_'+str(ratio)+'.npy'

		res = np.load(eval_pth, allow_pickle=True)
		
		corr_cnt = 0
		cnt = 0

		errs = []
		for r in res[0]:
			if r[2] < 12:
				y_hat = np.array(r[0])#minmax_normalize(r[0], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
				y = np.array(r[1])#minmax_normalize(r[1], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
				e = np.abs(y - y_hat)
				errs.append(e[0])

			elif r[2]==12:
				#errs = np.array(errs)
				#plt.figure(1)
				#plt.plot(errs, '-o')
				#plt.plot(errs)
				for e in errs:
					#print(e)
					if e[0]<0.0065 and e[1] < 0.0065 and e[2]<1.5:
						corr_cnt+=1
						break
				errs = []
				cnt += 1
			else:
				True


		#graph1, ax = plt.subplots()

		his_values.append(corr_cnt)
		print(corr_cnt, cnt)

	fig = plt.figure()
	plt.figure(figsize=(8, 4.6))
	#ax = fig.add_axes([0,0,1,1])
	plt.bar(legends,his_values, width=0.4)
	plt.grid(color='#95a5a6', linestyle='--', linewidth=2, axis='y', alpha=0.7)
	plt.yticks(np.arange(0, 46, 5.0))

	plt.ylabel('Number f successful predictions', fontsize=MEDIUM_SIZE)
	plt.savefig('enc_success_eval.pdf')
	plt.show()


def enc_eval(ratio):
	print('eval')
	eval_pth = 'data/evaluations/enc_eval_res_'+str(ratio)+'.npy'

	weight_pth = 'weights/insert_goal_encoder_'+str(ratio)+'.pth'
	res, runner = run_eval(ratio, weight_pth)

	preds = []#res[0]
	gts = []#res[1]
	
	for r in res[:]:
		if r[2] > 6:
			y_hat = minmax_normalize(r[0], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
			y = minmax_normalize(r[1], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
			preds.append(y_hat)
			gts.append(y)
			print(r[0], r[1])

	#print(preds)
	mse_res = mse(preds, gts)
	print(mse_res)
	vali_err = runner.encoder.validation_step()
	print(vali_err)
	#print(res)
	with open(eval_pth, 'wb') as f:
		np.save(f, [res, mse_res])
	

	weight_pth = 'weights/insert_goal_encoder_'+str(ratio)+'_validated.pth'

	#runner.load()

	
	res, runner = run_eval(ratio, weight_pth)

	preds = []#res[0]
	gts = []#res[1]
	
	for r in res[:]:
		if r[2] > 7:
			y_hat = minmax_normalize(r[0], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
			y = minmax_normalize(r[1], runner.encoder.enc_goal_mean, runner.encoder.enc_goal_max)
			preds.append(y_hat)
			gts.append(y)
			print(r[0], r[1])

	#print(preds)
	mse_res = mse(preds, gts)
	print(mse_res)
	vali_err = runner.encoder.validation_step()
	print(vali_err)
	#print(res)
	with open(eval_pth, 'wb') as f:
		np.save(f, [res, mse_res])
	
parser = ArgumentParser()
parser.add_argument("-r", "--ratio", default=0.1)
parser.add_argument("-t", "--train", default=1)
args = parser.parse_args()
print(args)
ratio = float(args.ratio)
train = float(args.train)
print(train)

ratios = [0.026, 0.1, 0.25, 0.5, 0.75, 1.0]

if train:
	for ratio in ratios:
		encoder_train(ratio)
else:
	#for ratio in ratios:
	#	enc_eval(ratio)


	enc_eval_visualization(ratios, vis_type='mse')
	enc_eval_visualization(ratios, vis_type='x')
	enc_eval_visualization(ratios, vis_type='y')
	enc_eval_visualization(ratios, vis_type='ang')


# python enc_train.py -r 0.026
# python enc_train.py -r 0.1
# python enc_train.py -r 0.25
# python enc_train.py -r 0.5 
# python enc_train.py -r 0.75
# python enc_train.py -r 1.0

'''
	# set width of bar
	barWidth = 0.25
	fig = plt.subplots(figsize =(12, 8))
	 
	# set height of bar
	IT = [10, 10, 8]
	ECE = [28, 6, 16, 5, 10]
	CSE = [29, 3, 24, 25, 17]
	 
	# Set position of bar on X axis
	br1 = np.arange(len(IT))
	br2 = [x + barWidth for x in br1]
	br3 = [x + barWidth for x in br2]
	 
	# Make the plot
	plt.bar(br1, IT, color ='r', width = barWidth,
	        edgecolor ='grey', label ='IT')
	plt.bar(br2, ECE, color ='g', width = barWidth,
	        edgecolor ='grey', label ='ECE')
	plt.bar(br3, CSE, color ='b', width = barWidth,
	        edgecolor ='grey', label ='CSE')
	 
	# Adding Xticks
	plt.xlabel('Branch', fontweight ='bold', fontsize = 15)
	plt.ylabel('Students passed', fontweight ='bold', fontsize = 15)
	plt.xticks([r + barWidth for r in range(len(IT))],
	        ['2015', '2016', '2017', '2018', '2019'])
	 
	plt.legend()
	plt.show()
	A()
'''
