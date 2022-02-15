import os
import matplotlib

# For the error: Exception ignored in: <bound method Image.del of <tkinter.PhotoImage object at 0x7f1b5f86a710>> Traceback (most recent call last):
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import json

'''
loc = '/model/parameters/'
_dir = os.listdir(loc)
evaluation = {
    'cut-in': {
        'param_ego_speed_init': [],
        'param_adv_speed_init': [],
        'param_start_diff': [],
        'param_cutin_start_dist': [],
        'param_cutin_time': [],
        'param_ego_to_cutin_dist': [],
        'param_ego_to_cutin_time': [],
        'param_adv_to_cutin_time': [],
        'param_adv_to_cutin_dist': []
    },
    'cut-out':{
        'param_ego_speed_init': [],
        'param_adv_speed_init': [],
        'param_start_diff': [],
        'param_cutout_start_dist': [],
        'param_cutout_time': [],
        'param_ego_to_cutout_dist': [],
        'param_ego_to_cutout_time': [],
        'param_adv_to_cutout_time': [],
        'param_adv_to_cutout_dist': []

    }
}
for _file in _dir:
    base = os.path.basename(_file)
    fileDetails = os.path.splitext(base)
    if fileDetails[1] == '.cutin' or fileDetails[1] == '.cutout':
        print("Processing the file: {}".format(_file))
        with open(_file) as f:
            data = json.loads(f.read())
        _type = data['type']
        if _type == 'cutin':
            for param in data['parameter']:
                evaluation['cut-in']['param_ego_speed_init'].append(param['param_ego_speed_init'])
                evaluation['cut-in']['param_adv_speed_init'].append(param['param_adv_speed_init'])
                evaluation['cut-in']['param_start_diff'].append(param['param_start_diff'])
                evaluation['cut-in']['param_cutin_start_dist'].append(param['param_cutin_start_dist'])
                evaluation['cut-in']['param_cutin_time'].append(param['param_cutin_time'])
                evaluation['cut-in']['param_ego_to_cutin_dist'].append(param['param_ego_to_cutin_dist'])
                evaluation['cut-in']['param_adv_to_cutin_time'].append(param['param_adv_to_cutin_time'])
                evaluation['cut-in']['param_adv_to_cutin_dist'].append(param['param_adv_to_cutin_dist'])
         
        if _type == 'cutout':
            for param in data['parameter']:
                evaluation['cut-out']['param_ego_speed_init'].append(param['param_ego_speed_init'])
                evaluation['cut-out']['param_adv_speed_init'].append(param['param_adv_speed_init'])
                evaluation['cut-out']['param_start_diff'].append(param['param_start_diff'])
                evaluation['cut-out']['param_cutout_start_dist'].append(param['param_cutout_start_dist'])
                evaluation['cut-out']['param_cutout_time'].append(param['param_cutout_time'])
                evaluation['cut-out']['param_ego_to_cutout_dist'].append(param['param_ego_to_cutout_dist'])
                evaluation['cut-out']['param_adv_to_cutout_time'].append(param['param_adv_to_cutout_time'])
                evaluation['cut-out']['param_adv_to_cutout_dist'].append(param['param_adv_to_cutout_dist'])
        
        

print(evaluation)

for key in evaluation['cut-in'].keys():
    path_to_save_dir = '/model/plots/param_plots/'
    name = path_to_save_dir+key
    plt.figure()
    hist = plt.hist(evaluation['cut-in'][key], bins='auto', color='#B45743', rwidth=0.25, normed=False)
    plt.ylabel("Frequency")
    plt.xlabel(key)
    plt.savefig(name)
    plt.close()
'''
#-----------------------------------esmini data------------------------------
path_to_save_dir = 'plots/'
_file = 'esmini_real_plot_data.json'
with open(_file) as f:
    data = json.loads(f.read())

print("total number of scenarios: {}".format(len(data['data'])))

'''
no = 7
key = 'speed'
adversary_esmini1 = data['data'][no]['adversary_esmini'] 
adversary_real1 = data['data'][no]['adversary_real'] 
ego_esmini1 = data['data'][no]['ego_esmini']
ego_real1 = data['data'][no]['ego_real']
esmini1 = data['data'][no]['esmini'] 
real1 = data['data'][no]['real'] 
'''
'''
no = 0
key = 'speed'
adversary_esmini2 = data['data'][no]['adversary_esmini'] 
adversary_real2 = data['data'][no]['adversary_real'] 
ego_esmini2 = data['data'][no]['ego_esmini'] 
ego_real2 = data['data'][no]['ego_real'] 
esmini2 = data['data'][no]['esmini'] 
real2 = data['data'][no]['real'] 
'''

no = 0
key = 'speed'
adversary_esmini3 = data['data'][no]['adversary_esmini']
adversary_real3 = data['data'][no]['adversary_real']
ego_esmini3 = data['data'][no]['ego_esmini']
ego_real3 = data['data'][no]['ego_real']

adversary_esmini_sec3 = data['data'][no]['adversary_esmini_sec']
adversary_real_sec3 = data['data'][no]['adversary_real_sec']
ego_esmini_sec3 = data['data'][no]['ego_esmini_sec']
ego_real_sec3 = data['data'][no]['ego_real_sec']

name = path_to_save_dir+"test"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+" (km/hr)")
plt.xlim([0, 11])
#plt.plot(adversary_real1['sec'], adversary_real1[key])
#plt.plot(adversary_esmini1['sec'], adversary_esmini1[key], '--')
#plt.plot(adversary_real2['sec'], adversary_real2[key])
#plt.plot(adversary_esmini2['sec'], adversary_esmini2[key], '--')
plt.plot(adversary_real_sec3['sec'], adversary_real_sec3[key])
plt.plot(adversary_esmini_sec3['sec'], adversary_esmini_sec3[key], '--')
plt.legend(['Real-world', 'OpenSCENARIO'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()

name = path_to_save_dir+"test_ego"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+' (km/hr)')
#plt.ylim([0, 20])
plt.plot(ego_real_sec3['sec'], ego_real_sec3[key], label='real')
plt.plot(ego_esmini_sec3['sec'], ego_esmini_sec3[key], label='generated')
plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()


adversary_esmini_milli = data['data'][no]['adversary_esmini_milli']
adversary_real_milli = data['data'][no]['adversary_real_milli']

sec = []
speed = []
for index in range(len(adversary_esmini_sec3['speed'])):
    sec.append(index*10)
    speed.append(adversary_esmini_sec3['speed'][index])

sec_real = []
speed_real = []
for index in range(len(adversary_real_milli['speed'])):
    sec_real.append(index)
    speed_real.append(adversary_real_milli['speed'][index])

name = path_to_save_dir+"test_other_milli"
plt.figure()
plt.xlabel("second")
plt.ylabel(key+' (km/hr)')
#plt.ylim([0, 20])
#plt.plot(adversary_esmini_milli['speed'], label='generated')
plt.plot(adversary_esmini_milli['sec'], adversary_esmini_milli['speed'], label='real')
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.legend(['Real-world', 'OpenSCENARIO'])
plt.savefig(name)

key = 's'
name = path_to_save_dir+"test_s_t"
plt.figure()
plt.xlabel("t, lateral displacement")
plt.ylabel('s, longitudinal displacement')
plt.xlim([-1, -5])
#plt.ylim([100, 130])
#plt.plot(adversary_real1['sec'], adversary_real1[key])
#plt.plot(adversary_esmini1['sec'], adversary_esmini1[key], '--')
#plt.plot(adversary_real2['sec'], adversary_real2[key])
#plt.plot(adversary_esmini2['sec'], adversary_esmini2[key], '--')
plt.plot(adversary_real3['t'], adversary_real3[key])
plt.plot(adversary_esmini3['t'], adversary_esmini3[key], '--')
plt.legend(['Real-world', 'OpenSCENARIO'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()



'''
name = path_to_save_dir+"test_rss"
key = 'rss'
plt.figure()
plt.xlabel("second")
plt.ylabel(key)
plt.ylim([10, 40])
#plt.plot(esmini1['sec'], esmini1[key], '--')
#plt.plot(real1['sec'], real1[key])
#plt.plot(esmini2['sec'], esmini2[key], '--')
#plt.plot(real2['sec'], real2[key])
plt.plot(esmini3['sec'], esmini3[key], '--')
plt.plot(real3['sec'], real3[key])
plt.legend(['RSS computed in OpenSCENARIO', 'RSS computed in real data'])
#plt.legend(loc="upper right", prop={'size': 8}, labelspacing=0.1, bbox_to_anchor=(1.125,1))
plt.savefig(name)
plt.close()
'''



