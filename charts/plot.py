import matplotlib.pyplot as plt
import csv
from os import listdir, path, stat
from stat import S_ISREG, ST_CTIME, ST_MODE


dir = '/Users/deanhart/Documents/udacity/self-driving-car-engineer-nanodegree/term2/github/CarND-PID-Control-Project/charts/'

# # graph all datasets
# files = [f for f in listdir(dir) if path.splitext(f)[1] == '.csv']
# saveTo = dir + 'results.eps'

# graph specific:
files = [
    # dir + 'tweak_all/0.25_0_4_3_0_1.csv',
    # dir + 'tweak_all/0.125_0_4_3_0_3.csv',
    # dir + 'tweak_all/0.1_0_4_3_0_3.csv',
    # dir + 'tweak_all/0.1_0_4_2_0_3.csv',
    dir + 'tweak_all/0.1_0_4_3_0_2.csv',
    # dir + 'tweak_all/0.1_0_4_2_0_2.csv',
    # dir + 'tweak_all/0.1_0_4_0.75_0_2.csv',
    # dir + 'tweak_all/0.1_0_4_0.75_0_3.csv',
    # dir + 'tweak_all/0.1_0_4_1_0_2.csv'
]

files = ((stat(path), path) for path in files)

# leave only regular files, insert creation date
files = ((stat[ST_CTIME], path)
           for stat, path in files if S_ISREG(stat[ST_MODE]))

print('files: ', files)

fig = plt.figure()
ax = plt.subplot(111)

for cdate, f in sorted(files):
    print('f: ', f)
    filename = path.splitext(path.basename(f))[0]
    args = filename.split('_')
    print('args: ', args)
    p_steering=args[0]
    i_steering=args[1]
    d_steering=args[2]

    if len(args)==3:
        graph_label = 'ps:{},is:{},ds:{}'.format(
            p_steering, i_steering, d_steering)
    else:
        p_throttle=args[3]
        i_throttle=args[4]
        d_throttle=args[5]
        graph_label = 'ps:{},is:{},ds:{},pt:{},it:{},dt:{}'.format(
            p_steering, i_steering, d_steering, p_throttle, i_throttle, d_throttle)

    with open(f,'r') as csvFile:
        print('reading: ', f)
        plots = csv.reader(csvFile, delimiter=',')
        x=[]
        y=[]
        for row in plots:
            # 0: ts
            # 1: cte
            # 2: angle
            # 3: steer value
            # 4: speed
            # 5: throttle
            x.append(int(row[0]))
            y.append(float(row[5]))
        ax.plot(x, y, label=graph_label)

# Put a legend to the right of the current axis
# ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax.legend(loc='best')

plt.xlabel('time')
# plt.ylabel('CTE')
# plt.ylabel('Angle')
# plt.ylabel('Steer Value')
# plt.ylabel('Speed')
plt.ylabel('Throttle')
plt.title('Manual PIT Optimization')

saveTo = dir + 'tweak_all/results-final-throttle.png'
plt.savefig(saveTo, bbox_inches='tight')
# plt.savefig(saveTo, format='eps', dpi=1000)
plt.show()




