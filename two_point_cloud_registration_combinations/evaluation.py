import os

# modify this path to load the log files
root_path = ""
filenames = os.listdir(root_path)
initial_success_comb = []
total_success_comb = []
for file in filenames:
    log = open(root_path+file)
    log_lines = log.readlines()
    if len(log_lines)< 23:
        continue
    initial_success_comb.append(file)
# print(len(initial_success_comb))
# print(initial_success_comb)
for file in initial_success_comb:
    log = open(root_path+file)
    log_lines = log.readlines()
    line = log_lines[16].strip('\n').split(" ")
    if_success = line[2]
    if if_success =="1":
        total_success_comb.append([file,float(line[-1])])
sorted_success_comb = sorted(total_success_comb, key=lambda x:x[1],reverse=False)
print("Num of successful combinations: ", len(sorted_success_comb),"/", len(filenames))
print("Top 5 combinations are ")
for i in range(0,5):
    print("Combinations: ",sorted_success_comb[i][0][:-4]," Scores: ", sorted_success_comb[i][1])
