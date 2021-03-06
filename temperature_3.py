import time

base_dir = '/sys/bus/w1/devices/'
sensors = ['28-0000065be5ef',
        '28-0000065c181d',
        '28-0000065cd66d']
device_files = {s_i:(base_dir + s_i + '/w1_slave') for s_i in sensors}

def read_temp_raw(device_file):
    for i in range(5):
        f = open(device_file, 'r')
        lines = f.readlines()
        f.close()
        if lines != []:
            break
    return lines

def read_temp(device_file):
    lines = read_temp_raw(device_file)
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c, temp_f

while True:
    for s_i in sensors:
        _,Tf = read_temp(device_files[s_i])
        print(f"{s_i} : Temp is {Tf} F")
    print("----------")
    time.sleep(1)
