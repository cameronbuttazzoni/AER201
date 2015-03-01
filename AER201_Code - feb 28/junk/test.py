count = 0
lines = 1000
f = open("AER201_Code.ino", 'r')
#f = open("test1.txt", 'r')
line = f.readline()
count_flag = 1
while (line != '' or lines > 0):
    try:
        temp = 0
        while line[temp] == ' ':
            temp += 1
        if line[temp] == '/' and line[temp+1] == '/':
            lines -= 1
            line = f.readline()
            continue
    except IndexError:
        pass
    for x in range(len(line)):
        if line[x] == '(':
            count += 1
            count_flag = 0
        if line[x] == ')':
            count -= 1
        if (count == 1 and count_flag != 1):
            print line
            count_flag = 1
    line = f.readline()
    lines -= 1
print count
