import re

filepath = 'result/vuforia-log.txt'
outpath = 'result/stripped-vuforia-log.txt'

# 18:12:18.487	5.475	20.898	-0.319

def getWords(line):
    return [word.strip() for word in re.split('[\s|,]+', line) if not word.isspace()]

with open(filepath) as file:
    result = ""
    previousData = 'hi';
    for i, line in enumerate(file):
        words = getWords(line)
        if words[2] != previousData:
            previousData = words[2]
            result += line
    with open(outpath, 'w+') as outfile:
        outfile.write(result)


