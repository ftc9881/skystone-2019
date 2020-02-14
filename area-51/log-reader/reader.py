import re

filepath = 'logs/vuforia-log.txt'
outpath = 'result/vuforia-log.txt'

# 2020-02-11 18:12:18.024 3269-3520/com.qualcomm.ftcrobotcontroller D/TeamCode@VuforiaPosePredictedY: 21.89939642404716
# 2020-02-11 18:12:18.034 3269-3520/com.qualcomm.ftcrobotcontroller D/TeamCode@VuforiaPose: 5.516, 20.59, -0.097
def getWords(line):
    return [word.strip() for word in re.split('[\s|,]+', line) if not word.isspace()]

def formatData(data):
    return '\t'.join(["{}".format(d) for d in data])

with open(filepath) as file:
    result = ""
    for i, line in enumerate(file):
        words = getWords(line)
        for i, word in enumerate(words):
            if "TeamCode@VuforiaPose:" in word:
                data = [words[1], words[i+1], words[i+2], words[i+3]]
                result += formatData(data) + '\n'
    with open(outpath, 'w+') as outfile:
        outfile.write(result)
