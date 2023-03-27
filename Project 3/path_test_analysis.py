# path_test_analysis.py

dataGroups = []
for i in range(1,17):
    with open("path_test_data" + str(i) + ".csv",'r') as dataFile:
        header = dataFile.readline().split(',')
        rawData = []
        for line in dataFile.readlines():
            line = line.split(',')
            rawData.append([float(line[0]),int(line[1]),int(line[2])])
        
    dataGroups.append([rawData[0]])
    for i in range(1,len(rawData)-1):
        if rawData[i][1] == rawData[i-1][1] and rawData[i][2] == rawData[i-1][2]:
            if rawData[i][1] == rawData[i+1][1] and rawData[i][2] == rawData[i+1][2]:
                continue
        dataGroups[-1].append(rawData[i])

outFile = open("path_analysis_report.txt", 'w')
for i in range(len(dataGroups)):
    print("\nGroup number", i+1)
    print("\tTiming:")
    print("\t\tStart time:", dataGroups[i][0][0])
    print("\t\tEnd time:", dataGroups[i][-1][0])
    print("\t\tTotal duration:", dataGroups[i][-1][0] - dataGroups[i][0][0])
    print("\tLeft Wheel:")
    print("\t\tStarting position:", dataGroups[i][0][1])
    print("\t\tEnding position:", dataGroups[i][-1][1])
    print("\t\tTotal traverse:", dataGroups[i][-1][1] - dataGroups[i][0][1])
    print("\tRight Wheel:")
    print("\t\tStarting position:", dataGroups[i][0][2])
    print("\t\tEnding position:", dataGroups[i][-1][2])
    print("\t\tTotal traverse:", dataGroups[i][-1][2] - dataGroups[i][0][2])
    outFile.write("Group number " + str(i+1))
    outFile.write("\n\tTiming:")
    outFile.write("\n\t\tStart time: " + str(dataGroups[i][0][0]))
    outFile.write("\n\t\tEnd time: " + str(dataGroups[i][-1][0]))
    outFile.write("\n\t\tTotal duration: " + str(dataGroups[i][-1][0] - dataGroups[i][0][0]))
    outFile.write("\n\tLeft Wheel:")
    outFile.write("\n\t\tStarting position: " + str(dataGroups[i][0][1]))
    outFile.write("\n\t\tEnding position: " + str(dataGroups[i][-1][1]))
    outFile.write("\n\t\tTotal traverse: " + str(dataGroups[i][-1][1] - dataGroups[i][0][1]))
    outFile.write("\n\tRight Wheel:")
    outFile.write("\n\t\tStarting position: " + str(dataGroups[i][0][2]))
    outFile.write("\n\t\tEnding position: " + str(dataGroups[i][-1][2]))
    outFile.write("\n\t\tTotal traverse: " + str(dataGroups[i][-1][2] - dataGroups[i][0][2]) + "\n\n")
outFile.close()