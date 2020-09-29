#!/usr/bin/env python
import requests
import os.path
import sys
url = "https://www.gosolarcalifornia.ca.gov/equipment/documents/Grid_Support_Inverter_List_Full_Data.xlsm"
content = requests.get(url)
directory = raw_input("Enter the directory path:")
fileName = raw_input("Please enter the name of the file: ")
extension = raw_input("What is the extension of the file(.txt, .pdf)?: ")
if os.path.isdir(directory):
    print "SUCCESS: Directory was found."
    counter = 0
    if (os.path.exists(directory+"\\"+fileName+extension)):
        counter += 1
        while (os.path.exists(directory+"\\"+fileName+"_%s"%(str(counter))+extension)):
            counter += 1
    if counter > 0:
        print "File already exists in directory, renaming to: %s_%s%s"%(fileName, str(counter),extension)
        with open(os.path.join(directory, fileName+"_%s"%(str(counter)+extension)), 'wb') as f:
            f.write(content.content)
    else:
         with open(os.path.join(directory, fileName+extension), 'wb') as f:
            f.write(content.content)
    print"Done."
else:
    print"ERROR: Directory is not found."
    sys.exit()