from __future__ import print_function

import os
import csv
import numpy as np
import json
from datetime  import datetime

def read_txt(fileName):
  f = open(fileName)
  return f.read()
  
def read_csv(fileName, dataType = str): 
  '''
  read csv file
  fileName: file name of .csv file
  dataType: data type (e.g., int, float...) default: str
  '''
  csvObj = csv.reader(open(fileName,'r'))
  data=[v for v in csvObj]
  if [] in data:
    data.remove([])

  if dataType == str:
    return data
  
  else:
    ret = []
    for line in data:
      tmp = []
      for dat in line:
        try:
          tmp.append(dataType(dat))
        except:
          tmp.append(dat)
      ret.append(tmp)

    return ret


def read_json(fileName):
  txt = read_txt(fileName)
  return json.loads(txt)


class writer:
  def __init__(self, fileName, add_time = False, overwrite = False):
    if overwrite == True:
      if os.path.exists(fileName):
        if os.path.exists(fileName + "~"):
          os.remove(fileName + "~")
        os.rename(fileName, fileName + "~")

    if add_time == False:
      self.fileName = fileName

    else:
      lst = list(os.path.split(fileName))
      dot_pos = lst[-1].find(".")
      
      if dot_pos >= 0:
        lst[-1] = lst[-1][0:dot_pos] + "-" + get_time() + lst[-1][dot_pos:]
        self.fileName = os.path.join(*lst)

      if dot_pos < 0:
        lst[-1] = lst[-1] + "-" + self.get_time()


      self.fileName = os.path.join(*lst)
    print("file name: ", self.fileName)
        
      
  def write(self, txt):
    f = open(self.fileName, "a")
    f.writelines(txt)
    f.close

  def csv_write(self, lst):
    f = open(self.fileName,'ab')
    csvWriter=csv.writer(f)
    csvWriter.writerow(lst)
    f.close()

  def json_write(self, dct):
    str_json = json.dumps(dct, indent = 2)
    self.write(str_json)

def get_time():
  '''getting time for filename'''
  return datetime.now().isoformat().replace(":", "-").replace(".","-")

