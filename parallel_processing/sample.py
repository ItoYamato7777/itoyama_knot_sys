import numpy as np
from parallel_processing import parallel_processing

def dummy1(arg1, arg2):
  return arg1*arg2

def dummy2(arg1, arg2, arg3):
  return (arg1+arg2)/arg3

def main():
  func_list = [dummy1, dummy1, dummy2, dummy2, "return", None]
  args_list = [[2,3], [4,5], [1.,2.,3.], [4.,2.,3.],[np.nan], [1,2,3]]
  ret = parallel_processing(func_list, args_list)
  return ret

