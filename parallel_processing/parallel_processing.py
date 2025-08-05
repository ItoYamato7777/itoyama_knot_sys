from multiprocessing import Pool, cpu_count

def wrapper(func_args):
  func = func_args[0]
  args = func_args[1:]
  return func(*args)

def ret_none():
  return None

def ret_arg(arg):
  return arg

def parallel_processing(func_list, args_list):
  p = Pool(cpu_count())
  func_args = []
  for i in range(len(func_list)):
    if func_list[i] is None:
      func_args.append([ret_none])
    elif func_list[i] == "return":
      func_args.append([ret_arg]+args_list[i])
    else:
      func_args.append([func_list[i]] + args_list[i])

  ret = p.map(wrapper, func_args)
  p.close()
  return ret
