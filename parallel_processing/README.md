# parallel_processing
parallel processing on Python

---

## How to Use
- import  
  ```
  from parallel_processing import parallel_processing
  ```

- define funcs   
  ```
  def dummy1(arg1, arg2):
      return arg1 * arg2
  def dummy2(arg1, arg2, arg3):
      return (arg1 + arg2) * arg3
  ```
- prepare func_list, args_list
  ```
  func_list = [dummy1, dummy1, dummy2, dummy2, None]
  args_list = [[2,3],[3,4],[1,2,3],[4,5,6],[]]
  ```
  
- calcurate
  ```
  result = parallel_processing(func_list, args_list)
  ```
  result is same with
  ```
  [dummy1(2,3), dummy1(3,4), dummy2(1,2,3), dummy2(4,5,6), None]
  ```
