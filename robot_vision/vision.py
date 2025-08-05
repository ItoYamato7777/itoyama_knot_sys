import cv2
import numpy as np
import math
from math import pi
from geo import FRAME, MATRIX, VECTOR

##  color image ##
# img[y, x] == np.array([b, g, r])

##  grayscale image ##
# img[y, x] == np.array([color])
# color: from 0 (black) to 255 (white)

##  binary image   ##
# img[y, x] == np.array([color])
# color: 0 (black) or 255 (white)

def get_perspective_transform(rvec, tvec, mtx, dist, vec_from, pos_to):
  pos_to = np.float32(pos_to)
  pos_from, jac = cv2.projectPoints(vec_from, rvec, tvec, mtx, dist)
  pos_from = np.float32(pos_from)
  mat = cv2.getPerspectiveTransform(pos_from, pos_to)
  return mat

def warp_perspective(src, mat, img_size):
  return cv2.warpPerspective(src, mat, img_size)

def thresholding(img, hsv_min, hsv_max, inv=False):
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  ret = cv2.inRange(hsv, np.array(hsv_min), np.array(hsv_max))
  if inv:
    return cv2.bitwise_not(ret)
  else:
    return ret

def thinning(img):
  src = np.ones_like(img, dtype = np.float32)
  src[img == 0] = -1
  patterns = [np.array([[-1,-1, 0],[-1, 1, 1],[ 0, 1, 0]]),
              np.array([[-1,-1,-1],[ 0, 1, 0],[ 1, 1, 0]]),
              np.array([[ 0,-1,-1],[ 1, 1,-1],[ 0, 1, 0]]),
              np.array([[ 1, 0,-1],[ 1, 1,-1],[ 0, 0,-1]]),
              np.array([[ 0, 1, 0],[ 1, 1,-1],[ 0,-1,-1]]),
              np.array([[ 0, 1, 1],[ 0, 1, 0],[-1,-1,-1]]),
              np.array([[ 0, 1, 0],[-1, 1, 1],[-1,-1, 0]]),
              np.array([[-1, 0, 0],[-1, 1, 1],[-1, 0, 1]])]
  removed_count = 1
  while removed_count > 0:
    removed_count = 0
    for kp in patterns:
      remove = cv2.filter2D(src, -1,  kp, borderType = cv2.BORDER_CONSTANT)
      remove = remove > 5.9
      src[remove] = -1
      removed_count += np.sum(remove * 1)
  result = np.array((src > 0) * 255, dtype = np.uint8)
  return result

def count_neighborhood(img, connectivity = 8):
  if connectivity == 8:
    conv_mat = np.array([[1.,1.,1.],[1.,0.,1.],[1.,1.,1.]])
  elif connectivity == 4:
    conv_mat = np.array([[0.,1.,0.],[1.,0.,1.],[0.,1.,0.]])
  else:
    raise ValueError
  #white : 255 -> 1
  binary = np.zeros_like(img, np.float32)
  binary[img != 0] = 1.0
  count = cv2.filter2D(binary, cv2.CV_32F, conv_mat)#.astype(np.uint8)
  count[img==0] = np.nan
  return count

def apply_colormap(img, colormap = cv2.COLORMAP_JET):
  img = np.float32(img)
  max_value = np.nanmax(img)
  min_value = np.nanmin(img)
  gray = np.array((img - min_value)/(max_value - min_value) * 255, np.uint8)
  color = cv2.applyColorMap(gray, colormap)
  color[np.isnan(img)] = 0
  return color
  
def remove_cross_point(img):
  img2 = img.copy()
  count = count_neighborhood(img)
  img2[np.where(count > 2)] = 0
  return img2

def line_trace(img, start_xy, show = False):
  img2 = img.copy()
  h,w = img.shape[:2]
  x, y = start_xy
  line_pts = []
  while True:
    line_pts.append([x,y])
    img2[y,x] = 0
    if show:
      cv2.imshow('img', img2)
      cv2.waitKey(1)
    if x > 0:
      if img2[y, x-1] == 255:
        x = x-1
        continue
      if y > 0:
        if img2[y-1, x-1] == 255:
          x = x-1
          y = y-1
          continue
      if y < h-1:
        if img2[y+1, x-1] == 255:
          x = x-1
          y = y+1
          continue
    if x < w-1:
      if img2[y, x+1] == 255:
        x = x+1
        continue
      if y > 0:
        if img2[y-1, x+1] == 255:
          x = x+1
          y = y-1
          continue
      if y < h-1:
        if img2[y+1, x+1] == 255:
          x = x+1
          y = y+1
          continue
    if y > 0:
      if img2[y-1,x] == 255:
        y = y-1
        continue
    if y < h-1:
      if img2[y+1,x] == 255:
        y = y+1
        continue
    if show:
      cv2.destroyAllWindows()
    return line_pts


def polyline_approximation(line_pts):
  line = np.array(line_pts).T
  node_index = [0, line.shape[1] - 1]
  i = 0
  while True:
    linetmp = line[:, node_index[i]:node_index[i+1]+1] - line[:,node_index[i]].reshape((2,1))
    ang = - math.atan2(linetmp[1,-1], linetmp[0,-1])
    mat = np.array([[math.sin(ang), math.cos(ang)]])
    dist = np.abs(np.dot(mat, linetmp))
    max_index = np.argmax(dist)
    if dist[0, max_index] > 1:
      node_index.insert(i+1, node_index[i] + max_index)
    else:
      i = i+1
      if i+1 == len(node_index):
        break
  nodes = []
  for i in node_index:
    nodes.append(line_pts[i])
  return nodes

def make_polyline_length_angle_list(nodes):
  length = [0.0]
  angle_i = math.atan2(nodes[1][1]-nodes[0][1], nodes[1][0]-nodes[0][0])
  angle = [math.atan2(nodes[1][1]-nodes[0][1], nodes[1][0]-nodes[0][0])]
  
  for i in range(len(nodes)-1):
    length.append(length[i] + np.sqrt((nodes[i+1][1]-nodes[i][1]) ** 2 + (nodes[i+1][0]-nodes[i][0]) ** 2 ))

    angle_i_plus_1 = math.atan2(nodes[i+1][1]-nodes[i][1], nodes[i+1][0]-nodes[i][0])
    delta_angle = angle_i_plus_1 - angle_i
    angle_i = angle_i_plus_1
    if delta_angle > pi:
      delta_angle = delta_angle - 2 * pi
    elif delta_angle < -pi:
      delta_angle = delta_angle + 2 * pi
    angle.append(angle[i] + delta_angle)

  return length, angle

def get_pos_on_polyline(node, dist = None, ratio = None):
  node = np.array(node)
  length = [0]
  angle = [math.atan2(node[1][1] - node[0][1], node[1][0] - node[0][0])]

  #make lists of length and angle
  for i in range(len(node)-1):
    length.append(length[i] + math.sqrt(np.sum((node[i+1]-node[i])**2)))
    angle.append(math.atan2(node[i+1][1] - node[i][1], node[i+1][0]- node[i][0]))

  if not ratio is None:
    dist = length[-1] * ratio

  if dist > length[-1]:
    print 'dist is too large!!!'
    raise ValueError

  #search point
  for i in range(len(length)-1):
    if length[i+1] >= dist:
      break

  a = dist - length[i]
  b = length[i+1] - dist
  c = length[i+1] - length[i]
  pos = (b * np.array(node[i]) + a * np.array(node[i+1])) / float(c)
  theta = angle[i+1]
  return pos.tolist(), theta

def flood_fill(img, seed_pt, label_color, lo_diff, up_diff, connectivity):
  img2 = img.copy()
  h,w = img.shape[:2]
  mask = np.zeros((h+2, w+2), np.uint8)
  cv2.floodFill(img2, mask, seed_pt, label_color, lo_diff, up_diff, connectivity)
  return img2

def labeling(img, connectivity = 4):
  h,w = img.shape[:2]
  mask = np.zeros((h+2, w+2), np.uint8)
  label_max = 255
  label = np.zeros_like(img, np.uint8)
  label[np.where( img > 0.5 )] = label_max
  label_color = 1
  while True:
    pos = label.argmax()
    x = pos % w
    y = pos / w
    if label[y, x] == label_max:
      cv2.floodFill(label, mask, (x, y), label_color, 0, 0, connectivity)
      label_color += 1
    else:
      return label

def find_loop(img):
  loop_inv = flood_fill(img, (0,0), 127, 0, 0, 4)
  return np.array((loop_inv == 0) * 255, np.uint8)


def search_point(src, search_num, theta):
  init_flag=0
  h,w=src.shape[:2]
  for x in xrange(w):
    for y in xrange(h):
      if src[y,x]==search_num:
        rho = x * math.cos(theta) + y * math.sin(theta)
        if init_flag==0:
          p_min=[x,y,rho]
          p_max=[x,y,rho]
          init_flag=1
          continue        
        if p_min[2]>rho:
          p_min=[x,y,rho]
          continue
        if p_max[2]<rho:
          p_max=[x,y,rho]
  ans = [p_min,p_max]
  return ans

def get_pos_by_hand(img = None, cap_func = None, print_color = False, pos1st = [0,0]):
  marker_color_list = [(0,0,0),(0,0,255),(0,255,0),(255,0,0),(255,255,255)]
  marker_color_i = 0
  marker_size = 2
  pos = pos1st

  class mouseEvent():
    def __init__(self, win_name):
      self.clicked_pos = None
      cv2.setMouseCallback(win_name, self._callBack)
    def _callBack(self, event, x, y, flags, param):
      #print event
      if event == cv2.EVENT_LBUTTONUP:
        self.clicked_pos = [x,y]
    def return_clicked_pos(self):
      self.clicked_pos = None

  cv2.namedWindow("image")
  m_event = mouseEvent("image")

  while True:
    if not cap_func is None:
      disp = cap_func()
    elif not img is None:
      disp = np.copy(img)
    else:
      print "no image"
      raise ValueError
    if print_color:
      if len(disp.shape) == 3:
        hsv = cv2.cvtColor(disp, cv2.COLOR_BGR2HSV)
        print "BGR: ",disp[pos[1], pos[0]], "HSV: ",hsv[pos[1], pos[0]]
      elif len(disp.shape) == 2:
        print "VALUE: ", disp[pos[1], pos[0]]

    if len(disp.shape) == 2:
      disp = apply_colormap(disp)

    cv2.circle(disp, tuple(pos), marker_size, marker_color_list[marker_color_i], -1)
    cv2.imshow("image", disp)
    key = cv2.waitKey(33) % 0xff

    if not m_event.clicked_pos is None:
      pos = m_event.clicked_pos
      m_event.clicked_pos = None
    if key == ord("w"):
      pos[1] -= 1
    elif key == ord("s"):
      pos[1] += 1
    elif key == ord("a"):
      pos[0] -= 1
    elif key == ord("d"):
      pos[0] += 1
    elif key == ord("c"):
      marker_color_i = (marker_color_i + 1) % len(marker_color_list)
    elif key == ord("x"):
      marker_size +=1
    elif key == ord("z"):
      if marker_size > 1:
        marker_size -=1
    elif key == 13: #Enter
      return pos

def background_subtraction(img1, img2):
  diff = img1.astype(np.float32) - img2.astype(np.float32)
  diff_sq = diff ** 2
  return np.sqrt(np.sum(diff_sq, axis = 2))

def compare_lighter(im1, im2):
  im3 = im1.copy()
  tf = np.sum(im1.astype(np.uint16), axis=2) < np.sum(im2.astype(np.uint16), axis=2)
  im3[tf] = im2[tf]
  return im3

def coordinate_transform(frm, pointmap):
  shape = pointmap.shape
  pm = pointmap.reshape(shape[0]*shape[1], shape[2]).T
  if isinstance(frm, FRAME):
    frm = frm.toarray()
  mat = frm[0:3, 0:3]
  vec = frm[0:3, 3:]

  newpm = (np.dot(mat, pm) + vec).T.reshape(shape)
  return newpm


def rm_small_objects(img, size, connectivity = 8):
  """
  Remove small objects from binary image
  MATLAB's bwareaopen
  """
  img2 = img.copy()
  label = labeling(img, connectivity)
  hist, bins = np.histogram(label, range(label.max()+2))
  for l, num in enumerate(hist):
    if num < size:
      img2[label == l] = 0
  return img2


def filter_objects_by_size(img, size_min, size_max, connectivity = 8):
  """
  Extract objects from binary image by size
  MATLAB's bwareafilt
  """
  img2 = np.zeros_like(img)
  label = labeling(img, connectivity)
  hist, bins = np.histogram(label, range(label.max()+2))
  for l, num in enumerate(hist):
    if l == 0:
      continue
    if (num > size_min) and (num <= size_max):
      img2[label == l] = 255
  return img2
