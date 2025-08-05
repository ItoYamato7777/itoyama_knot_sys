import kinematics_hiro as kh
from geo import FRAME, MATRIX, VECTOR
from math import pi

ang1st = kh.angle_center
#solve forward kinema

frame_dic = kh.solve_fk(ang1st)
print "L_arm: ", FRAME(frm = frame_dic["L_arm"]).xyzrpy()
print "R_arm: ", FRAME(frm = frame_dic["R_arm"]).xyzrpy()
print "Neck:  ", FRAME(frm = frame_dic["Neck"]).xyzrpy()

# solve inverse kinema
## target position
tarR = FRAME(xyzrpy = [0.4, -0.3, 0.4, 0., 0., 0.]).toarray()
tarL = FRAME(xyzrpy = [0.4,  0.3, 0.4, 0., 0., 0.]).toarray()
## t_wrist2hand
w2h = FRAME(xyzrpy=[-0.25, 0., 0., pi, 0., pi])
w2h_a = w2h.toarray()

angles = kh.solve_ik(ang1st, tarR, tarL, w2h_a, w2h_a, None)

for angl in angles:
  frame_dic = kh.solve_fk(angl)
  R = FRAME(frm = frame_dic["R_arm"])
  L = FRAME(frm = frame_dic["L_arm"])
  print "R: ", (R*w2h).xyzrpy()
  print "L: ", (L*w2h).xyzrpy()
#  raw_input("Press Enter")
print angles[-1]


angles = kh.solve_ik(ang1st, tarR, tarL, w2h_a, w2h_a, 0.1)

for angl in angles:
  frame_dic = kh.solve_fk(angl)
  R = FRAME(frm = frame_dic["R_arm"])
  L = FRAME(frm = frame_dic["L_arm"])
  print "R: ", (R*w2h).xyzrpy()
  print "L: ", (L*w2h).xyzrpy()
#  raw_input("Press Enter")
print angles[-1]

tarR = FRAME(xyzrpy=[0.6, 0, 0.4, 0, pi/2, 0]).toarray()
angles = kh.solve_ik(ang1st, tarR, None, w2h_a, w2h_a, None)

for angl in angles:
  frame_dic = kh.solve_fk(angl)
  R = FRAME(frm = frame_dic["R_arm"])
  L = FRAME(frm = frame_dic["L_arm"])
  print "R: ", (R*w2h).xyzrpy()
  print "L: ", (L*w2h).xyzrpy()
#  raw_input("Press Enter")
print angles[-1]

#Speed check
tarR = FRAME(xyzrpy = [0.4, -0.3, 0.4, 0., 0., 0.]).toarray()
tarL = FRAME(xyzrpy = [0.4,  0.3, 0.4, 0., 0., 0.]).toarray()
import time
s = time.time()
for i in xrange(10):
  angles = kh.solve_ik(ang1st, tarR, tarL, w2h_a, w2h_a, None, cycle_max = 10000, th_lim = 0.5)
e = time.time()
print e-s
