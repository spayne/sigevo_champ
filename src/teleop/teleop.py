#!/usr/bin/env python
import roslib; roslib.load_manifest('sigevo_champ')
import rospy
import curses
from sigevo_champ.msg import CarControl 

def main(stdscr):
	stdscr.nodelay(1)
	pub = rospy.Publisher('torcs_car_control', CarControl)
	rospy.init_node('teleop')
	cc = CarControl() 
	cc.gear = 1
	brakeaccel = 0
	quit = False
	while not rospy.is_shutdown() and not quit:
		# get keyboard input, returns -1 if none available
		c = stdscr.getch()
		stdscr.addstr(0, 0, "w to accelerate, s to brake", curses.A_REVERSE)
		stdscr.addstr(1, 0, "a d to steer left and right", curses.A_REVERSE)
		stdscr.addstr(2, 0, "1 for 1st gear, r for reverse", curses.A_REVERSE)
		stdscr.addstr(3, 0, "q to quit", curses.A_REVERSE)
		stdscr.addstr(4, 0 , str(c), curses.A_REVERSE)

		
		publish = False
		if c == ord('w'):
			brakeaccel = min(1, brakeaccel + 0.1)
			publish = True 
		elif c == ord('s'):
			brakeaccel = max(-1, brakeaccel - 0.1)
			publish = True 
		elif c == ord('a'):
			cc.steer = min(1, cc.steer + 0.1)
			publish = True 
		elif c == ord('d'):
			cc.steer = max(-1, cc.steer - 0.1)
			publish = True 
		elif c == ord('q'):
			quit = True
		elif c == ord('r'):
			cc.gear = -1;
			publish = True 
		elif c == ord('1'):
			cc.gear = 1;
			publish = True 

		if publish:
			if brakeaccel > 0:
				cc.accel = brakeaccel;
				cc.brake = 0;
			else:
				cc.brake = -brakeaccel;
				cc.accel = 0;
			pub.publish(cc)
        rospy.sleep(0.5)

if __name__ == '__main__':
	try:
		curses.wrapper(main)
	except rospy.ROSInterruptException: pass

