#!/usr/bin/env python2

import rospy
from pathos.pools import ProcessPool

class DeclanTest:
	def __init__(self):
		self.names = ['Penne', 'Rigatoni']
		self.results = []

	def multitasking(self):
		print('Multitasking triggered.')
		pool=ProcessPool()
		r = pool.map(self.client, self.names)
		self.results = r

	def client(self, name, req=True):
		print('Client triggered: %s.' % name)
		return name

if __name__ == '__main__':
	rospy.init_node('test_pool', anonymous=True)

	dec = DeclanTest()
	dec.multitasking()

	print(dec.results)