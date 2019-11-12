#!/usr/bin/env python2

# two_agents = {'edges': [['Penne','Rigatoni']], \
# 			'bearing': ((0,-1,0),) }

two_agents = {'edges': [['Penne','Rigatoni'], \
						['Penne','Linguine'], \
						['Penne','Gnocchi'], \
						['Linguine','Rigatoni'], \
						['Linguine','Gnocchi'], \
						['Rigatoni','Gnocchi']], \

			# ## Inverted triangular pyramid
			# 'bearing': ((2,0,0), \
			# 			(1,1,0), \
			# 			(1,0.5,-1), \
			# 			(1,-1,0), \
			# 			(0,-0.5,-1), \
			# 			(-1,0.5,-1)) }

			## Triangular pyramid
			# 'bearing': ((-2,0,0), \
			# 			(-1,-1,0), \
			# 			(-1,-0.5,1), \
			# 			(-1,1,0), \
			# 			(0,0.5,1), \
			# 			(1,-0.5,1)) }

			## Trapozhedra
			'bearing': ((-2,0,0), \
						(-1,1,1), \
						(-1,-1,1), \
						(-1,-1,-1), \
						(0,-2,0), \
						(1,-1,1)) }