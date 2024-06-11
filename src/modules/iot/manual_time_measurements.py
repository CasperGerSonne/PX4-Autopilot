import time

Distances= []
timemessurements = []
counts = []

while True:
	user_dist = input("Distance: ")
	user_count = input("count: ")

	try:
		user_intdist= int(user_dist)
		user_intcount = int(user_count)
	except ValueError:
		print("Invalid input. Please enter an integer.")


	# Get the current Unix epoch time
	current_time = int(time.time())

	# Log the input and the time
	Distances.append(user_intdist)
	counts.append(user_count)
	timemessurements.append(current_time)
	print("Distance : at time ")
	for i in range(len(Distances)):
		print(f"{Distances[i]} : {counts[i]} : {timemessurements[i]}")

