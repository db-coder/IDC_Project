Group 7: 
--------------------------------------------------------------------------------------------------------------------------------------------------------

	Members				|	Roll Number
	-----------------------------------
	Utkarsh Kumar		|	130050022
	Ankit Rathore		|	130050029
	Ashish Anand		|	130050035
	Dibyendu Mondal		|	130050046

About the project:
--------------------------------------------------------------------------------------------------------------------------------------------------------
We firstly went on with our original idea of a Rube Goldberg Machine and executed it as planned. 
It is infact a "safety-mechanism"; something that might save you when a gun is pointed at you (provided you are not killed while the machine is running!). We have implemented multiple elements, that might be helpful for an animator, while not violating any of the Physics laws, hence making the animation seem more practical, while minimising the efforts.
Coming to the machine, the machine takes painfully long time to finish, but dont worry, because it is a Rube Goldberg Machine (and that's what it's supposed to do!). Once the first man kicks the ball away, it sets off a series of events, balloons drifting to the top, causing movement of balls, fall of dominos, vertical sticks moving, pulley system lifting up the ball and causing systematic displacement of balls, and finally a heavy ball smacking the gunman away! (Yay!)

But later we realised it might not be "helpful enough" as an animation tool, hence we made a rain model as a backup for the project. Here we have emulated the rain-drops using spheres, that are falling from a great height, and hence entering the frame with initial non-zero velocities. The initial coordinates of the balls are all randomised, thus making it more realistic. We could not implement spattering of rain due to time constraint, but we have tried to account for it by bouncing back of the droplets from the ground.


We have tried to animate rain using the GLUI graphics of box2D. Apart from the default rain, we can easily change the speed and duration of the rain as follows:
-> Change the speed:
   In /src/dominos.cpp, in the for loops starting from line no. 149 decrease the no. of iterations and change the value of the radius and density of the particles to change the speed.
-> Change the duration:
   In /src/dominos.cpp, to increase the duration of the rain copy the for loop starting from line 149 and paste it after the loop ends. To decrease the duaration delete on for loop.



How to execute:
--------------------------------------------------------------------------------------------------------------------------------------------------------
-> In terminal, cd to the project root directory containing src.
-> make
-> ./mybins/id408_exelibs_07

