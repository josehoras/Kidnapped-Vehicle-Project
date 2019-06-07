# Kidnapped Vehicle Project

Self-Driving Car Engineer Nanodegree Program

In this project I implemented a Particle Filter to accurately estimate the position of a car based on distance measurements to predetermined landmarks in the map. 

A number of particles are defined, which will be updated on each step. Particles with low probability to be close to the car position according to the measurements to the landmark, will be randomly discarded and instead particles with higher probability will be resampled. The accuracy of the filter is measured by the RMSE value of the highest probability particle.

The simulation delivers a great accuracy within the time specified in the [project rubric](https://review.udacity.com/#!/rubrics/1965/view).

<figure>
	<img src="./final_screenshot.png" width="40%" height="40%" />
</figure>

## Code desdription

A starter code is given by the Udacity project contained in `/src`. To implement the full particle filter only the file `particle_filter.cpp` needed to be completed. The main functions, and steps in the implementation, are:

- init(): Initializes the particle randomly in a Gaussian distribution around the car's position given by the GPS.
The Gaussian distribution is implemented using `default_random_engine`, that picks values from the given distribution, and the distribution `normal_distribution`
```
num_particles = 50;

std::default_random_engine gen;
normal_distribution<double> dist_x(x, std[0]);
  	...
for (int i = 0; i < num_particles; ++i) {
	....
	p.x = dist_x(gen);
	....
	particles.push_back(p);
}
ParticleFilter::is_initialized = true;
```

	Choosing	 even 9 particles was already enough to finish the simulation successfully, so 50 particles contains a big buffer without still compromising on execution speed. The simulation began to last more than the specified 100 seconds with 1200 particles, on my computer.

- prediction(): Updates the particles' position according to the motion model, and introducing the random Gaussian noise.
- updateWeights(): Assigns weights to the particles according to the probability to be at the correct car's position.
- resample(): Chooses which particles pass to the next cycle, with repetition, according to their weights.






## Installation and Build

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Once you clone this repo, it includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

- Make a build directory: `mkdir build && cd build`
- Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
- Run it: `./ExtendedKF `

Refer to the [Udacity project repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) for more detail intalation instructions.

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
