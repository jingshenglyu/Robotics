#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i <this->numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i <this->numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

//generate uniformly distributed particles
void ParticleFilter::initParticlesUniform() {
    	//get map properties
    	int mapWidth, mapHeight;
    	double mapResolution;
    	this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

		// init uniform
		for (int i = 0; i <this->numberOfParticles; i++) {
			//particle position needs to be expressed in [meter] while the mapWidth and mapHeight are given in [pixel]
			// therefore the dimensions have to be multiplied by the resolution [meter/pixel]
			this->particleSet[i]->x = Util::uniformRandom(0,mapWidth*mapResolution);
			this->particleSet[i]->y = Util::uniformRandom(0,mapHeight*mapResolution);
			this->particleSet[i]->theta = Util::uniformRandom(-M_PI,M_PI);
			this->particleSet[i]->weight = 1.0 / this->getNumberOfParticles();
		}
		this->sumOfParticleWeights = 1.0;
}

//generate normal distributed particles
void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {

	// init Gaussian
	for (int i = 0; i <this->numberOfParticles; i++) {
		this->particleSet[i]->x = Util::gaussianRandom(mean_x, std_xx);
		this->particleSet[i]->y = Util::gaussianRandom(mean_y, std_yy);
		this->particleSet[i]->theta = Util::gaussianRandom(mean_theta, std_tt);
		this->particleSet[i]->weight = 1.0 / this->getNumberOfParticles();
	}
	this->sumOfParticleWeights = 1.0;
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
		const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
	ROS_INFO("Creating likelihood field for laser range finder...");

	// create the likelihood field - with the same discretization as the occupancy grid map
	this->likelihoodField = new double[map.info.height * map.info.width];
	this->likelihoodFieldWidth = map.info.width;
	this->likelihoodFieldHeight = map.info.height;
	this->likelihoodFieldResolution = map.info.resolution;

    // calculates the distance map and stores it in member variable 'distMap'
	// for every map position it contains the distance to the nearest occupied cell.
	calculateDistanceMap(map);

    // Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	double sigmaHit_scaled = sigmaHit / this->likelihoodFieldResolution;
	for (int w = 0; w < this->likelihoodFieldWidth; w++) {
		for (int h=0; h < this->likelihoodFieldHeight; h++) {
			int index = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, w, h);

			double z_rand = zRand;
			double p_rand = 1.0;
			double z_hit = 1.0-zRand;
			//generate normal distribution with mean at the nearest obstacle
			double p_hit = Util::gaussian(distMap[index], sigmaHit_scaled, 0);

			this->likelihoodField[index] = log(z_hit * p_hit + z_rand * p_rand);
			}
	}


	ROS_INFO("...DONE creating likelihood field!");

}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {

	this->sumOfParticleWeights = 0.0;
	for (int i = 0; i < this->numberOfParticles; i++) {
		double weight = 0.0;
		//particle postions
		double particleX = this->particleSet[i]->x / this->likelihoodFieldResolution;
		double particleY = this->particleSet[i]->y / this->likelihoodFieldResolution;
		double particleTheta = this->particleSet[i]->theta;

		for (int j = 0; j <(int)laserScan->ranges.size(); j+=this->laserSkip) {
			//laser scan
			double range = laserScan->ranges[j];
			double laser_angle = laserScan->angle_min + laserScan->angle_increment * j;
			double map_angle = Util::normalizeTheta(laser_angle + particleTheta);
			//endpoint
			int x = (int)particleX + (int)(cos(map_angle) * range / this->likelihoodFieldResolution);
			int y = (int)particleY + (int)(sin(map_angle) * range / this->likelihoodFieldResolution);
			int idx = computeMapIndex(this->likelihoodFieldWidth, this->likelihoodFieldHeight, x, y);
			//Check whether the endpoint is inside the map. If not use a lower prob then the min prob inside the map
			if (x < 0 || y < 0 || x >= this->likelihoodFieldWidth || y >= this->likelihoodFieldHeight){
				weight += -10;
			} else {
				weight += this->likelihoodField[idx];
			}
		}
		this->sumOfParticleWeights += exp(weight);
		this->particleSet[i]->weight = exp(weight);
	}

	//normalize the weights
	for (int i = 0; i < this->numberOfParticles; i++) {
		if (this->sumOfParticleWeights != 0) {
			this->particleSet[i]->weight = this->particleSet[i]->weight / this->sumOfParticleWeights;
		} else {
			ROS_WARN("weigh = 0");
		}
	}
	
}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	//validate input (first iteration has NaN in it)
	
	if(std::isnan(oldX) || std::isnan(oldY) || std::isnan(oldTheta) ||
			std::isnan(newX) || std::isnan(newY) || std::isnan(newTheta)) {
		ROS_WARN("Not a valid value");
		return;
	}

	//determine odometry information
	double delta_trans = sqrt( pow(newX-oldX,2) + pow(newY-oldY,2) );
	double delta_rot1 = Util::diffAngle(oldTheta, atan2( newY-oldY, newX-oldX));
	double delta_rot2 = Util::diffAngle(delta_rot1, Util::diffAngle(oldTheta, newTheta));


	for (int i = 0; i <this->numberOfParticles; i++) {
		//add uncertainty
		double d_rot1_hat = Util::gaussianRandom(delta_rot1,
			this->odomAlpha1 * abs(delta_rot1) + this->odomAlpha2 * delta_trans);

		double d_trans_hat = Util::gaussianRandom(delta_trans,
			this->odomAlpha3 * delta_trans + this->odomAlpha4 * (abs(Util::normalizeTheta(delta_rot1 + delta_rot2))));

		double d_rot2_hat = Util::gaussianRandom(delta_rot2,
			this->odomAlpha1 * abs(delta_rot2) + this->odomAlpha2 * delta_trans);

		//assign new states
		this->particleSet[i]->x = this->particleSet[i]->x + d_trans_hat * cos(this->particleSet[i]->theta + d_rot1_hat);
		this->particleSet[i]->y = this->particleSet[i]->y + d_trans_hat * sin(this->particleSet[i]->theta + d_rot1_hat);
		this->particleSet[i]->theta = Util::normalizeTheta(Util::normalizeTheta(this->particleSet[i]->theta + d_rot1_hat) + d_rot2_hat);
	}
	
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
	
	int nofP = this->getNumberOfParticles();
	std::vector<Particle> pSetNew;
	pSetNew.reserve(nofP);

	double r = Util::uniformRandom(0, 1.0/nofP);
	double u = 0;
	double c = this->particleSet[0]->weight;
	int i = 0;
	double maxWeight = 0;

	for(int j = 0; j < nofP; j++)
	{
		//increment threshold
		u = r + (j) * 1.0/nofP;

		while(u > c)
		{
			i = i + 1;
			// update cdf
			c = c + this->particleSet[i]->weight;
		}

		Particle part(this->particleSet[i]->x,this->particleSet[i]->y,this->particleSet[i]->theta,0);

		// update the bestHypothesis (the particle with maxWeight)
		if(maxWeight < this->particleSet[i]->weight)
		{
			maxWeight = this->particleSet[i]->weight;
			this->bestHypothesis->x = this->particleSet[i]->x;
			this->bestHypothesis->y = this->particleSet[i]->y;
			this->bestHypothesis->theta = this->particleSet[i]->theta;
		}

		// insert
		pSetNew[j] = part;
	}

	//replace the old particles with the the new ones
	for(int i = 0; i < nofP; i++)
	{
		this->particleSet[i]->x = pSetNew[i].x;
		this->particleSet[i]->y = pSetNew[i].y;
		this->particleSet[i]->theta = pSetNew[i].theta;
		this->particleSet[i]->weight = 1.0 / this->getNumberOfParticles();
	}
	
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

